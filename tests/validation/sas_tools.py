# Copyright 2026 The SAS Sensor Plugin Authors
# SPDX-License-Identifier: Apache-2.0
"""Offline helpers for checking SasPing data. Test tooling only, not a processing node."""

import numpy as np


def echo(msg):
    """Raw echo as complex array of shape (num_elements, num_samples)."""
    n_el, ns = msg.num_elements, msg.num_samples
    re = np.asarray(msg.data_real, dtype=np.float32).reshape(n_el, ns)
    im = np.asarray(msg.data_imag, dtype=np.float32).reshape(n_el, ns)
    return re + 1j * im


def chirp(msg):
    """Transmit pulse p(t) = exp(j pi K (t - Tp/2)^2), rebuilt from message fields only."""
    fs, tp, bw = msg.sample_rate, msg.pulse_length, msg.bandwidth
    n = np.arange(int(np.ceil(tp * fs - 1e-9)))
    t = n / fs - 0.5 * tp
    return np.exp(1j * np.pi * (bw / tp) * t ** 2)


def matched_filter(msg):
    """Range-compressed echo (num_elements, num_samples); peak index n means delay n / fs."""
    x = echo(msg)
    p = chirp(msg)
    nfft = 1 << int(np.ceil(np.log2(x.shape[1] + p.size)))
    y = np.fft.ifft(np.fft.fft(x, nfft, axis=1) * np.conj(np.fft.fft(p, nfft)), axis=1)
    return y[:, : x.shape[1]]


def parabolic_peak(mag, n):
    """Fractional index of the peak at integer index n of magnitude array mag."""
    if n <= 0 or n + 1 >= mag.size:
        return float(n)
    a, b, c = mag[n - 1], mag[n], mag[n + 1]
    den = a - 2.0 * b + c
    return float(n) if den == 0 else n + 0.5 * (a - c) / den


# --- Geometry and models (from message fields + documented defaults) -------------------------

# Not in SasPing (amplitude-only); defaults from sas_array.xacro.
ELEMENT_HEIGHT = 0.015
TX_HEIGHT = 0.015


def quat_to_matrix(x, y, z, w):
    """Rotation matrix (array frame -> world) from a unit quaternion."""
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def array_geometry(msg):
    """(origin, rotation, tx_world, [rx_world]) at transmit, from SasPing.pose."""
    q = msg.pose.orientation
    rot = quat_to_matrix(q.x, q.y, q.z, q.w)
    o = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
    tx = o + rot @ np.array([msg.tx_position.x, msg.tx_position.y, msg.tx_position.z])
    rx = [o + rot @ np.array([p.x, p.y, p.z]) for p in msg.rx_positions]
    return o, rot, tx, rx


def velocity(msg):
    return np.array([msg.velocity.x, msg.velocity.y, msg.velocity.z])


def thorp_db_per_m(f_hz):
    f2 = (f_hz / 1000.0) ** 2
    return (0.11 * f2 / (1 + f2) + 44.0 * f2 / (4100 + f2) + 2.75e-4 * f2 + 0.003) / 1000.0


def delay(msg, p, k, vel=None):
    """Two-way delay [s] to point p for element k, receiver moving at vel (two iterations)."""
    _, _, tx, rx = array_geometry(msg)
    v = np.zeros(3) if vel is None else vel
    c = msg.sound_speed
    r_tx = np.linalg.norm(p - tx)
    tau = (r_tx + np.linalg.norm(p - rx[k])) / c
    for _ in range(2):
        tau = (r_tx + np.linalg.norm(p - (rx[k] + v * tau))) / c
    return tau


def amplitude(msg, p, normal, k, alpha_db_m=None):
    """Relative echo amplitude of a facet at p with unit normal, element k (SL, mu, area omitted).

    Same model as synth.cpp: cos(incidence) * b_tx * b_rx / (R_tx R_rx) * 10^(-alpha (R_tx+R_rx)/20).
    """
    _, rot, tx, rx = array_geometry(msg)
    lam = msg.sound_speed / msg.center_frequency
    alpha = thorp_db_per_m(msg.center_frequency) if alpha_db_m is None else alpha_db_m
    d = p - tx
    r_tx = np.linalg.norm(d)
    u = rot.T @ (d / r_tx)
    b_tx = np.sinc(msg.tx_length * u[1] / lam) * np.sinc(TX_HEIGHT * u[2] / lam)
    b_rx = np.sinc(msg.element_length * u[1] / lam) * np.sinc(ELEMENT_HEIGHT * u[2] / lam)
    cos_inc = max(0.0, -float(np.dot(normal, d / r_tx)))
    r_rx = np.linalg.norm(p - rx[k])
    return cos_inc * b_tx * b_rx / (r_tx * r_rx) * 10 ** (-alpha * (r_tx + r_rx) / 20.0)


def element_peaks(msg, expected_samples, half_window=40):
    """Per element: (fractional peak index, peak magnitude, peak/median) near expected delays."""
    y = matched_filter(msg)
    out = []
    for k, e in enumerate(expected_samples):
        lo = max(0, int(e - half_window))
        hi = min(y.shape[1], int(e + half_window))
        mag = np.abs(y[k, lo:hi])
        n = int(np.argmax(mag))
        frac = parabolic_peak(mag, n)
        if 0 < n < mag.size - 1:
            a, b, c = mag[n - 1], mag[n], mag[n + 1]
            d = frac - n
            peak = b - 0.25 * (a - c) * d
        else:
            peak = mag[n]
        out.append((lo + frac, float(peak), float(mag[n] / max(np.median(mag), 1e-30))))
    return out


# --- Distributed targets: energy instead of a speckled coherent peak --------------------------

def facet_grid(centre, size, cell):
    """Facet centres on a horizontal size x size square at `centre`, one per voxel column."""
    n = int(round(size / cell))
    offs = (np.arange(n) - (n - 1) / 2.0) * cell
    xx, yy = np.meshgrid(offs, offs)
    pts = np.stack([xx.ravel() + centre[0], yy.ravel() + centre[1],
                    np.full(xx.size, centre[2])], axis=1)
    return pts


def energy_model(msg, facets, normal, alpha_db_m=None):
    """Expected echo energy (relative) of incoherent facets, mean over elements."""
    _, rot, tx, rx = array_geometry(msg)
    lam = msg.sound_speed / msg.center_frequency
    alpha = thorp_db_per_m(msg.center_frequency) if alpha_db_m is None else alpha_db_m
    d = facets - tx
    r_tx = np.linalg.norm(d, axis=1)
    u = (d / r_tx[:, None]) @ rot  # rows: direction in array frame
    b = (np.sinc(msg.tx_length * u[:, 1] / lam) * np.sinc(TX_HEIGHT * u[:, 2] / lam) *
         np.sinc(msg.element_length * u[:, 1] / lam) * np.sinc(ELEMENT_HEIGHT * u[:, 2] / lam))
    cos_inc = np.clip(-(d / r_tx[:, None]) @ normal, 0.0, None)
    e = 0.0
    for r in rx:
        r_rx = np.linalg.norm(facets - r, axis=1)
        a = cos_inc * b / (r_tx * r_rx) * 10 ** (-alpha * (r_tx + r_rx) / 20.0)
        e += np.sum(a ** 2)
    return e / len(rx)


def energy_measured(msg, facets, margin=10):
    """Matched-filter energy in the delay window spanned by the facets, mean over elements."""
    _, _, tx, rx = array_geometry(msg)
    c, fs = msg.sound_speed, msg.sample_rate
    r_tx = np.linalg.norm(facets - tx, axis=1)
    taus = [(r_tx + np.linalg.norm(facets - r, axis=1)) / c * fs for r in rx]
    lo = int(min(t.min() for t in taus)) - margin
    hi = int(max(t.max() for t in taus)) + margin
    y = matched_filter(msg)
    return float(np.mean(np.sum(np.abs(y[:, lo:hi]) ** 2, axis=1))), (lo, hi)


def lag_samples(a, b, upsample=512):
    """Sub-sample lag of b relative to a (positive: b later): peak of the magnitude of the complex
    cross-correlation, interpolated by zero-padding the cross-spectrum. Using the complex signal
    (not the envelope) keeps the phase structure and is less sensitive to speckle."""
    n = 1 << int(np.ceil(np.log2(2 * a.size)))
    spec = np.conj(np.fft.fft(a, n)) * np.fft.fft(b, n)
    big = np.zeros(n * upsample, dtype=complex)
    big[: n // 2] = spec[: n // 2]
    big[-n // 2:] = spec[-n // 2:]
    xc = np.abs(np.fft.ifft(big))
    k = int(np.argmax(xc))
    if k > n * upsample // 2:
        k -= n * upsample
    return k / upsample
