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
