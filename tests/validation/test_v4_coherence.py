# Copyright 2026 The SAS Sensor Plugin Authors
# SPDX-License-Identifier: Apache-2.0
"""V4: coherence of the world-fixed scatterer field across pings (REQ-004, REQ-005).

DPCA test: at v = 0.2 m/s and PRI 0.1 s the array advances d/2 = 0.02 m per ping, so the two-way
phase centre of element k at ping n+1 coincides with that of element k+1 at ping n. With
world-fixed scatterers those two echoes must be nearly identical over the seabed. A field whose
speckle changed between pings would give correlation near 0.
"""

import numpy as np

import sas_tools
from conftest import load_pings

MIN_DPCA_CORR = 0.9


def corr(a, b):
    return np.abs(np.vdot(b, a)) / np.sqrt(np.vdot(a, a).real * np.vdot(b, b).real)


def test_v4_dpca_coherence():
    pings = load_pings('v4_dpca')
    v = sas_tools.velocity(pings[0])
    pri = 1.0 / 10.0
    d = pings[0].rx_positions[1].y - pings[0].rx_positions[0].y
    print(f'v = {v}, advance per ping {np.linalg.norm(v) * pri:.4f} m, d/2 = {d / 2:.4f} m')
    assert abs(np.linalg.norm(v) * pri - d / 2) < 1e-6
    fs, c = pings[0].sample_rate, pings[0].sound_speed
    lo, hi = int(2 * 12.0 / c * fs), int(2 * 45.0 / c * fs)  # seabed echoes, 12-45 m
    dpca, control = [], []
    for p0, p1 in zip(pings, pings[1:]):
        e0, e1 = sas_tools.echo(p0)[:, lo:hi], sas_tools.echo(p1)[:, lo:hi]
        for k in range(p0.num_elements - 1):
            dpca.append(corr(e1[k], e0[k + 1]))
            control.append(corr(e1[k], e0[k]))
    print(f'DPCA pairs: mean {np.mean(dpca):.4f} min {np.min(dpca):.4f} (n={len(dpca)})')
    print(f'control (same element, next ping): mean {np.mean(control):.4f}')
    assert np.min(dpca) >= MIN_DPCA_CORR
    assert np.mean(dpca) > np.mean(control)
