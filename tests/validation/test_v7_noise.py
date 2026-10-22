# Copyright 2026 The SAS Sensor Plugin Authors
# SPDX-License-Identifier: Apache-2.0
"""V7: receiver noise in an empty world (REQ-009)."""

import numpy as np

import sas_tools
from conftest import load_pings


def test_v7_noise_floor():
    pings = load_pings('v7_noise')
    x = np.concatenate([sas_tools.echo(m).ravel() for m in pings])
    sigma = pings[0].noise_std
    s_re, s_im = np.std(x.real), np.std(x.imag)
    mean = np.mean(x)
    flat = sas_tools.echo(pings[0])[0]
    lag1 = np.abs(np.vdot(flat[:-1], flat[1:])) / np.vdot(flat, flat).real
    print(f'noise_std {sigma:.3e}: std re {s_re:.4e} im {s_im:.4e} |mean| {abs(mean):.2e} '
          f'lag-1 corr {lag1:.4f} ({x.size} samples)')
    assert abs(s_re / sigma - 1) < 0.02 and abs(s_im / sigma - 1) < 0.02
    assert abs(mean) < 0.01 * sigma
    assert lag1 < 0.02  # white
    seqs = [sas_tools.echo(m)[0, :100] for m in pings]
    assert not np.array_equal(seqs[0], seqs[1])  # fresh noise every ping
