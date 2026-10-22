# Copyright 2026 The SAS Sensor Plugin Authors
# SPDX-License-Identifier: Apache-2.0
"""V5: receiver motion during propagation (REQ-008).

Each static run is spawned at exactly the pose of one moving ping (record_bags.sh @POSEX), so both
see the same scan, voxels, and speckle; only the velocity differs. The change in per-element delay
of the squinted plate_b must equal the model's motion term.

The delay change is measured by complex cross-correlation of the moving and static matched-filter
outputs over the plate window, not by peak picking (near-equal speckle peaks flip).

Estimator noise depends on the speckle realization: gpu_ray realizations at moving pings 10, 20,
30 gave worst residuals 0.053, 0.141, 0.056 sample, the 0.14 bump in only one of them, means
-0.04, +0.04, +0.03 (plan 009 log). The gpu_ray criterion therefore averages three realizations
per element (pings 10, 15, 20; ping 30 is too near broadside, motion term < 0.1, and ping 0 puts
the plate outside the ray fan); each realization must still stay within 0.2.
"""

import numpy as np
import pytest

import sas_tools
from conftest import PLATE_B, front, load_pings, plate_b_normal

TOL_SAMPLES = 0.1
SANITY_SAMPLES = 0.2
HALF_WINDOW = 40  # samples around the plate echo


def residuals(moving_bag, idx, static_bag):
    static = load_pings(static_bag)[-1]
    m = load_pings(moving_bag)[idx]
    dp = np.array([m.pose.position.x - static.pose.position.x,
                   m.pose.position.y - static.pose.position.y,
                   m.pose.position.z - static.pose.position.z])
    assert np.linalg.norm(dp) < 1e-6, f'{static_bag} not at the pose of {moving_bag}[{idx}]'
    v = sas_tools.velocity(m)
    p = front(PLATE_B, plate_b_normal())
    fs = m.sample_rate
    ym, ys = sas_tools.matched_filter(m), sas_tools.matched_filter(static)
    meas, motion = np.zeros(m.num_elements), np.zeros(m.num_elements)
    for k in range(m.num_elements):
        c = int(round(sas_tools.delay(static, p, k) * fs))
        w = slice(c - HALF_WINDOW, c + HALF_WINDOW)
        # The plate must be seen: echo window energy >= 20 dB above the noise-only level.
        noise_e = 2 * HALF_WINDOW * (static.noise_std ** 2) * len(sas_tools.chirp(static)) * 2
        assert np.sum(np.abs(ys[k, w]) ** 2) > 100 * noise_e, f'no plate echo, element {k}'
        meas[k] = sas_tools.lag_samples(ys[k, w], ym[k, w])
        motion[k] = (sas_tools.delay(m, p, k, v) - sas_tools.delay(m, p, k)) * fs
    print(f'{moving_bag}[{idx}] vs {static_bag}: v={v}')
    for k in range(m.num_elements):
        print(f'  k={k:2d} measured shift {meas[k]:+.3f}  motion model {motion[k]:+.3f}  '
              f'diff {meas[k] - motion[k]:+.3f} samples')
    # A sensor without motion modelling (shift 0) must fail: motion term beyond the tolerance.
    assert np.all(np.abs(motion) > TOL_SAMPLES), 'motion term too small to test'
    return meas - motion


def test_v5_moving_receiver_gpu():
    res = np.array([residuals('v5_moving', 10, 'v5_static_p10'),
                    residuals('v5_moving', 20, 'v5_static'),
                    residuals('v5_moving', 15, 'v5_static_p15')])
    mean = res.mean(axis=0)
    print('mean over realizations:', ' '.join(f'{r:+.3f}' for r in mean))
    assert np.all(np.abs(res) <= SANITY_SAMPLES)
    assert np.all(np.abs(mean) <= TOL_SAMPLES)


def test_v5_moving_receiver_ray():
    res = residuals('v5_moving_ray', 20, 'v5_static_ray')
    assert np.all(np.abs(res) <= TOL_SAMPLES)


@pytest.mark.parametrize('bag', ['v5_static', 'v5_static_p10', 'v5_static_p15', 'v5_static_ray'])
def test_v5_static_is_static(bag):
    assert np.linalg.norm(sas_tools.velocity(load_pings(bag)[-1])) == 0.0
