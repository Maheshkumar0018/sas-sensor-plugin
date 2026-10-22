# Copyright 2026 The SAS Sensor Plugin Authors
# SPDX-License-Identifier: Apache-2.0
"""V5: receiver motion during propagation (REQ-008).

The static run is spawned at exactly the pose of moving ping 20 (record_bags.sh), so both pings
see the same scan, the same voxels, and the same speckle; only the velocity differs. The change
in per-element delay of the squinted plate_b must then equal the model's motion term.

The delay change is measured by complex cross-correlation of the moving and static matched-filter
outputs over the plate window, not by peak picking: the plate echo has several near-equal speckle peaks,
and a sub-sample change can flip which one is highest (seen as a 3-sample jump on one element).
"""

import numpy as np

import sas_tools
from conftest import PLATE_B, front, load_pings, plate_b_normal

TOL_SAMPLES = 0.1


HALF_WINDOW = 40  # samples around the plate echo


def test_v5_moving_receiver():
    static = load_pings('v5_static')[-1]
    moving = load_pings('v5_moving')
    m = moving[20]
    dp = np.array([m.pose.position.x - static.pose.position.x,
                   m.pose.position.y - static.pose.position.y,
                   m.pose.position.z - static.pose.position.z])
    print(f'moving ping {m.ping_id}: pose offset to static {dp} m, v={sas_tools.velocity(m)}')
    assert np.linalg.norm(dp) < 1e-6, 'static run not at the moving ping pose'
    v = sas_tools.velocity(m)
    p = front(PLATE_B, plate_b_normal())
    fs = m.sample_rate
    ym, ys = sas_tools.matched_filter(m), sas_tools.matched_filter(static)
    meas = np.zeros(m.num_elements)
    for k in range(m.num_elements):
        c = int(round(sas_tools.delay(static, p, k) * fs))
        w = slice(c - HALF_WINDOW, c + HALF_WINDOW)
        meas[k] = sas_tools.lag_samples(ys[k, w], ym[k, w])
    motion = np.array([(sas_tools.delay(m, p, k, v) - sas_tools.delay(m, p, k)) * fs
                       for k in range(m.num_elements)])
    for k in range(m.num_elements):
        print(f'k={k:2d} measured shift {meas[k]:+.3f}  motion model {motion[k]:+.3f}  '
              f'diff {meas[k] - motion[k]:+.3f} samples')
    assert np.all(np.abs(motion) > 0.15), 'motion term too small to test'
    assert np.all(np.abs(meas - motion) <= TOL_SAMPLES)
