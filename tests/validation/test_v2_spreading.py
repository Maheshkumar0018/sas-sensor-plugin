# Copyright 2026 The SAS Sensor Plugin Authors
# SPDX-License-Identifier: Apache-2.0
"""V2: spherical spreading and Thorp absorption with range (REQ-006, REQ-007).

Distributed target plate_c (4 x 4 m, ~100 range cells), carrier at 10, 15, 20, 25 m along the
same line of sight. Echo energy over the plate's delay window is compared with the model summed
over the plate's voxel facets. Energy averages the speckle that makes a single small plate's
coherent peak jump by several dB when the ray sampling of its edges changes.
"""

import numpy as np

import sas_tools
from conftest import PLATE_C_SIZE, PLATE_C_TOP, UP, load_pings

TOL_DB = 1.0
BAGS = {10: 'v2_r10', 15: 'v2_r15', 20: 'v2_r20', 25: 'v2_r25'}


def test_v2_spreading_and_absorption():
    facets = sas_tools.facet_grid(PLATE_C_TOP, PLATE_C_SIZE, 0.04)
    res = {}
    for r, bag in BAGS.items():
        msg = load_pings(bag)[-1]
        meas, win = sas_tools.energy_measured(msg, facets)
        res[r] = (meas, sas_tools.energy_model(msg, facets, UP), win)
    m0, d0, _ = res[15]
    for r, (m, d, win) in res.items():
        meas_db, model_db = 10 * np.log10(m / m0), 10 * np.log10(d / d0)
        print(f'R={r:2d} m window {win}: measured {meas_db:+7.2f} dB  model {model_db:+7.2f} dB  '
              f'diff {meas_db - model_db:+5.2f} dB')
    for r, (m, d, _) in res.items():
        assert abs(10 * np.log10(m / m0) - 10 * np.log10(d / d0)) <= TOL_DB, f'R={r}'
