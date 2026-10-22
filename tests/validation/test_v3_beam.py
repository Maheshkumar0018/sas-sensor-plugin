# Copyright 2026 The SAS Sensor Plugin Authors
# SPDX-License-Identifier: Apache-2.0
"""V3: transmit x receive beam pattern versus along-track angle (REQ-003).

The carrier is yawed so the distributed plate_c moves off the boresight. Echo energy over the
plate is compared with the model summed over the plate's facets (beams, spreading, Lambert,
absorption at the reported pose). Model-relative levels below MIN_MODEL_DB are near the beam
null and are not asserted.
"""

import numpy as np

import sas_tools
from conftest import PLATE_C_SIZE, PLATE_C_TOP, UP, load_pings

TOL_DB = 1.0
YAWS = {0.00: 'v3_y000', 0.05: 'v3_y005', 0.10: 'v3_y010', 0.15: 'v3_y015',
        0.20: 'v3_y020', 0.25: 'v3_y025', 0.30: 'v3_y030'}
MIN_MODEL_DB = -25.0


def test_v3_beam_pattern():
    facets = sas_tools.facet_grid(PLATE_C_TOP, PLATE_C_SIZE, 0.04)
    res = {}
    for y, bag in YAWS.items():
        msg = load_pings(bag)[-1]
        _, rot, tx, _ = sas_tools.array_geometry(msg)
        u = rot.T @ ((PLATE_C_TOP - tx) / np.linalg.norm(PLATE_C_TOP - tx))
        res[y] = (sas_tools.energy_measured(msg, facets)[0],
                  sas_tools.energy_model(msg, facets, UP), u[1])
    m0, d0, _ = res[0.0]
    asserted = 0
    for y, (m, d, s) in res.items():
        meas_db, model_db = 10 * np.log10(m / m0), 10 * np.log10(d / d0)
        print(f'yaw {y:4.2f} sin(az) {s:+.3f}: measured {meas_db:+7.2f} dB  model '
              f'{model_db:+7.2f} dB  diff {meas_db - model_db:+5.2f} dB')
    for y, (m, d, _) in res.items():
        model_db = 10 * np.log10(d / d0)
        if model_db >= MIN_MODEL_DB:
            assert abs(10 * np.log10(m / m0) - model_db) <= TOL_DB, f'yaw {y}'
            asserted += 1
    assert asserted >= 5
