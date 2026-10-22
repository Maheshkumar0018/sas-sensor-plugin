# Copyright 2026 The SAS Sensor Plugin Authors
# SPDX-License-Identifier: Apache-2.0
"""V6: two runs with the same world, seed, and parameters are bit-identical (REQ-012).

Static carrier: a moving carrier started by `ros2 topic pub` begins moving at a sim time that
depends on wall-clock start-up, so its poses differ between runs (input, not sensor, differs).
Pings are matched by ping_id (recorded with --from-id). Stamps are not compared: the ping time
grid starts when spawn_entity inserts the carrier, which depends on wall-clock start-up too.
"""

import numpy as np
import pytest

from conftest import load_pings


@pytest.mark.parametrize('kind', ['gpu', 'ray'])
def test_v6_bit_identical(kind):
    a = load_pings(f'v6_{kind}_a')
    b = load_pings(f'v6_{kind}_b')
    by_id = {m.ping_id: m for m in b}
    common = [m for m in a if m.ping_id in by_id]
    print(f'{kind}: {len(common)} common ping ids')
    assert len(common) >= 3
    for ma in common:
        mb = by_id[ma.ping_id]
        assert ma.pose == mb.pose
        assert np.array_equal(np.asarray(ma.data_real), np.asarray(mb.data_real))
        assert np.array_equal(np.asarray(ma.data_imag), np.asarray(mb.data_imag))
        assert np.any(np.asarray(ma.data_real) != 0.0)
