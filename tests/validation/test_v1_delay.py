# Copyright 2026 The SAS Sensor Plugin Authors
# SPDX-License-Identifier: Apache-2.0
"""V1: target delay on every element, gpu_ray and ray (REQ-001, REQ-002, REQ-009; BUG-001)."""

import pytest

import sas_tools
from conftest import PLATE_A, PLATE_A_NORMAL, front, load_pings

# 1.5 samples = 2.8 cm < range resolution 3.75 cm. A plate's voxels are jittered +-1.6 cm along
# the line of sight (+-0.85 sample), which moves the speckled peak; a single scatterer is
# checked to 0.1 sample in test_synth.
TOL_SAMPLES = 1.5
MIN_PEAK_TO_MEDIAN = 10.0  # target must stand out of the seabed clutter (BUG-001 regression)


@pytest.mark.parametrize('bag', ['v1_gpu', 'v1_ray'])
def test_v1_delay(bag):
    msg = load_pings(bag)[-1]
    p = front(PLATE_A, PLATE_A_NORMAL)
    expected = [sas_tools.delay(msg, p, k) * msg.sample_rate for k in range(msg.num_elements)]
    peaks = sas_tools.element_peaks(msg, expected)
    for k, ((meas, _, snr), e) in enumerate(zip(peaks, expected)):
        print(f'{bag} k={k:2d} expected {e:8.2f} measured {meas:8.2f} diff {meas - e:+5.2f} '
              f'peak/median {snr:5.1f}')
    for k, ((meas, _, snr), e) in enumerate(zip(peaks, expected)):
        assert abs(meas - e) <= TOL_SAMPLES, f'element {k}'
        assert snr >= MIN_PEAK_TO_MEDIAN, f'element {k}'
