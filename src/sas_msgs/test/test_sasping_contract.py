# Copyright 2026 The SAS Sensor Plugin Authors
# SPDX-License-Identifier: Apache-2.0
"""Freeze test for sas_msgs/SasPing (ADR-0003, REQ-010, REQ-011).

Fails when any field is added, removed, renamed, retyped, or reordered.
Changing EXPECTED requires a new ADR that supersedes ADR-0003.
"""

from sas_msgs.msg import SasPing

EXPECTED = [
    ('header', 'std_msgs/Header'),
    ('ping_id', 'uint64'),
    ('sound_speed', 'double'),
    ('center_frequency', 'double'),
    ('bandwidth', 'double'),
    ('pulse_length', 'double'),
    ('sample_rate', 'double'),
    ('t0', 'double'),
    ('source_level', 'double'),
    ('noise_std', 'double'),
    ('tx_position', 'geometry_msgs/Point'),
    ('rx_positions', 'sequence<geometry_msgs/Point>'),
    ('element_length', 'double'),
    ('tx_length', 'double'),
    ('pose', 'geometry_msgs/Pose'),
    ('velocity', 'geometry_msgs/Vector3'),
    ('num_elements', 'uint32'),
    ('num_samples', 'uint32'),
    ('data_real', 'sequence<float>'),
    ('data_imag', 'sequence<float>'),
]


def test_sasping_fields_frozen():
    assert list(SasPing.get_fields_and_field_types().items()) == EXPECTED


def test_element_major_layout_fits():
    msg = SasPing()
    msg.num_elements = 4
    msg.num_samples = 8
    n = msg.num_elements * msg.num_samples
    msg.data_real = [0.0] * n
    msg.data_imag = [0.0] * n
    k, i = 3, 7
    assert k * msg.num_samples + i == n - 1
