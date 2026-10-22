# Copyright 2026 The SAS Sensor Plugin Authors
# SPDX-License-Identifier: Apache-2.0
"""Bag loading for the plan 008 validation tests. Record bags with record_bags.sh first."""

import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
BAGS = os.path.join(HERE, 'bags')

# Scene constants shared with sas_bringup/worlds (see the world files).
PLATE_A = np.array([0.0, -14.2606, 4.8565])        # sas_point_target / sas_target_only
PLATE_A_NORMAL = -np.array([0.0, -0.9394, -0.3429])  # faces the carrier at (0, 0, 10), yaw 0
PLATE_B = np.array([40.0, -13.4551, 5.1507])       # sas_target_only, squinted
PLATE_C_TOP = np.array([20.0, -14.2606, 4.84])     # sas_target_only, 4 x 4 m horizontal
PLATE_C_SIZE = 4.0
UP = np.array([0.0, 0.0, 1.0])
PLATE_HALF_THICKNESS = 0.01                        # 0.02 m plates


def plate_b_normal():
    a = np.array([35.0, -0.17, 10.0])
    v = a - PLATE_B
    return v / np.linalg.norm(v)


def front(centre, normal):
    return centre + PLATE_HALF_THICKNESS * normal


def load_pings(name):
    """All SasPing messages in bags/<name>, in order. Skips the test if the bag is missing."""
    path = os.path.join(BAGS, name)
    if not os.path.isdir(path):
        pytest.skip(f'bag {name} not recorded (run tests/validation/record_bags.sh {name})')
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sas_msgs.msg import SasPing
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    msgs = []
    while reader.has_next():
        _, data, _ = reader.read_next()
        msgs.append(deserialize_message(data, SasPing))
    assert msgs, f'bag {name} is empty'
    return msgs
