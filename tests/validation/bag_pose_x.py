#!/usr/bin/env python3
# Copyright 2026 The SAS Sensor Plugin Authors
# SPDX-License-Identifier: Apache-2.0
"""Print the array x position of message INDEX in a SasPing bag with full precision."""

import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from sas_msgs.msg import SasPing

reader = rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id='sqlite3'),
            rosbag2_py.ConverterOptions('cdr', 'cdr'))
msgs = []
while reader.has_next():
    msgs.append(deserialize_message(reader.read_next()[1], SasPing))
print(repr(msgs[int(sys.argv[2])].pose.position.x))
