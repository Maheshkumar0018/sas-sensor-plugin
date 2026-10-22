#!/usr/bin/env python3
# Copyright 2026 The SAS Sensor Plugin Authors
# SPDX-License-Identifier: Apache-2.0
"""Record exactly N SasPing messages to a rosbag2 (sqlite3, cdr) after skipping K.

Usage: python3 tests/validation/record_pings.py OUT_DIR --count N [--skip K | --from-id ID]
       [--topic /sas/ping]
--from-id records pings ping_id >= ID, so separate runs can be compared ping by ping.
"""

import argparse
import shutil
import sys

import rclpy
import rosbag2_py
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.serialization import serialize_message
from sas_msgs.msg import SasPing


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('out')
    ap.add_argument('--count', type=int, required=True)
    ap.add_argument('--skip', type=int, default=2)
    ap.add_argument('--from-id', type=int, default=None)
    ap.add_argument('--topic', default='/sas/ping')
    a = ap.parse_args()

    shutil.rmtree(a.out, ignore_errors=True)
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=a.out, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    writer.create_topic(rosbag2_py.TopicMetadata(
        name=a.topic, type='sas_msgs/msg/SasPing', serialization_format='cdr'))

    rclpy.init()
    node = rclpy.create_node('sas_record_pings')
    state = {'seen': 0, 'written': 0}

    def on_ping(msg):
        state['seen'] += 1
        if state['written'] >= a.count:
            return
        if a.from_id is not None and msg.ping_id < a.from_id:
            return
        if a.from_id is None and state['seen'] <= a.skip:
            return
        t = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        writer.write(a.topic, serialize_message(msg), t)
        state['written'] += 1

    node.create_subscription(SasPing, a.topic, on_ping,
                             QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE))
    while state['written'] < a.count and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=1.0)
    del writer
    print(f'{a.out}: wrote {state["written"]} pings (skipped {a.skip})')
    node.destroy_node()
    rclpy.shutdown()
    return 0 if state['written'] == a.count else 1


if __name__ == '__main__':
    sys.exit(main())
