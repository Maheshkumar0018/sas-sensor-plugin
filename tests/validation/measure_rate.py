#!/usr/bin/env python3
# Copyright 2026 The SAS Sensor Plugin Authors
# SPDX-License-Identifier: Apache-2.0
"""REQ-103 check: wall-clock time per ping and real-time factor, measured from outside.

Usage (sonar running, debug off): python3 tests/validation/measure_rate.py [--pings 40] [--skip 10]
RTF = (sim time between pings) / (wall time between pings), median over consecutive pairs.
"""

import argparse
import statistics
import sys
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sas_msgs.msg import SasPing


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--pings', type=int, default=40)
    ap.add_argument('--skip', type=int, default=10)
    ap.add_argument('--min-rtf', type=float, default=0.5)
    a = ap.parse_args()
    rclpy.init()
    node = rclpy.create_node('sas_measure_rate')
    got = []

    def on_ping(m):
        got.append((time.monotonic(), m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec))

    node.create_subscription(SasPing, '/sas/ping', on_ping,
                             QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE))
    while len(got) < a.skip + a.pings and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=1.0)
    s = got[a.skip:]
    wall = [b[0] - a_[0] for a_, b in zip(s, s[1:])]
    sim = [b[1] - a_[1] for a_, b in zip(s, s[1:])]
    rtf = statistics.median(si / wi for si, wi in zip(sim, wall))
    print(f'{len(wall)} intervals: wall per ping median {1e3 * statistics.median(wall):.1f} ms, '
          f'p90 {1e3 * sorted(wall)[int(0.9 * len(wall))]:.1f} ms; sim per ping '
          f'{1e3 * statistics.median(sim):.1f} ms; RTF {rtf:.2f} (target >= {a.min_rtf})')
    node.destroy_node()
    rclpy.shutdown()
    return 0 if rtf >= a.min_rtf else 1


if __name__ == '__main__':
    sys.exit(main())
