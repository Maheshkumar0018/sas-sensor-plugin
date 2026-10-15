#!/usr/bin/env python3
# Copyright 2026 The SAS Sensor Plugin Authors
# SPDX-License-Identifier: Apache-2.0
"""Plan 005 exit check: matched-filter peak of a known target on every element.

Usage (sonar and world running, array static):
  python3 tests/validation/check_delay.py --target X Y Z --radius R

Array pose and element positions come from the message (SasPing.pose, rx_positions).
Prints per-element measured vs expected delay (samples) and exits non-zero on failure.
"""

import argparse
import sys

import numpy as np
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sas_msgs.msg import SasPing

sys.path.insert(0, __file__.rsplit('/', 1)[0])
import sas_tools  # noqa: E402


def quat_to_matrix(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--target', nargs=3, type=float, required=True)
    ap.add_argument('--radius', type=float, default=0.0,
                    help='distance from target centre to its front surface along the line of sight')
    ap.add_argument('--tol', type=float, default=2.0, help='samples')
    ap.add_argument('--skip', type=int, default=2, help='pings to skip after start-up')
    a = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node('sas_check_delay')
    got = []
    node.create_subscription(SasPing, '/sas/ping', got.append,
                             QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
    while len(got) <= a.skip and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=1.0)
    msg = got[-1]

    q = msg.pose.orientation
    rot = quat_to_matrix(q.x, q.y, q.z, q.w)
    origin = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
    rx = [origin + rot @ np.array([p.x, p.y, p.z]) for p in msg.rx_positions]
    centre = np.array(a.target)
    front = centre - a.radius * (centre - origin) / np.linalg.norm(centre - origin)

    y = sas_tools.matched_filter(msg)
    fs, c = msg.sample_rate, msg.sound_speed
    exp_all = [(np.linalg.norm(front - origin) + np.linalg.norm(front - r)) / c * fs for r in rx]
    lo, hi = int(min(exp_all) - 40), int(max(exp_all) + 40)
    ok = True
    print(f'ping {msg.ping_id}: N={msg.num_elements} Ns={msg.num_samples} window [{lo}, {hi})')
    print(' k   expected   measured   diff   peak/median(window)')
    for k, e in enumerate(exp_all):
        mag = np.abs(y[k, lo:hi])
        n = int(np.argmax(mag))
        m = lo + sas_tools.parabolic_peak(mag, n)
        snr = mag[n] / max(np.median(mag), 1e-30)
        good = abs(m - e) <= a.tol
        ok &= good
        print(f'{k:2d}  {e:9.2f}  {m:9.2f}  {m - e:+6.2f}  {snr:8.1f}  {"ok" if good else "FAIL"}')
    print(f'array origin ({origin[0]:.3f}, {origin[1]:.3f}, {origin[2]:.3f}), '
          f'boresight {np.round(rot[:, 0], 4)}')
    print('PASS' if ok else 'FAIL')
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
