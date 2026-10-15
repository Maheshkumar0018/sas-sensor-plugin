#!/usr/bin/env python3
# Copyright 2026 The SAS Sensor Plugin Authors
# SPDX-License-Identifier: Apache-2.0
"""Plan 006 exit check: ground-truth pose, velocity, and stamps in SasPing.

Usage (sonar moving at a known constant velocity):
  python3 tests/validation/check_state.py --vel VX VY VZ [--pings 20]

Checks per consecutive ping pair: stamp interval = PRI, pose advance = velocity * interval,
reported velocity = commanded velocity, orientation unchanged. Exits non-zero on failure.
"""

import argparse
import sys

import numpy as np
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sas_msgs.msg import SasPing


def t_of(m):
    return m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec


def p_of(m):
    return np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--vel', nargs=3, type=float, required=True)
    ap.add_argument('--pings', type=int, default=20)
    ap.add_argument('--pri', type=float, default=0.1)
    ap.add_argument('--pos-tol', type=float, default=1e-3, help='m per ping')
    ap.add_argument('--vel-tol', type=float, default=1e-2, help='m/s')
    a = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node('sas_check_state')
    got = []
    node.create_subscription(SasPing, '/sas/ping', got.append,
                             QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE))
    while len(got) < a.pings and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=1.0)
    v_cmd = np.array(a.vel)
    ok = True
    ids = [m.ping_id for m in got]
    ok &= ids == list(range(ids[0], ids[0] + len(ids)))
    print(f'pings {ids[0]}..{ids[-1]} consecutive: {ok}')
    dts, errs, verrs, qerrs = [], [], [], []
    for m0, m1 in zip(got, got[1:]):
        dt = t_of(m1) - t_of(m0)
        dts.append(dt)
        v_rep = np.array([m0.velocity.x, m0.velocity.y, m0.velocity.z])
        errs.append(np.linalg.norm((p_of(m1) - p_of(m0)) - v_cmd * dt))
        verrs.append(np.linalg.norm(v_rep - v_cmd))
        q0 = np.array([m0.pose.orientation.x, m0.pose.orientation.y,
                       m0.pose.orientation.z, m0.pose.orientation.w])
        q1 = np.array([m1.pose.orientation.x, m1.pose.orientation.y,
                       m1.pose.orientation.z, m1.pose.orientation.w])
        qerrs.append(1.0 - abs(float(q0 @ q1)))
    m = got[-1]
    print(f'last pose  p=({m.pose.position.x:.4f}, {m.pose.position.y:.4f}, '
          f'{m.pose.position.z:.4f}) q=({m.pose.orientation.x:.4f}, {m.pose.orientation.y:.4f}, '
          f'{m.pose.orientation.z:.4f}, {m.pose.orientation.w:.4f})')
    print(f'last vel   ({m.velocity.x:.4f}, {m.velocity.y:.4f}, {m.velocity.z:.4f})  '
          f'commanded {tuple(v_cmd)}')
    checks = [
        ('stamp interval = PRI', max(abs(d - a.pri) for d in dts), 1e-6),
        ('|dp - v*dt| per ping [m]', max(errs), a.pos_tol),
        ('|v_reported - v_cmd| [m/s]', max(verrs), a.vel_tol),
        ('orientation change', max(qerrs), 1e-9),
    ]
    for name, worst, tol in checks:
        good = worst <= tol
        ok &= good
        print(f'{name:28s} worst {worst:.3e}  tol {tol:.1e}  {"ok" if good else "FAIL"}')
    print('PASS' if ok else 'FAIL')
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
