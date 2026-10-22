#!/usr/bin/env python3
# Copyright 2026 The SAS Sensor Plugin Authors
# SPDX-License-Identifier: Apache-2.0
"""Check every recorded bag against its scenario in record_bags.sh: carrier pose (y, z, yaw) and
speed from SasPing must match the launch arguments. Catches bags that recorded the wrong run
(e.g. two recorders on /sas/ping at once). Exit 1 on any mismatch."""

import math
import os
import re
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from conftest import load_pings  # noqa: E402
import sas_tools  # noqa: E402

DEFAULTS = {'x': -10.0, 'y': 0.0, 'z': 10.0, 'yaw': 0.0, 'speed': 1.5}
ARRAY_OFFSET = np.array([0.0, -0.17, 0.0])  # sas_test_carrier mount, base frame


def scenarios():
    text = open(os.path.join(HERE, 'record_bags.sh')).read()
    for name, args in re.findall(r'^\s*"(v\d\w*)\|\d+\|([^"|]+)', text, re.M):
        kv = dict(DEFAULTS)
        for k, v in re.findall(r'(\w+):=(\S+)', args):
            if k in kv and not v.startswith('@'):
                kv[k] = float(v)
        yield name, kv


def main():
    bad = 0
    for name, kv in scenarios():
        try:
            pings = load_pings(name)
        except BaseException as e:  # pytest.skip raises a BaseException subclass
            print(f'{name:10s} MISSING ({e})')
            bad += 1
            continue
        m = pings[0]
        c, s = math.cos(kv['yaw']), math.sin(kv['yaw'])
        off = np.array([c * ARRAY_OFFSET[0] - s * ARRAY_OFFSET[1],
                        s * ARRAY_OFFSET[0] + c * ARRAY_OFFSET[1], 0.0])
        exp_y, exp_z = kv['y'] + off[1], kv['z']
        _, rot, _, _ = sas_tools.array_geometry(m)
        along = rot[:, 1]  # array y-axis in world = along-track
        yaw = math.atan2(along[1], along[0])
        v = np.linalg.norm(sas_tools.velocity(m))
        errs = [abs(m.pose.position.y - exp_y), abs(m.pose.position.z - exp_z),
                abs(math.remainder(yaw - kv['yaw'], 2 * math.pi)), abs(v - kv['speed'])]
        ok = errs[0] < 1e-3 and errs[1] < 1e-3 and errs[2] < 1e-3 and errs[3] < 1e-3
        if kv['speed'] == 0.0:
            ok &= abs(m.pose.position.x - (kv['x'] + off[0])) < 1e-3 or name.startswith('v5_static')
        bad += 0 if ok else 1
        print(f'{name:10s} {"ok " if ok else "BAD"} pings {len(pings):2d}  '
              f'p=({m.pose.position.x:8.3f},{m.pose.position.y:8.3f},{m.pose.position.z:7.3f}) '
              f'yaw {yaw:+.3f} |v| {v:.2f}  expected y {exp_y:.3f} z {exp_z:.3f} '
              f'yaw {kv["yaw"]:+.3f} |v| {kv["speed"]:.2f}')
    print('ALL OK' if bad == 0 else f'{bad} bag(s) do not match their scenario')
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
