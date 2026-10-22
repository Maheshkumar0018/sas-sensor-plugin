#!/usr/bin/env bash
# Copyright 2026 The SAS Sensor Plugin Authors
# SPDX-License-Identifier: Apache-2.0
#
# REQ-103 measurement: launch the sensor (debug off), measure wall time per ping from outside.
# Usage: tests/validation/measure_rtf.sh [min_rtf] [launch args...]
#   tests/validation/measure_rtf.sh 0.5                                        # REQ-103 defaults
#   tests/validation/measure_rtf.sh 0.0 sensor_type:=ray h_samples:=256 v_samples:=256
# Needs a display for gpu_ray (DISPLAY=:1 on the dev workstation).
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$WS"
source /opt/ros/humble/setup.bash
source install/setup.bash
MIN=${1:-0.5}; shift || true
LOG=$(mktemp)
setsid ros2 launch sas_bringup sas_sensor.launch.py world:=sas_seabed.world speed:=1.5 "$@" \
  > "$LOG" 2>&1 &
LP=$!
trap 'kill -INT -- -$LP 2>/dev/null; sleep 8; kill -9 -- -$LP 2>/dev/null; rm -f "$LOG"' EXIT
for _ in $(seq 1 120); do grep -q "SAS sonar ready" "$LOG" && break; sleep 1; done
grep -o "SAS sonar ready.*" "$LOG" || { echo "sonar not ready"; tail -20 "$LOG"; exit 1; }
timeout 600 python3 tests/validation/measure_rate.py --pings 60 --skip 10 --min-rtf "$MIN"
