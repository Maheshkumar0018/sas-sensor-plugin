#!/usr/bin/env bash
# Copyright 2026 The SAS Sensor Plugin Authors
# SPDX-License-Identifier: Apache-2.0
#
# Record the plan 008 validation bags into tests/validation/bags/ (git-ignored).
# Usage: tests/validation/record_bags.sh [scenario-name ...]   (default: all)
# Needs a display for gpu_ray (DISPLAY=:1 on the dev workstation). Takes ~15 min for all.
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$WS"
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u  # after sourcing: ROS setup scripts read unset variables
BAGS=tests/validation/bags
LOGS=$BAGS/logs
mkdir -p "$LOGS"

# name | count | launch args [| recorder args]
SCENARIOS=(
  "v1_gpu|5|world:=sas_point_target.world sensor_type:=gpu_ray speed:=0.0 x:=0.0"
  "v1_ray|5|world:=sas_point_target.world sensor_type:=ray h_samples:=256 v_samples:=256 speed:=0.0 x:=0.0"
  "v2_r10|3|world:=sas_target_only.world sensor_type:=gpu_ray speed:=0.0 x:=20.0 y:=-4.6966 z:=8.2855"
  "v2_r15|3|world:=sas_target_only.world sensor_type:=gpu_ray speed:=0.0 x:=20.0 y:=0.0 z:=10.0"
  "v2_r20|3|world:=sas_target_only.world sensor_type:=gpu_ray speed:=0.0 x:=20.0 y:=4.6974 z:=11.7145"
  "v2_r25|3|world:=sas_target_only.world sensor_type:=gpu_ray speed:=0.0 x:=20.0 y:=9.3944 z:=13.4290"
  "v3_y000|3|world:=sas_target_only.world sensor_type:=gpu_ray speed:=0.0 x:=20.0 yaw:=0.00"
  "v3_y005|3|world:=sas_target_only.world sensor_type:=gpu_ray speed:=0.0 x:=20.0 yaw:=0.05"
  "v3_y010|3|world:=sas_target_only.world sensor_type:=gpu_ray speed:=0.0 x:=20.0 yaw:=0.10"
  "v3_y015|3|world:=sas_target_only.world sensor_type:=gpu_ray speed:=0.0 x:=20.0 yaw:=0.15"
  "v3_y020|3|world:=sas_target_only.world sensor_type:=gpu_ray speed:=0.0 x:=20.0 yaw:=0.20"
  "v3_y025|3|world:=sas_target_only.world sensor_type:=gpu_ray speed:=0.0 x:=20.0 yaw:=0.25"
  "v3_y030|3|world:=sas_target_only.world sensor_type:=gpu_ray speed:=0.0 x:=20.0 yaw:=0.30"
  "v4_dpca|12|world:=sas_seabed.world sensor_type:=gpu_ray speed:=0.2"
  "v5_moving|40|world:=sas_target_only.world sensor_type:=gpu_ray speed:=1.5 x:=30.0"
  "v5_static|3|world:=sas_target_only.world sensor_type:=gpu_ray speed:=0.0 x:=@POSEX:v5_moving"
  "v5_static_p10|3|world:=sas_target_only.world sensor_type:=gpu_ray speed:=0.0 x:=@POSEX:v5_moving:10"
  "v5_static_p15|3|world:=sas_target_only.world sensor_type:=gpu_ray speed:=0.0 x:=@POSEX:v5_moving:15"
  "v5_moving_ray|40|world:=sas_target_only.world sensor_type:=ray h_samples:=256 v_samples:=256 speed:=1.5 x:=30.0"
  "v5_static_ray|3|world:=sas_target_only.world sensor_type:=ray h_samples:=256 v_samples:=256 speed:=0.0 x:=@POSEX:v5_moving_ray"
  "v6_gpu_a|5|world:=sas_seabed.world sensor_type:=gpu_ray speed:=0.0|--from-id 30"
  "v6_gpu_b|5|world:=sas_seabed.world sensor_type:=gpu_ray speed:=0.0|--from-id 30"
  "v6_ray_a|5|world:=sas_seabed.world sensor_type:=ray h_samples:=256 v_samples:=256 speed:=0.0|--from-id 30"
  "v6_ray_b|5|world:=sas_seabed.world sensor_type:=ray h_samples:=256 v_samples:=256 speed:=0.0|--from-id 30"
  "v7_noise|5|world:=sas_empty.world sensor_type:=gpu_ray speed:=0.0"
)

want=" $* "
rc=0
for row in "${SCENARIOS[@]}"; do
  IFS='|' read -r name count args rec_args <<<"$row"
  if [ $# -gt 0 ] && [[ "$want" != *" $name "* ]]; then continue; fi
  if [[ "$args" =~ @POSEX:([a-z0-9_]+)(:([0-9]+))? ]]; then
    # V5: static carrier at exactly the pose of ping IDX (default 20) of the named moving bag, so
    # the scan (and the voxels) are identical and only the velocity differs.
    tok=${BASH_REMATCH[0]}; src=${BASH_REMATCH[1]}; idx=${BASH_REMATCH[3]:-20}
    px=$(python3 tests/validation/bag_pose_x.py "$BAGS/$src" "$idx") || { echo "$name: need $src"; rc=1; continue; }
    args=${args//$tok/$px}
  fi
  log=$LOGS/$name.log
  # shellcheck disable=SC2086
  setsid ros2 launch sas_bringup sas_sensor.launch.py $args > "$log" 2>&1 &
  lp=$!
  for _ in $(seq 1 120); do grep -q "SAS sonar ready" "$log" && break; sleep 1; done
  if grep -q "SAS sonar ready" "$log"; then
    # Moving scenarios: let the carrier reach constant speed before recording.
    skip=2; [[ "$args" == *"speed:=0.0"* ]] || skip=15
    if [ -n "$rec_args" ]; then sel=$rec_args; else sel="--skip $skip"; fi
    # shellcheck disable=SC2086
    timeout 900 python3 tests/validation/record_pings.py "$BAGS/$name" --count "$count" \
      $sel || rc=1
  else
    echo "$name: sonar not ready, see $log"; rc=1
  fi
  grep -q "world file not found" "$log" && { echo "$name: world file not found"; rc=1; }
  kill -INT -- -"$lp" 2>/dev/null; sleep 8; kill -9 -- -"$lp" 2>/dev/null; sleep 2
done
exit $rc
