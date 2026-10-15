# System validation (plan 008)

pytest checks V1–V7 run against rosbag2 recordings of `/sas/ping`. They do not start Gazebo.

Recording (example, V1):

```bash
ros2 launch sas_bringup sas_sensor.launch.py world:=sas_point_target.world speed:=0.0 &
ros2 bag record -o tests/validation/bags/v1_point_static /sas/ping
```

Live checkers (run while the launch file is up): `check_delay.py` (per-element target delay,
plan 005/007), `check_state.py` (stamps, pose, velocity, plan 006/007). Shared helpers:
`sas_tools.py`.

Bags go in `tests/validation/bags/` and are not committed. See `docs/plans/008-sensor-validation.md`
for each check's setup and pass criterion.
