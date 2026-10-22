# System validation (plan 008)

pytest checks V1–V7 run against rosbag2 recordings of `/sas/ping`. They do not start Gazebo.

```bash
DISPLAY=:1 tests/validation/record_bags.sh          # all 21 scenarios, ~15 min
DISPLAY=:1 tests/validation/record_bags.sh v1_gpu   # or selected ones
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/validation -s
```

A test whose bag is missing is skipped. Scenarios and pass criteria: `docs/plans/008-sensor-validation.md`.

Recording (example, V1):

```bash
ros2 launch sas_bringup sas_sensor.launch.py world:=sas_point_target.world speed:=0.0 &
ros2 bag record -o tests/validation/bags/v1_point_static /sas/ping
```

Live checkers (run while the launch file is up): `check_delay.py` (per-element target delay,
plan 005/007), `check_state.py` (stamps, pose, velocity, plan 006/007). Shared helpers:
`sas_tools.py`.

Bags go in `tests/validation/bags/` (~50 MB) and are not committed. There is no shared copy yet;
regenerate them with `record_bags.sh`. See `docs/plans/008-sensor-validation.md`
for each check's setup and pass criterion.
