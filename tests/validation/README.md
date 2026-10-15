# System validation (plan 008)

pytest checks V1–V7 run against rosbag2 recordings of `/sas/ping`. They do not start Gazebo.

Recording (example, V1):

```bash
ros2 launch sas_bringup sas_sensor.launch.py world:=sas_point_target.world speed:=0.0 &
ros2 bag record -o tests/validation/bags/v1_point_static /sas/ping
```

Bags go in `tests/validation/bags/` and are not committed. See `docs/plans/008-sensor-validation.md`
for each check's setup and pass criterion.
