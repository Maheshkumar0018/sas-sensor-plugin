# 007 — Description, test worlds, launch

- Status: Not started
- REQs: REQ-014, REQ-102
- Depends on: 006
- ADRs: 0005

## Goal

Reusable array macro, test carrier, test worlds, one launch file.

## Files

`src/sas_description/urdf/sas_array.xacro`, `src/sas_description/urdf/sas_test_carrier.urdf.xacro`,
`src/sas_bringup/worlds/sas_point_target.world`, `src/sas_bringup/worlds/sas_seabed.world`,
`src/sas_bringup/launch/sas_sensor.launch.py`.

## Steps

1. `sas_array.xacro`: macro with `parent` arg; array link, fixed joint, ray fan (horizontal ±0.40 rad,
   vertical −0.80…+0.20 rad), `sensor_type` arg (`gpu_ray` | `ray`), plugin block driven by xacro args.
2. `sas_test_carrier.urdf.xacro`: box, gravity off, array starboard with 20° depression; stock
   `libgazebo_ros_planar_move.so`.
3. Worlds:
   - `sas_point_target.world`: flat seabed + one high-retro sphere.
   - `sas_seabed.world`: seabed + cylinder + targets with `<laser_retro>`
     (sand 0.002, rock 0.02, metal 0.3, point 1.0).
4. `sas_sensor.launch.py`: `gzserver --lockstep`, `world` arg, `robot_state_publisher`,
   `spawn_entity`, `speed` arg published once as constant `cmd_vel`. No processing nodes.

## Exit check

World loads with no plugin errors; `ros2 topic hz /sas/ping` ≈ PRF with both `gpu_ray` and `ray`.
