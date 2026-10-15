# 007 — Description, test worlds, launch

- Status: Done (2026-10-15)
- REQs: REQ-014, REQ-102
- Depends on: 006
- ADRs: 0005, 0010, 0011

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

## Progress log

- 2026-10-15: `sas_array.xacro` macro (args: parent, xyz, rpy, prefix, sensor_type, prf,
  h_samples, v_samples, num_elements, element_spacing, topic, seed, cell_size, noise_std, debug).
  `sas_test_carrier.urdf.xacro`: 1.0 × 0.3 × 0.3 m box, gravity off, array starboard with
  rpy (0, 0.35, −π/2) so array y = vehicle +x, stock `libgazebo_ros_planar_move` on `/sas/cmd_vel`.
  `check_urdf` parses it.
- 2026-10-15: Worlds: `sas_point_target.world` (seabed + 0.3 m plate at 15 m on the carrier
  boresight, facing the array; the plan said "sphere", replaced per the plan 005 finding) and
  `sas_seabed.world` (reference targets). `<laser_retro>` on both visual and collision.
- 2026-10-15: `sas_sensor.launch.py`: gazebo_ros `gazebo.launch.py` with `--lockstep`,
  robot_state_publisher, spawn_entity, constant `cmd_vel`; args world, sensor_type, prf,
  h_samples, v_samples, speed, x, y, z, gui, debug. Plugin path comes from the `sas_gazebo` export.
- 2026-10-15: `check_delay.py` now takes array pose and element positions from `SasPing`.
- 2026-10-15: BUG-001 found: `gpu_ray` gave no laser_retro (Gazebo 11 `GpuRaySensor::Retro()` is a
  stub returning 0). Fixed via the raw laser frame (ADR-0011). Debug summary now logs retro and μ.
- 2026-10-15: Exit check (launch file, no plugin errors in any run):
  - `gpu_ray`, 512 × 512 rays, point-target world, static at x = 0: `check_delay.py` PASS,
    16/16 elements −0.81 … +0.37 samples, peak/median 23–38.
  - `ray` (CPU, REQ-102), 256 × 256 rays, same world: PASS, 16/16 elements −0.80 … +1.31 samples,
    peak/median 12–34.
  - `gpu_ray` 512 × 512 and `ray` 256 × 256, seabed world, 1.5 m/s: `check_state.py` PASS,
    30 consecutive pings, stamp interval = 0.1 s (sim time) within 4e-16 s, pose advance = v·Δt
    within 6e-15 m, velocity = commanded.
  - "Rate ≈ PRF" is met in sim time. Wall-clock rates: 30 pings in 16.8 s (`gpu_ray` 512²,
    RTF ≈ 0.18) and 20.1 s (`ray` 256², RTF ≈ 0.15). `ros2 topic hz` measures wall time and will
    show ~1.5–2 Hz; that is expected under lockstep.
