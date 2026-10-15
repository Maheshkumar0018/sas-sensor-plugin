# 0011 — `gpu_ray` laser_retro is read from the raw laser frame

- Status: Accepted
- Date: 2026-10-15
- REQs: REQ-007
- Supersedes: —

## Context

In Gazebo 11, `GpuRaySensor::Retro(int)` is a stub that always returns 0.0 (gazebo-classic
`gazebo11` branch, `gazebo/sensors/GpuRaySensor.cc`). The renderer does compute retro: each
visual's `<laser_retro>` is written to the laser shader, and the second channel of the GPU laser
frame carries it. With `gpu_ray`, every voxel therefore got `default_backscatter_db` (BUG-001).

## Decision

- For `gpu_ray`, connect `GpuRaySensor::ConnectNewLaserFrame` and copy the range and retro
  channels (`data[i*depth]`, `data[i*depth+1]`) of each frame. The frame event fires in the
  rendering thread before the sensor's update event, in the same ray order as `Ranges()`.
- Before using the copy, check that it belongs to the current scan: same ray count and every
  finite in-range range equal within 1e-3 relative. On mismatch, warn (throttled) and keep the
  default μ for that ping.
- CPU `ray` keeps using `RaySensor::Retro()`, which works.
- Worlds set `<laser_retro>` on both visual (read by `gpu_ray`) and collision (read by `ray`).

## Consequences

- `gpu_ray` and `ray` give the same μ field (56 bright voxels on the plate target in both).
- Depends on the frame layout (range, retro, unused). The alignment check turns a layout change in
  a future Gazebo release into a visible warning, not silent wrong data.
