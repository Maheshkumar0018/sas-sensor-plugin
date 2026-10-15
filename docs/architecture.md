# Architecture

## System context

```
 ┌───────────────────────── Gazebo Classic 11 (lockstep) ─────────────────────────┐
 │                                                                                 │
 │  world (seabed, targets, <laser_retro>)                                        │
 │        │ ray cast                                                               │
 │        ▼                                                                        │
 │  <sensor type="gpu_ray"> ──► SasSonarPlugin (libsas_sonar_plugin.so)           │
 │        ▲                         │                                              │
 │        │ parent link pose/vel    │                                              │
 │  vehicle / test carrier          │                                              │
 │  (moved by something else)       │                                              │
 └──────────────────────────────────┼──────────────────────────────────────────────┘
                                    │  sas_msgs/SasPing on /sas/ping
                                    ▼
                    downstream processing package (not in this repo)
                    nav error, matched filter, imaging, waterfall
```

## Module map

| Package | Type | Responsibility | REQs |
|---------|------|----------------|------|
| `sas_msgs` | ament_cmake, rosidl | `SasPing` message definition | REQ-010 |
| `sas_gazebo` | ament_cmake, C++ | Sensor plugin: waveform, array, scatterers, synthesis, publishing | REQ-001…013 |
| `sas_description` | ament_cmake, xacro | `sas_array.xacro` macro; `sas_test_carrier.urdf.xacro` | REQ-002, REQ-014 |
| `sas_bringup` | ament_cmake | Test worlds, `sas_sensor.launch.py` | REQ-007, REQ-012 |
| `tests/validation` | pytest | System validation V1–V7 on recorded bags | all functional REQs |

## Plugin internals (`sas_gazebo/src/sas_sonar_plugin.cpp`)

```
Load()            parse SDF, validate, range-ambiguity check (REQ-013)
SetupDsp()        chirp spectrum, FFTW plans, rx positions        (REQ-001, 002)
OnScan()          per sensor update = one ping
 ├─ ReadState()   parent-link pose + velocity at transmit time    (REQ-011)
 ├─ ReadScan()    range image + retro
 ├─ BuildScatterers()  quads → shadows → voxels → hash speckle    (REQ-004, 005, 007)
 ├─ Synthesize()  per scatterer: spreading, Thorp, Lambert, beams (REQ-003, 006)
 │                per element: moving-receiver delay, frac. delay (REQ-008)
 │                FFT × chirp spectrum, + noise                   (REQ-009)
 ├─ DpcaCheck()   throttled warning                               (REQ-013)
 └─ Publish()     SasPing                                         (REQ-010)
```

## Interfaces

- **Output:** `sas_msgs/SasPing` on `topic` (default `/sas/ping`), one per ping. Definition in
  `src/sas_msgs/msg/SasPing.msg`; see `plans/002-sasping-message.md`.
- **Input:** SDF parameters on the plugin block (set via `sas_array.xacro` args); world geometry and
  `<laser_retro>`; parent link motion.
- **Frames:** array frame x = boresight, y = along-track, z = up. `pose` is array frame in world.

## Cross-cutting constraints

- Lockstep (`gzserver --lockstep`) so each sensor update yields exactly one ping (REQ-012).
- Hash-based randomness for scatterers; seeded per-(ping, element) noise streams (REQ-012).
- OpenMP across elements; per-scatterer terms precomputed once per ping and shared.

## Test strategy

| Level | Where | Tool | Examples |
|-------|-------|------|----------|
| Unit | `src/<pkg>/test/` | gtest via `colcon test` | chirp generation, `Sinc`, `ThorpDbPerMeter`, `CellKey`/`SplitMix64` stability, delay solver |
| System | `tests/validation/` | pytest + rosbag2 | V1–V7 (see `plans/008-sensor-validation.md`) |

## References

- Theory and equations: `reference/SAS_Gazebo_ROS2_Implementation.md` Sections 2.1–2.3.
- Original reference code (includes out-of-scope parts): same document, Sections 5–9.
