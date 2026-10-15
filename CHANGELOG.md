# Changelog

All notable changes to this project are recorded here.
Format: [Keep a Changelog 1.1.0](https://keepachangelog.com/en/1.1.0/).
Versioning: [Semantic Versioning 2.0.0](https://semver.org/spec/v2.0.0.html).
A change to `sas_msgs/msg/SasPing.msg` fields is a breaking change (major version after 1.0.0).

Each entry is one line, written for a user of the sensor, and ends with its references:
`REQ-xxx`, `ADR-NNNN`, `BUG-NNN`, `plan NNN`.

Sections in this order, only when non-empty: Added, Changed, Deprecated, Removed, Fixed, Security.

## [Unreleased]

### Added
- `sas_description`: reusable `sas_array.xacro` macro and `sas_test_carrier.urdf.xacro` (stock `planar_move`). plan 007, REQ-014, ADR-0005
- `sas_bringup`: `sas_point_target.world`, `sas_seabed.world`, `sas_sensor.launch.py` (args world, sensor_type, prf, h_samples, v_samples, speed, x, y, z, gui, debug). plan 007, REQ-102
- Debug summary (`debug_scatterers` / `debug:=true`) logs raw retro and μ range per ping. plan 007
- `SasPing.pose`, `velocity`, and stamp now carry the ground-truth array state at the scan's measurement time, from a per-physics-step state history. plan 006, REQ-011, ADR-0010
- Strict-rate gate (`strict_rate`, default on for CPU `ray`) keeps one ping per PRI; `gpu_ray` uses `gzserver --lockstep`. plan 006, REQ-012, ADR-0010
- Design-check warnings: range ambiguity at load, DPCA along-track sampling per ping, ping interval vs PRI. plan 006, REQ-013
- `tests/validation/check_state.py` (stamp, pose, velocity check). plan 006
- `sas_gazebo`: raw echo synthesis (`synth.hpp`/`synth.cpp`, `EchoSynthesizer`): beams, Lambert, spherical spreading, Thorp absorption, moving-receiver delay (two iterations), carrier phase, band-limited fractional delay, chirp convolution, seeded receiver noise, OpenMP; 8 gtests. SDF params `element_height`, `tx_height`, `absorption_db_per_m`. plan 005, REQ-003, REQ-006, REQ-008, REQ-009, REQ-012
- `tests/validation/sas_tools.py` (offline matched filter from message fields) and `check_delay.py` (per-element delay check). Smoke world has a plate target. plan 005
- `sas_gazebo`: world-fixed scatterer field (`scatter.hpp`/`scatter.cpp`): surface reconstruction from the ray image, shadow rejection, voxels centred on multiples of `cell_size` with hash-derived jitter and speckle, μ from `<laser_retro>`; 10 gtests; SDF params `cell_size`, `max_incidence_deg`, `default_backscatter_db`, `debug_scatterers`. plan 004, REQ-004, REQ-005, REQ-007
- Smoke world can move along-track via `/sas/cmd_vel` (stock `planar_move`). plan 004
- `scripts/setup_local_fftw.sh`: FFTW dev files into `.deps/` without root; CMake uses them only when no system install exists. ADR-0009
- `sas_gazebo`: `dsp.hpp` helpers (chirp, beam pattern, array positions, parameter validation) with gtest `test_dsp`; `sas_sonar_plugin` skeleton publishing zero-echo `SasPing`; smoke world. plan 003, REQ-001, REQ-002, REQ-003
- `sas_msgs/msg/SasPing.msg` (frozen contract) and freeze test `test_sasping_contract.py`. plan 002, REQ-010, ADR-0003
- Apache-2.0 `LICENSE`; all packages declare `Apache-2.0`. ADR-0008
- Package shells `sas_msgs`, `sas_gazebo`, `sas_description`, `sas_bringup` (version 0.0.0) with dependencies declared. plan 001, REQ-101
- Project structure: `CLAUDE.md`, `README.md`, `docs/`, `src/`, `tests/`, shared `.claude/` config. ADR-0006
- Requirements REQ-001…014 and REQ-101…103. `docs/requirements.md`
- Architecture overview and traceability matrix. `docs/architecture.md`, `docs/traceability.md`
- Implementation plans 000–009 for the sensor-only build. `docs/plans/`
- Decision records ADR-0001…0011 and decision log. `docs/decisions/`
- Change-tracking policy: changelog, decision log, bug records. ADR-0007
- Git commit policy in `CLAUDE.md` and `.gitignore`.
- Project overview deck, self-contained HTML, 11 slides. `docs/presentations/sas_sensor_overview.html`

### Changed
- Scope reduced to the sensor plugin only. Navigation error, trajectory, and processing move to a downstream package. ADR-0001
- `SasPing` revised: `pose_true` renamed to `pose`; `source_level`, `noise_std`, `element_length`, `tx_length` added. ADR-0003
- Sensor outputs raw echo only. ADR-0002

### Fixed
- `gpu_ray` ignored `<laser_retro>`: Gazebo 11 `GpuRaySensor::Retro()` always returns 0. Retro is now read from the raw laser frame with an alignment check. BUG-001, ADR-0011, REQ-007
- Overview deck stated 40 kHz bandwidth; default is 20 kHz (δr = c/2B = 3.75 cm). `docs/presentations/sas_sensor_overview.html`

### Removed
- `pose_nav` and `range_compressed` from `SasPing`. ADR-0003
- `sas_trajectory_plugin`, `sas_processing` package, and processing config from the plan. ADR-0001, ADR-0005

<!-- Release links go here once a remote exists, e.g.
[Unreleased]: https://<host>/<repo>/compare/v0.1.0...HEAD
-->
