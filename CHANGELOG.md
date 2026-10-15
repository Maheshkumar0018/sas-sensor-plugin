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
- `scripts/setup_local_fftw.sh`: FFTW dev files into `.deps/` without root; CMake uses them only when no system install exists. ADR-0009
- `sas_gazebo`: `dsp.hpp` helpers (chirp, beam pattern, array positions, parameter validation) with gtest `test_dsp`; `sas_sonar_plugin` skeleton publishing zero-echo `SasPing`; smoke world. plan 003, REQ-001, REQ-002, REQ-003
- `sas_msgs/msg/SasPing.msg` (frozen contract) and freeze test `test_sasping_contract.py`. plan 002, REQ-010, ADR-0003
- Apache-2.0 `LICENSE`; all packages declare `Apache-2.0`. ADR-0008
- Package shells `sas_msgs`, `sas_gazebo`, `sas_description`, `sas_bringup` (version 0.0.0) with dependencies declared. plan 001, REQ-101
- Project structure: `CLAUDE.md`, `README.md`, `docs/`, `src/`, `tests/`, shared `.claude/` config. ADR-0006
- Requirements REQ-001…014 and REQ-101…103. `docs/requirements.md`
- Architecture overview and traceability matrix. `docs/architecture.md`, `docs/traceability.md`
- Implementation plans 000–009 for the sensor-only build. `docs/plans/`
- Decision records ADR-0001…0009 and decision log. `docs/decisions/`
- Change-tracking policy: changelog, decision log, bug records. ADR-0007
- Git commit policy in `CLAUDE.md` and `.gitignore`.
- Project overview deck, self-contained HTML, 11 slides. `docs/presentations/sas_sensor_overview.html`

### Changed
- Scope reduced to the sensor plugin only. Navigation error, trajectory, and processing move to a downstream package. ADR-0001
- `SasPing` revised: `pose_true` renamed to `pose`; `source_level`, `noise_std`, `element_length`, `tx_length` added. ADR-0003
- Sensor outputs raw echo only. ADR-0002

### Fixed
- Overview deck stated 40 kHz bandwidth; default is 20 kHz (δr = c/2B = 3.75 cm). `docs/presentations/sas_sensor_overview.html`

### Removed
- `pose_nav` and `range_compressed` from `SasPing`. ADR-0003
- `sas_trajectory_plugin`, `sas_processing` package, and processing config from the plan. ADR-0001, ADR-0005

<!-- Release links go here once a remote exists, e.g.
[Unreleased]: https://<host>/<repo>/compare/v0.1.0...HEAD
-->
