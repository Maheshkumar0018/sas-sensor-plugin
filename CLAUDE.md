# CLAUDE.md — SAS Sensor Plugin

Operating manual for Claude in this repository. Read this first in every session.

## What this project is

A Gazebo Classic 11 sensor plugin that simulates a Synthetic Aperture Sonar (SAS) receiver array.
It publishes raw multi-element complex baseband echoes and the ground-truth platform state at
transmit time on `sas_msgs/SasPing`.

**Sensor only.** Navigation error, INS drift, vehicle trajectory and disturbances, matched
filtering, motion compensation, image formation, and waterfall displays belong to a separate
downstream package. Do not add any of them here. See `docs/decisions/0001-sensor-only-scope.md`.

## Stack

Ubuntu 22.04, ROS 2 Humble, Gazebo Classic 11 (`gazebo_ros_pkgs`), C++17, FFTW3, OpenMP.
Python 3.10 only for launch files and validation scripts.

## Layout

```
CLAUDE.md, README.md, CHANGELOG.md
.claude/            settings.json, commands/ (/plan, /review, /trace)
docs/
  requirements.md   REQ-xxx IDs — source of truth for what to build
  architecture.md   module map, data flow, interfaces
  traceability.md   REQ → ADR → plan → code → test
  decisions/        ADRs (NNNN-title.md) + README.md decision log index
  bugs/             bug records (BUG-NNN-title.md) + README.md index
  plans/            implementation plans (NNN-title.md), one per phase
  reference/        theory + original reference implementation (read-only input)
  presentations/    stakeholder decks (self-contained HTML)
src/                colcon packages: sas_msgs, sas_gazebo, sas_description, sas_bringup
tests/              cross-package system validation (V1–V7), pytest on recorded bags
```

This directory is the colcon workspace root. `src/` holds ROS 2 packages. Unit tests live in each
package's own `test/` directory (ament convention). `tests/` at the root is for system-level checks
that span packages.

## Commands

```bash
scripts/setup_local_fftw.sh               # only if libfftw3-dev is not installed (ADR-0009)
source /opt/ros/humble/setup.bash
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1   # see "Environment notes"
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
colcon test && colcon test-result --verbose          # unit tests
pytest tests/validation                              # system validation (needs recorded bags)
ros2 launch sas_bringup sas_sensor.launch.py         # run sensor in test world
```

## Environment notes

- This machine has a `langsmith` pytest plugin in `~/.local` that crashes on import (urllib3 2.x vs
  system `requests_toolbelt`). With it, `python3 -m pytest` fails and ament skips every pytest test
  with "The Python module 'pytest' was not found". Export `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` before
  `colcon build` and `colcon test`. If a pytest target is missing, rebuild with
  `--cmake-force-configure`.

- `libfftw3-dev` is not installed system-wide here and sudo is unavailable. FFTW headers come
  from `.deps/fftw` via `scripts/setup_local_fftw.sh` (ADR-0009). `rosdep check` reporting
  `libfftw3-dev` missing is expected.
- Smoke run without launch: `GAZEBO_PLUGIN_PATH=$PWD/install/sas_gazebo/lib gzserver --lockstep
  src/sas_gazebo/test/worlds/sas_smoke.world`; move it with
  `ros2 topic pub -r 10 /sas/cmd_vel geometry_msgs/msg/Twist "{linear: {y: 1.5}}"`.
  `timeout` does not always stop `gzserver`; stop it with `pkill -INT -f <world>` and check
  `pgrep -a gzserver`, or the next run fails with "Address already in use".

## Working rules

1. **Requirements first.** Every change maps to a `REQ-xxx` in `docs/requirements.md`. If no
   requirement covers it, stop and ask; do not invent scope.
2. **Follow the active plan.** Work from `docs/plans/NNN-*.md`. Do one phase at a time. A phase is
   done only when its exit check passes. Update the plan's status line when a phase finishes.
3. **Decisions go in ADRs.** Any choice that changes the interface, scope, or a physical model gets
   a new file in `docs/decisions/`. Do not edit an accepted ADR; supersede it with a new one.
4. **Keep traceability current.** When code or a test for a REQ lands, update
   `docs/traceability.md` in the same change.
5. **`SasPing` is a frozen contract.** Changing `sas_msgs/msg/SasPing.msg` needs an ADR, because
   the downstream package depends on it.
6. **Determinism.** Same world + seed + parameters must give bit-identical output. No stateful
   RNG in scatterer generation; no thread-order-dependent results.
7. **Ground truth only.** The plugin never corrupts `pose` or `velocity`.
8. **Reference doc is input, not spec.** `docs/reference/SAS_Gazebo_ROS2_Implementation.md`
   contains theory and code, including downstream parts that are out of scope. Where it conflicts
   with `docs/requirements.md` or an ADR, requirements and ADRs win.
9. Release builds for any timing measurement; Debug is about 10× slower.
10. **Record every change** (ADR-0007), in the same commit as the change:
    - user-visible change → one line under `[Unreleased]` in `CHANGELOG.md` with refs
      (`REQ-xxx`, `ADR-NNNN`, `BUG-NNN`, `plan NNN`)
    - new or superseded decision → ADR file + row in `docs/decisions/README.md` + changelog line
    - defect in landed code → `docs/bugs/BUG-NNN-*.md` + index row + regression test +
      changelog line under `Fixed`
    - commit message: Conventional Commits, scope = package, footer `Refs: REQ-xxx, BUG-NNN`

## Git: what to commit

Policy is enforced by `.gitignore`. Update both together.

**Commit (shared):**
- `CLAUDE.md`, `README.md`, `CHANGELOG.md`, `.gitignore`
- `.claude/settings.json`, `.claude/commands/`
- `docs/` — requirements, architecture, traceability, ADRs, bug records, plans, reference
- `src/` — package sources, `package.xml`, `CMakeLists.txt`, msg, urdf/xacro, worlds, launch,
  config, unit tests, module `CLAUDE.md` files
- `tests/` — validation scripts and their README

**Keep local (ignored):**
- colcon output: `build/`, `install/`, `log/`
- recorded data: `tests/validation/bags/`, `*.db3`, `*.mcap`, `*.bag`, `*.npy`, `*.npz`
- tool output: `__pycache__/`, `.pytest_cache/`, coverage, `perf.data`, core dumps
- personal Claude config: `.claude/settings.local.json`, `CLAUDE.local.md`
- IDE/editor files: `.vscode/`, `.idea/`, `.cache/`, `compile_commands.json`, swap files
- secrets: `.env`

Rules:
- Before staging, run `git status` and check that no file from the "keep local" list appears.
- Never commit large binaries. Share reference bags (plan 008) outside git and record where they
  are in `tests/validation/README.md`.
- A new kind of generated or personal file → add it to `.gitignore` and to this list in the same
  change.

## Module-specific rules

- `src/sas_gazebo/CLAUDE.md` — plugin implementation rules.
- `src/sas_msgs/CLAUDE.md` — interface change rules.
