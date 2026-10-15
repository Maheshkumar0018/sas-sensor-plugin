# 0009 — Workspace-local FFTW dev files when root is not available

- Status: Accepted
- Date: 2026-10-15
- REQs: REQ-101
- Supersedes: —

## Context

`sas_gazebo` needs FFTW headers (`libfftw3-dev`). On the development workstation only the runtime
library `libfftw3-single3` is installed, and `sudo apt install libfftw3-dev` could not be run.

## Decision

- `scripts/setup_local_fftw.sh` downloads the `libfftw3-dev` package matching the installed runtime
  version with `apt download` (no root), unpacks it into `<workspace>/.deps/fftw`, points
  `libfftw3f.so` at the system runtime, and rewrites the pkg-config prefix.
- `sas_gazebo/CMakeLists.txt` adds that pkg-config path only when it exists and
  `/usr/include/fftw3.h` does not. A system install always wins.
- `.deps/` is ignored by git; the script is committed.
- `package.xml` keeps the normal rosdep key `libfftw3`.

## Consequences

- The plugin links the system `libfftw3f.so.3`; no `LD_LIBRARY_PATH` change at runtime.
- `rosdep check` still reports `libfftw3-dev` missing on this machine. That is expected while the
  workaround is in use.
- Other machines and CI should use `sudo apt install libfftw3-dev` (or `rosdep install`).
- Remove the workaround by installing `libfftw3-dev` system-wide and deleting `.deps/`.
