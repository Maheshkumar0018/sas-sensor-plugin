# 001 — Workspace and toolchain

- Status: In progress — waiting on `libfftw3-dev` install (needs sudo)
- REQs: REQ-101
- Depends on: —

## Goal

Build scaffold for all four packages.

## Files

`src/sas_msgs/`, `src/sas_gazebo/`, `src/sas_description/`, `src/sas_bringup/` — `package.xml` and
`CMakeLists.txt` each. `.gitignore` for `build/ install/ log/`.

## Steps

1. Create each package with `package.xml` (format 3) and `CMakeLists.txt` (ament_cmake).
2. Install `ros-humble-gazebo-ros-pkgs`, `ros-humble-xacro`, `ros-humble-robot-state-publisher`,
   `libfftw3-dev`; `python3-numpy`, `python3-pytest` for validation.
   The rosdep key for FFTW is `libfftw3` (resolves to `libfftw3-3 libfftw3-dev`); `libfftw3-dev`
   is not a rosdep key.
3. Check OpenGL GPU for `gpu_ray`; otherwise plan for `sensor_type:=ray` (REQ-102).

## Exit check

`colcon build` succeeds on empty packages; `rosdep check --from-paths src --ignore-src` is clean.

## Progress log

- 2026-10-15: Four package shells created (`package.xml` format 3, `CMakeLists.txt` ament_cmake,
  version 0.0.0). Dependencies for later plans already declared so `rosdep check` covers them.
  `colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release`: 4 packages finished.
- 2026-10-15: Environment: Ubuntu 22.04.5, ROS 2 Humble, Gazebo 11.10.2, `gazebo_ros_pkgs` 3.9.0,
  xacro, robot_state_publisher, numpy, pytest installed. GPU: NVIDIA T1000 8GB, driver 535 →
  `gpu_ray` usable; `ray` fallback still tested in plan 007.
- 2026-10-15: `rosdep check` fails only on `libfftw3-3`, `libfftw3-dev` (not installed; install
  needs sudo).
- 2026-10-15: License set to Apache-2.0 in all `package.xml` files; `LICENSE` added. ADR-0008
