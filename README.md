# SAS Sensor Plugin for Gazebo Classic 11 + ROS 2 Humble

## What

A physics-based Synthetic Aperture Sonar (SAS) sensor for Gazebo Classic. It simulates an LFM
transmitter and an N-element along-track receiver array. It publishes raw complex baseband echoes
for every element, once per ping, together with the true array pose and velocity at transmit time.

## Why

SAS processing (micronavigation, motion compensation, image formation) needs coherent raw data from
a known geometry. This plugin provides that data with full ground truth, so that processing
algorithms can be developed and tested against it.

This repository contains the **sensor only**. Navigation-error models, trajectories, and all
processing (matched filtering, backprojection, waterfall) live in a separate package that
subscribes to `/sas/ping`.

## Packages

| Package | Contents |
|---------|----------|
| `sas_msgs` | `SasPing` message |
| `sas_gazebo` | `libsas_sonar_plugin.so` |
| `sas_description` | `sas_array.xacro` macro, test carrier model |
| `sas_bringup` | test worlds, launch file |

## Build

```bash
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-xacro \
                 ros-humble-robot-state-publisher libfftw3-dev python3-numpy
source /opt/ros/humble/setup.bash
cd sas_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## Run

```bash
ros2 launch sas_bringup sas_sensor.launch.py                       # seabed world, carrier at 1.5 m/s
ros2 launch sas_bringup sas_sensor.launch.py world:=sas_point_target.world speed:=0.0
ros2 topic hz /sas/ping
```

## Test

```bash
colcon test && colcon test-result --verbose     # unit tests
pytest tests/validation                         # sensor validation V1–V7
```

## Documentation

- `docs/requirements.md` — requirements (REQ IDs)
- `docs/architecture.md` — design overview
- `docs/plans/` — implementation plans, phase by phase
- `docs/decisions/` — architecture decision records (index in `README.md`)
- `docs/bugs/` — bug records
- `CHANGELOG.md` — release history
- `docs/reference/` — SAS theory and reference implementation

## Status

Pre-implementation. See `docs/plans/000-roadmap.md`.
