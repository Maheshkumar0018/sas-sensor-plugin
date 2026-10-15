# 0001 — Sensor-only scope

- Status: Accepted
- Date: 2026-10-15
- REQs: all

## Context

The original design (`reference/SAS_Gazebo_ROS2_Implementation.md`) combined the sonar sensor, an
INS-drift model, a trajectory plugin with motion errors, and processing nodes (backprojection,
waterfall) in one workspace. These parts change for different reasons and have different users.

## Decision

This repository builds only the sensor plugin and what is needed to run and verify it. The
following move to a separate downstream package:

- navigation / INS error and drift (old F10, `pose_nav`)
- vehicle trajectory and sway / heave / yaw disturbances (old F11)
- matched filtering, micronavigation, motion compensation
- image formation (old F13), waterfall (old F14), PSF and focus tests

## Consequences

- The sensor behaves like hardware: raw echoes plus ground truth.
- The `SasPing` message is the only coupling between the two packages, so it is frozen (0003).
- Validation here checks physics of the echo, not image quality.
