# 0005 — No custom trajectory plugin; stock mover for tests

- Status: Accepted
- Date: 2026-10-15
- REQs: REQ-008, REQ-011, REQ-014

## Context

The sensor needs motion to exercise REQ-008 and to test REQ-011, but vehicle motion is out of scope
(0001).

## Decision

- The sensor reads motion from its parent link and never moves itself.
- `sas_array.xacro` is a macro with a `parent` argument so any vehicle can mount it.
- Tests use `sas_test_carrier.urdf.xacro` (box, gravity off) driven by stock
  `libgazebo_ros_planar_move.so` with a constant `cmd_vel` from the launch file.

## Consequences

- No custom motion code to maintain here.
- Test motion is a straight constant-velocity leg only; disturbance cases are tested downstream.
