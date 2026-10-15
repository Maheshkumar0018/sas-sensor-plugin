# sas_gazebo — plugin rules

Scope: one sensor plugin, `libsas_sonar_plugin.so`. No model plugins, no motion code, no
processing (ADRs 0001, 0005).

## Must

- Raw echo only. No matched filter, no range compression (ADR 0002).
- `pose` and `velocity` are ground truth from the parent link. Never perturb them.
- Scatterer jitter and speckle come only from `SplitMix64(CellKey ^ seed)` (ADR 0004).
- Noise generator seeded from `(seed, ping_id, element)`; each OpenMP thread uses its own.
- Results must not depend on thread count or order. No floating-point reductions across threads
  into shared buffers.
- Use sim time, never wall-clock time.
- FFTW plans and chirp spectrum created once in `SetupDsp()`, not per ping.
- Design checks (range ambiguity, DPCA) only warn; they never change output.

## Style

- C++17, `gazebo::` plugin conventions, `RCLCPP_*` logging through the `gazebo_ros::Node`.
- Pure maths helpers (`Sinc`, `ThorpDbPerMeter`, `CellKey`, `SplitMix64`, `SolveDelay`, chirp
  generation) are free functions in a header so gtest can call them without Gazebo.
- Comment each physics line with the equation it implements and the REQ ID.

## Tests

`test/` holds gtest unit tests for the free functions. System checks live in
`/tests/validation` (plan 008).
