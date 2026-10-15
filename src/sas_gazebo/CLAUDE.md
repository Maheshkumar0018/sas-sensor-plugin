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
- Use sim time for anything that reaches the output. `steady_clock` is allowed only for
  diagnostic timing logs.
- FFTW plans and chirp spectrum created once in the `EchoSynthesizer` constructor, not per ping.
  Plans use `FFTW_ESTIMATE` (MEASURE makes rounding depend on timing).
- Design checks (range ambiguity, DPCA) only warn; they never change output.
- Never read link pose or velocity in `OnScan()`. Physics keeps stepping during the callback; use
  the per-step state history looked up at `LastMeasurementTime()` (ADR-0010).
- Throttled logs use `steady_clock_`, not the node clock (sim time, 0 without `/clock`).

## Style

- C++17, `gazebo::` plugin conventions, `RCLCPP_*` logging through the `gazebo_ros::Node`.
- Physics lives in the Gazebo-free `sas_core` library (`dsp.hpp`, `scatter.hpp/.cpp`,
  `synth.hpp/.cpp`) so gtest can call it without Gazebo. The plugin only reads Gazebo state and
  publishes.
- Comment each physics line with the equation it implements and the REQ ID.

## Tests

`test/` holds gtest unit tests for the free functions. System checks live in
`/tests/validation` (plan 008).
