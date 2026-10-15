# 005 — Propagation and echo synthesis

- Status: Done (2026-10-15)
- REQs: REQ-006, REQ-008, REQ-009, REQ-012
- Depends on: 004
- ADRs: 0002, 0004

## Goal

Raw multi-element echo with correct physics and reproducible noise.

## Files

`Synthesize()`, `ThorpDbPerMeter()`, `SolveDelay()` in `sas_sonar_plugin.cpp`;
`src/sas_gazebo/test/test_propagation.cpp`.

## Steps

1. Per scatterer, once per ping (`pre_rtx_`, `pre_amp_`): transmit range, transmit beam, Lambert
   `μ cos²θ A_cell`, source level.
2. Per element: one fixed-point iteration of `τ = (‖P−T‖ + ‖P−(R_k + v·τ)‖) / c`. Velocity input is
   zero until plan 006.
3. Receive beam, spreading `1/(R_tx R_rx)`, Thorp `10^{−α(R_tx+R_rx)/20}` (or SDF override), carrier
   phase `e^{−j2πf_c τ}`.
4. Deposit into per-element impulse train with 8-tap Hann-windowed sinc fractional delay.
5. FFT-multiply with `chirp_spec_` → raw echo.
6. Add complex Gaussian noise (`noise_std`) from a generator seeded by `(seed, ping_id, element)`.
7. OpenMP over elements.
8. No matched filter.
9. Unit tests: Thorp value at 100 kHz against reference; delay solver error < 1 µm at 3 m/s.

## Exit check

Point target, static carrier: offline matched filter of the raw echo peaks at the expected two-way
delay on every element, with per-element offsets matching array geometry.

## Progress log

- 2026-10-15: `include/sas_gazebo/synth.hpp` + `src/synth.cpp`: `ThorpDbPerMeter`,
  `SolveDelay`/`SolveDelayFromTx`, `synth::Params`, `EchoSynthesizer` (owns FFTW plans, element
  buffers, resident chirp spectrum; `Synthesize()` per ping). Model as in reference Section 6.2:
  per-scatterer tx range, tx/rx beams (along-track and vertical apertures), Lambert
  μ cos²θ A_cell, source level; per element moving-receiver delay, 1/R_rx spreading, Thorp,
  carrier phase e^{−j2πf_cτ}, 8-tap Hann-windowed sinc fractional delay, FFT × chirp spectrum,
  noise from `mt19937_64` seeded by (seed, ping_id, element). No matched filter (ADR-0002).
  OpenMP over scatterers (pre-terms) and over elements (one element per thread).
- 2026-10-15: `sas_core` static library (`scatter.cpp`, `synth.cpp`) now holds all physics; plugin
  only reads Gazebo state and publishes. New SDF params: `element_height`, `tx_height`
  (0.015 m, vertical beam only; not in `SasPing`, they affect amplitude, not focusing),
  `absorption_db_per_m` (default Thorp at fc: 0.0341 dB/m at 100 kHz).
- 2026-10-15: Correction to the reference: it claims one fixed-point iteration of the delay
  equation is accurate to < 1 µm. Measured: 38 µm path error at 3 m/s, 30 m (error ~ (v/c)·v·Δτ).
  `SolveDelayFromTx` now iterates twice (< 1e-6 m). Phase effect of the old error was ~λ/400.
- 2026-10-15: `test/test_synth.cpp`, 8 tests: Thorp at 100 kHz (34.0687 dB/km), delay solver
  exact when static and < 1 µm at 3 m/s, point-target delay on all 16 elements within 0.1 sample,
  spreading + absorption ratio 15 m vs 30 m within 0.05 dB, beam null suppresses target
  (< 1e-6), receiver motion shifts the delay as modelled within 0.1 sample, noise mean/std and
  per-ping reproducibility, bit-identical output with 1 vs 8 OpenMP threads.
  `colcon test --packages-select sas_gazebo`: 26 results, 0 failures.
- 2026-10-15: Exit check in `sas_smoke.world` with `tests/validation/check_delay.py` (offline
  matched filter from message fields only, `tests/validation/sas_tools.py`). Static array, plate
  target 0.6 × 0.6 m facing the sonar at 5.99 m: measured peak − expected = −0.58 … +0.78 samples
  on all 16 elements (tolerance ±2), peak/median 24–66. Passed.
- 2026-10-15: Finding: a sphere (r = 0.25 m) is not a usable point target in-sim. Its visible cap
  spans ~25 cm of range with hundreds of speckled voxels, and the peak moved −0 … +6 samples between
  elements. This is correct physics (speckle), not a delay error. Use facing plates for in-sim
  delay checks; plan 008 V1 should do the same.
- Velocity input is still zero; plan 006 feeds the link velocity.
- Timing (CPU ray 64×64, 16 elements, ~0.81 M voxels): build 150 ms, synthesis 240 ms per ping.
