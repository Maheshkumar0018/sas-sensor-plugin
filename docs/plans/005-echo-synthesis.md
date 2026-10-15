# 005 — Propagation and echo synthesis

- Status: Not started
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
