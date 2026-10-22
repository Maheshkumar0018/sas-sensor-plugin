# 009 — Performance tuning (optional)

- Status: Done (2026-10-22)
- REQs: REQ-103
- Depends on: 008

## Goal

Reach the performance target once REQ-103 has a value.

## Steps

1. Release build only for measurement.
2. Tune `cell_size`, `h_samples`/`v_samples`, `num_elements`, `max_range`
   (reference doc Section 12, sensor rows only).
3. CPU fallback: `sensor_type:=ray` with fewer rays.
4. Possible later: CUDA synthesis loop (needs ADR).

## Exit check

REQ-103 target met; V1–V7 still pass.

## Progress log

- 2026-10-22: REQ-103 set by the user: RTF ≥ 0.5 (≤ 0.2 s wall per ping) at the defaults on
  `gpu_ray`; CPU `ray` best effort.
- Per-stage profile (debug log, `sas_seabed.world`, 1.5 m/s, `gpu_ray` 512², ~0.90 M voxels):

  | Step | build ms | synth ms | wall period ms | RTF |
  |------|----------|----------|----------------|-----|
  | baseline | 309 | 270 | 609 | 0.16 |
  | fractional-delay taps from 3 trig calls (was 16) | 313 | 142 | 485 | 0.21 |
  | flat generation-stamped `KeySet` (was `std::unordered_set`) | 166 | 141 | 338 | 0.30 |
  | parallel build: per-row candidates, ordered merge, parallel hashing | 51 | 142 | 213 | 0.47 |
  | closed-form moving-receiver delay (1 sqrt, exact; was 2 iterations, 3 sqrt) | 52 | 127 | 202 | 0.50 |
  | 4 fixed scatterer chunks per element (64 tasks on 20 threads) | 48 | 107 | 186 | 0.54 |

- None of the steps changes the model. Determinism kept by construction and tested: new gtests
  `Kernel.FastTapsMatchDirectFormula` (≤ 1e-9; the old direct form loses ~1e-11 near f = 1),
  `KeySet.InsertClearGrow`, `Build.IndependentOfThreadCount` (1 vs 8 threads bit-identical);
  `DelaySolverStaticExactMovingSubMicron` tightened to < 1 nm. `colcon test`: 35, 0 failures.
- Exit check, REQ-103 (`tests/validation/measure_rtf.sh`, debug off, external subscriber, 59
  intervals): `gpu_ray` 512² median 187.0 ms per ping, p90 202.3 ms, **RTF 0.53** (target 0.5).
  CPU `ray` 256²: 330.0 ms, RTF 0.30 (was ~0.15); Gazebo's CPU ray casting dominates there.
- Exit check, V1–V7 re-run on bags re-recorded with the optimised plugin: 14 passed.
  - V5 needed work. On a new realization `gpu_ray` showed a 0.14-sample bump on elements 4–6.
    Hypothesis "GPU renders from a lagging pose" was tested and rejected: the laser camera position
    equals the physics state used for the ping within 2 µm (debug log `render-vs-state`). Three
    `gpu_ray` realizations (moving pings 10, 20, 30, each with an exact-pose static twin) gave
    worst residuals 0.053 / 0.141 / 0.056 sample with means −0.04 / +0.04 / +0.03, and CPU `ray`
    0.029: speckle-dependent estimator noise, not a model error. V5 `gpu_ray` now averages three
    realizations per element (pings 10, 15, 20; ping 30 is too near broadside and ping 0 puts the
    plate outside the ray fan), each realization ≤ 0.2; V5 `ray` keeps ≤ 0.1 on one realization.
    Result: mean residual ≤ 0.044 sample. V5 also asserts the plate echo is ≥ 20 dB above noise.
  - Recording incidents (test infrastructure, not sensor): a background recorder from the
    previous session restarted and ran beside a new one; `v2_r25` captured the other run's pings.
    New `check_bags.py` compares each bag's pose/yaw/speed with its scenario (now ALL OK).
    `v6_ray_b` started recording after ping 5 had passed; V6 now records from `ping_id` 30.
- Remaining headroom: synthesis 107 ms (~14.5 M scatterer-element pairs), build 48 ms. Further
  gains would need fewer pairs (coarser `cell_size`, fewer rays) or GPU synthesis (needs an ADR).
