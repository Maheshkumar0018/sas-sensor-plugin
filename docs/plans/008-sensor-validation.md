# 008 — Sensor validation V1–V7

- Status: Done (2026-10-22)
- REQs: all functional
- Depends on: 007

## Goal

Prove the echo physics. No image formation (out of scope, ADR 0001).

## Files

`tests/validation/`: `record_bags.sh` (21 scenarios, ~15 min, needs `DISPLAY` for `gpu_ray`),
`record_pings.py`, `bag_pose_x.py`, `conftest.py` (bag loading, scene constants), `sas_tools.py`
(matched filter, delay / amplitude / energy models from `SasPing` fields), `test_v1_delay.py` …
`test_v7_noise.py`. Worlds added: `sas_target_only.world`, `sas_empty.world`.

## Checks (as implemented)

| ID | Setup | Pass criterion | REQs |
|----|-------|----------------|------|
| V1 | Point-target world (seabed + facing plate at 15 m), static, `gpu_ray` and `ray` | Per-element matched-filter peak within ±1.5 samples of the model delay; peak/median ≥ 10 | 001, 002, 009, 007 (BUG-001) |
| V2 | Target-only world, 4 × 4 m horizontal plate, carrier at 10/15/20/25 m on one line of sight | Echo energy vs model energy (summed over plate facets), relative to 15 m, within 1 dB | 006, 007 |
| V3 | Same plate, carrier yawed 0 … 0.30 rad | Energy vs model within 1 dB wherever model ≥ −25 dB | 003 |
| V4 | Seabed world, 0.2 m/s (= d/2 per ping) | DPCA pair correlation (ping n+1 el k vs ping n el k+1) ≥ 0.9 and above the same-element control | 004, 005 |
| V5 | Squinted plate; static run spawned at the exact pose of moving ping 20 (1.5 m/s) | Per-element delay change (complex cross-correlation) = model motion term within 0.1 sample | 008 |
| V6 | Seabed world, static, two runs each `gpu_ray` and `ray`, pings matched by `ping_id` | Pose and echo arrays bit-identical | 012 |
| V7 | Empty world | Std per component within 2 % of `noise_std`, mean < 1 % σ, lag-1 correlation < 0.02, new noise each ping | 009 |

## Exit check

`pytest tests/validation` passes all seven. V4/V6 bags handed to the downstream package as
reference input. Update `docs/traceability.md` and REQ statuses to `Verified`.

## Progress log

- 2026-10-15: Recorder, scenario script, models, V1–V7 written. First run: V1 and V4 passed;
  V2, V3, V5, V7 measured seabed clutter. Cause (test setup): the two new worlds had not been
  rebuilt into `install/` (symlink-install links files at build time), and `gzserver` silently
  fell back to its default world with a ground plane. Fix: `sas_sensor.launch.py` now raises
  "world file not found" instead of starting; `record_bags.sh` reports it.
- 2026-10-15: V6 first matched pings by recording window and compared stamps. Stamps legitimately
  differ between runs: the ping time grid starts when `spawn_entity` inserts the carrier, which
  depends on wall-clock start-up. The recorder now selects `--from-id 5`, and V6 compares pings by
  `ping_id` (pose and echo arrays), not stamps.
- 2026-10-15: V2/V3 with the 0.3 m plate failed by up to ±6 dB at random. Cause (physics, not a
  sensor defect): a 56-voxel plate in one range cell is a coherent speckle sum, and when the carrier
  moves or yaws the rays sample its edges differently, so edge voxels drop in or out and the sum
  jumps. Redesign: 4 × 4 m horizontal plate (top at a voxel-centre height) spanning ~100 range
  cells; compare echo energy with the model summed over its facets.
- 2026-10-15/22: V5 redesign for the same reason: static run spawned at exactly the pose of a
  moving ping (`bag_pose_x.py`), so voxels and speckle are identical and only velocity differs.
  Peak picking still flipped between near-equal speckle peaks (3-sample jump on one element), and
  envelope cross-correlation left a correlated +0.10 bias on elements 0–3; the complex
  cross-correlation used now gives a smooth residual. Tolerance unchanged (0.1 sample).
- 2026-10-22: Final run, `pytest tests/validation`: 9 passed.
  - V1 `gpu_ray`: −0.81 … +0.37 samples, peak/median 23–38; `ray`: −0.80 … +1.31, 12–34.
  - V2: measured − model +0.73 (10 m), −0.34 (20 m), −0.17 dB (25 m).
  - V3: measured − model +0.28 … +0.63 dB for yaw 0.05 … 0.30 rad (model 0 … −11.0 dB).
  - V4: DPCA correlation mean 0.9995, min 0.9990 (165 pairs); control 0.268.
  - V5: motion term −0.150 … −0.181 samples; measured − model −0.006 … +0.070.
  - V6: 5 pings each, `gpu_ray` and `ray`, bit-identical.
  - V7: std 1.0028e-7 / 1.0004e-7 (σ = 1e-7), |mean| 2.8e-10, lag-1 0.0199.
- Deviations from the original table: V1 tolerance ±1.5 samples (plate voxels are jittered
  ±0.85 sample; a single scatterer is checked to 0.1 sample in `test_synth`); V2/V3 use energy of
  a distributed target; V3 asserted only where the model is ≥ −25 dB; V6 static carrier only (a
  carrier started by `ros2 topic pub` begins moving at a wall-clock-dependent sim time).
- Bags (50 MB) stay local in `tests/validation/bags/` (git-ignored); regenerate with
  `record_bags.sh`.
