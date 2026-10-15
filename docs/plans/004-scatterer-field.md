# 004 — Scene reconstruction and scatterer field

- Status: Done (2026-10-15)
- REQs: REQ-004, REQ-005, REQ-007
- Depends on: 003
- ADRs: 0004

## Goal

World-fixed coherent scatterers from the ray image, with shadows and per-material strength.

## Files

`ReadScan()`, `BuildScatterers()`, `CellKey()`, `SplitMix64()` in `sas_sonar_plugin.cpp`;
`src/sas_gazebo/test/test_scatterers.cpp`.

## Steps

1. `ReadScan()`: range image + per-ray `Retro()` every update.
2. Project valid ranges to world points; form quads on the range-image grid.
3. Reject quads with `incidence > max_incidence` or depth discontinuity → acoustic shadows.
4. Subdivide accepted quads at `cell_size/2`; snap to world voxel via `CellKey()`; dedupe per ping.
5. Per voxel: jitter and complex speckle from `SplitMix64(CellKey ^ seed)`. No stateful RNG.
6. Lambert `μ` from mean quad `<laser_retro>`; fallback `default_backscatter_db` when zero.
7. Unit tests: `CellKey` stable for points inside one voxel; same key + seed → same speckle.

## Exit check

Scatterer count per ping stable over a uniform seabed. Debug dump keyed by `CellKey` shows identical
`pos` and `speckle` for a voxel seen in two pings.

## Progress log

- 2026-10-15: `include/sas_gazebo/scatter.hpp` + `src/scatter.cpp` (static lib `sas_scatter`, only
  ignition-math): `SplitMix64`, `U01`, `CellKey`, `VoxelIndex`, `VoxelScatterer`, `Scan`,
  `Scatterer` (now carries its `key`), `Params`, `Workspace`, `BuildScatterers`. Plugin reads the
  scan (`ReadScan` for `gpu_ray`/`ray`), composes the array pose from parent link + sensor pose, and
  builds scatterers every ping. New SDF params: `cell_size` (0.04 m), `max_incidence_deg` (88),
  `default_backscatter_db` (−27), `debug_scatterers` (false).
- 2026-10-15: `test/test_scatter.cpp`, 10 tests: key uniqueness, deterministic and bounded voxel
  jitter, speckle zero mean / unit variance (200k voxels), flat ground on plane, single voxel layer
  for a surface on a round height, revisited voxels identical after a 0.15 m move, shadow behind a
  solid box, retro → μ with default, range gate, identical output order for identical input.
- 2026-10-15: Shadow limit found while testing: the quad bridging a depth jump ΔR between rays
  Δθ apart at range R is rejected only when R·Δθ/ΔR < cos(max_incidence). At 64 rays over the fan
  (Δθ ≈ 0.016 rad) the bridge is ≈ 87.7° and is kept; at 256–512 rays it is ≈ 89.5–89.7° and
  rejected. Shadows therefore need the design ray resolution (512 rays). Documented on
  `Params::max_incidence`; the shadow test uses 256 rays.
- 2026-10-15: Defect found in the smoke run (not yet committed, so no BUG record): the reference
  `floor(x / cell)` voxel grid puts surfaces at round heights (ground z = 0) on a voxel boundary.
  Float noise split the ground across two z layers (z range ±0.036 m) and voxels flickered between
  pings. Fix: voxels centred on multiples of `cell` (`VoxelIndex = floor(x / cell + 0.5)`).
  Regression test `Build.BoundaryAlignedSurfaceIsOneLayer` fails with the old grid, passes with the
  new one.
- 2026-10-15: Exit check, `sas_smoke.world` (sonar 10 m up, CPU ray 64×64, cell 0.04 m), static for
  ~50 pings then 1.5 m/s along-track via `/sas/cmd_vel` (stock `libgazebo_ros_planar_move`):
  180 pings, scatterer count 821,015–821,046 (0.004 % spread), z within ±0.016 m (jitter only),
  tracked voxel `400e7e002fb00000` identical `pos` and `speckle` in pings 0–164 (bit-identical
  printout, 9 decimals), then out of the footprint. Passed.
- Performance note for plan 009: `BuildScatterers` takes 140–200 ms per ping for ~0.82 M voxels.
