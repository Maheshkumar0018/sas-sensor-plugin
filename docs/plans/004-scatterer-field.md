# 004 — Scene reconstruction and scatterer field

- Status: Not started
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
