# 0004 — Hash-based world-fixed scatterers and deterministic noise

- Status: Accepted
- Date: 2026-10-15
- REQs: REQ-005, REQ-012

## Context

SAS focusing needs each scatterer to keep the same position and complex reflectivity across all
pings. Ray hits move with the vehicle, and a stateful RNG gives different values each time a voxel
is visited. Multi-threaded synthesis must not make output depend on thread order.

## Decision

- Quantise reconstructed surfaces into world voxels of size `cell_size`.
- Derive each voxel's jitter and speckle from `SplitMix64(CellKey ^ seed)`; no stateful RNG.
- Receiver noise uses an independent generator seeded from `(seed, ping_id, element)`.
- Simulation runs with `gzserver --lockstep`.

## Consequences

- Bit-identical reruns (V6) and coherent revisits (V4).
- Surface is quantised to `cell_size`, which adds small geometric roughness.
