# 0003 — `SasPing` carries ground truth only

- Status: Accepted
- Date: 2026-10-15
- REQs: REQ-010, REQ-011

## Context

The reference message had `pose_true` and `pose_nav`. With navigation error out of scope (0001),
`pose_nav` has no producer in this repository.

## Decision

Revise `SasPing`:

- `pose_true` → `pose` (array frame in world, ground truth at transmit time)
- remove `pose_nav` and `range_compressed`
- add `source_level`, `noise_std`, `element_length`, `tx_length` so downstream needs no SDF

After this ADR the message is frozen. Any further change needs a new ADR.

## Consequences

- Downstream applies its own INS error model to `pose`.
- Breaking change versus the reference document; the reference code is not used as-is.
