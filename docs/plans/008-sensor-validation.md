# 008 — Sensor validation V1–V7

- Status: Not started
- REQs: all functional
- Depends on: 007

## Goal

Prove the echo physics. No image formation (out of scope, ADR 0001).

## Files

`tests/validation/conftest.py` (bag loading, offline matched filter helper),
`tests/validation/test_v1_delay.py` … `test_v7_noise.py`, `tests/validation/README.md` (how to record
the bags).

## Checks

| ID | Setup | Pass criterion | REQs |
|----|-------|----------------|------|
| V1 | Facing plate target (not a sphere, see plan 005 log), static, both `gpu_ray` and `ray` (BUG-001 regression) | Matched-filter peak at `2R/c ± 1 sample` per element | 001, 002, 009 |
| V2 | Point target at ≥3 ranges | Peak amplitude follows `1/R² · 10^{−α·2R/20}` within 1 dB | 006, 007 |
| V3 | Rotate carrier across target | Amplitude vs angle matches `b_tx·b_rx` within 1 dB to first null | 003 |
| V4 | Moving carrier, seabed world | Revisited voxel identical; point-target phase follows `e^{−j2πf_c τ(t)}` from `pose` | 004, 005 |
| V5 | Carrier 1.5 m/s vs static | Per-element delay shift matches moving-receiver model within 0.1 sample | 008 |
| V6 | Two runs, same seed | Echo arrays bit-identical | 012 |
| V7 | Empty world | Sample std ≈ `noise_std` per component | 009 |

## Exit check

`pytest tests/validation` passes all seven. V4/V6 bags handed to the downstream package as
reference input. Update `docs/traceability.md` and REQ statuses to `Verified`.
