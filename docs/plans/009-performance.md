# 009 — Performance tuning (optional)

- Status: Not started
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
