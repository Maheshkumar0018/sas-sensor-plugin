# 006 — Platform state, timing, design checks

- Status: Not started
- REQs: REQ-011, REQ-012, REQ-013
- Depends on: 005
- ADRs: 0003, 0005

## Goal

Correct ground-truth state in every ping, deterministic timing, runtime warnings.

## Files

`OnScan()`, `ReadState()`, `DpcaCheck()` in `sas_sonar_plugin.cpp`.

## Steps

1. Array world pose from parent link at transmit time → `pose`.
2. Parent link world linear velocity → `velocity`; feed to the delay solver (plan 005).
3. `header.stamp` = sim time of sensor update; `ping_id` monotonic from 0.
4. Require `gzserver --lockstep`; warn if measured ping interval deviates from `1/PRF`.
5. Warnings only, no behaviour change:
   - at `Load()`: `2·max_range/c > PRI` → range ambiguity
   - each ping, throttled: `|v|·PRI > N·d/2` → DPCA under-sampling
6. No navigation error, drift, or pose corruption anywhere.

## Exit check

Parent link at known constant velocity: `pose` advances by `v·PRI` per ping and `velocity` matches.
DPCA warning appears exactly when speed crosses `N·d/(2·PRI)`.
