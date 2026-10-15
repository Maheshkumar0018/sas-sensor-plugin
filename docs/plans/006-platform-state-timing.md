# 006 — Platform state, timing, design checks

- Status: Done (2026-10-15)
- REQs: REQ-011, REQ-012, REQ-013
- Depends on: 005
- ADRs: 0003, 0005, 0010

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

## Progress log

- 2026-10-15: `dsp.hpp`: `RangeAmbiguous`, `DpcaViolated`, `PingIntervalOff` with 3 gtests
  (32 unit tests in the workspace, 0 failures). Plugin: range-ambiguity warning at load; DPCA
  warning per ping (throttled) using the along-track velocity component; ping-interval warning;
  `SasPing.pose`, `velocity`, and stamp filled from the measurement time.
- 2026-10-15: Step 4 did not hold as planned. Findings (`tests/validation/check_state.py`):
  CPU `ray` + `--lockstep` gave ping intervals of ~0.43 s sim time (callback time), because Gazebo
  does not wait for non-rendering sensor callbacks. `gpu_ray` + `--lockstep` held 0.100 s, but the
  link pose read in the callback was 1–8 physics steps late. Earlier smoke runs (plans 004–005) had
  the same small pose lag; their exit checks were static or voxel-identity checks and are not
  affected.
- 2026-10-15: Fix, ADR-0010: per-step state history looked up at `LastMeasurementTime()`;
  strict-rate gate for CPU `ray` (`strict_rate`, default on for `ray`, forced off for `gpu_ray`);
  throttled warnings on a steady clock (the sim-time node clock stays 0 without `/clock`, so
  throttled warnings never printed).
- 2026-10-15: Two dead ends recorded in ADR-0010: arming the gate at ping start stalls against
  Gazebo's sensor wake-up; the gate with `gpu_ray` deadlocks against rendering lockstep.
- 2026-10-15: Exit check, smoke world moving along-track (`/sas/cmd_vel`), 30 pings at 1.5 m/s,
  both sensor types: stamp interval − PRI ≤ 5e-16 s, |Δpose − v·Δt| ≤ 6e-15 m, reported velocity =
  commanded, orientation constant, `ping_id` consecutive; no interval warnings, no gate timeouts,
  no missing state samples. DPCA: no warning at 3.1 m/s, warning at 3.3 m/s
  ("v_along*PRI = 0.330 m > N*d/2 = 0.320 m"; limit 3.20 m/s). Passed.
- Regression: `check_delay.py` (plan 005) still passes.
- REQ-013 wording changed to the along-track speed: cross-track motion does not alias the
  synthetic aperture.
