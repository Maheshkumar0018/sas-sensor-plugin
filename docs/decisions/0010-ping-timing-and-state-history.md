# 0010 — Ping timing and transmit-time state are enforced by the plugin

- Status: Accepted
- Date: 2026-10-15
- REQs: REQ-011, REQ-012
- Supersedes: —

## Context

Plan 006 assumed `gzserver --lockstep` gives one ping per PRI and that the link state read in the
sensor callback is the state at transmit time. Measured in the smoke world (plan 006 log):

- CPU `ray` sensor: `--lockstep` does not wait for the callback. Each ping takes ~0.43 s wall
  time, physics runs in real time meanwhile, and pings came every ~0.43 s of sim time instead of
  0.1 s.
- `gpu_ray` sensor: `--lockstep` holds the PRI exactly, but physics keeps stepping while the
  callback runs, so the link pose read in the callback was 1–8 physics steps after the scan.
- Both: a scan rendered at one pose combined with a later pose misplaces the scatterers and the
  reported ground truth.

## Decision

1. **State history.** On every `WorldUpdateEnd` (physics thread) the plugin stores
   (sim time, array pose, array velocity) in a ring buffer of 8192 samples. Each ping looks up the
   sample at the sensor's `LastMeasurementTime()`. That state is used to build scatterers, to
   synthesise the echo, and to fill `SasPing.pose` and `velocity`.
2. **Strict-rate gate for CPU `ray`.** SDF param `strict_rate` (default true for `ray`). After the
   physics step that reaches `last_published_ping + PRI`, the physics thread waits until that ping
   is published, with a 10 s wall-time timeout and a warning. The gate is armed at load, with the
   first ping due at t = 0.
3. **`gpu_ray` relies on `--lockstep`.** The gate deadlocks against Gazebo's rendering lockstep,
   so it is forced off for `gpu_ray` (warning if requested).
4. A throttled warning reports any ping interval that differs from the PRI by more than 1 %.
   Throttled warnings use a steady clock, because the node clock is sim time and stays at 0 when
   no `/clock` is published.

## Consequences

- Stamps are exact multiples of the PRI and pose advance equals v·PRI to ~1e-14 m, for both sensor
  types (plan 006 exit check).
- With CPU `ray`, simulation runs only as fast as the sonar computes (real-time factor < 1).
- `gpu_ray` without `--lockstep` still drops pings; the interval warning shows it.
- The gate must not be armed when a ping starts: Gazebo wakes the sensor thread at world update
  begin, and a busy sensor thread misses that wake-up, which stalls the gate until its timeout.
