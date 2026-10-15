# 000 — Roadmap

Ordered implementation plans for the SAS sensor plugin. Each plan is one phase. A phase is done only
when its exit check passes. Do them in order.

| Plan | Title | REQs | Status |
|------|-------|------|--------|
| 001 | Workspace and toolchain | REQ-101 | Done |
| 002 | `SasPing` message | REQ-010, REQ-011 | Done |
| 003 | Plugin skeleton: waveform, array, beams | REQ-001, 002, 003 | Done |
| 004 | Scene reconstruction and scatterer field | REQ-004, 005, 007 | Done |
| 005 | Propagation and echo synthesis | REQ-006, 008, 009, 012 | Done |
| 006 | Platform state, timing, design checks | REQ-011, 012, 013 | Done |
| 007 | Description, test worlds, launch | REQ-014, 102 | Not started |
| 008 | Sensor validation V1–V7 | all functional | Not started |
| 009 | Performance tuning (optional) | REQ-103 | Not started |

```
001 workspace
 └─ 002 SasPing (frozen) ──► downstream package can start
     └─ 003 waveform + array + beams
         └─ 004 scatterer field
             └─ 005 echo synthesis
                 └─ 006 state, timing, checks
                     └─ 007 xacro, worlds, launch
                         └─ 008 validation
                             └─ 009 tuning
```

## Sensor contract (what downstream can rely on)

1. One `SasPing` per transmit at the configured PRF, stamped with transmit sim time.
2. Raw complex baseband echo, element-major, `num_elements × num_samples`.
3. Ground-truth pose and linear velocity of the array frame at transmit time.
4. Waveform and geometry fields sufficient to rebuild the pulse and element positions.
5. Bit-identical output for identical world, seed, and parameters.
6. The sensor does not move itself.

## Plan file template

```
# NNN — Title
- Status: Not started | In progress | Done
- REQs: REQ-xxx
- Depends on: NNN
## Goal
## Files
## Steps
## Exit check
```
