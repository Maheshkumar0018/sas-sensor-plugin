# Decision log

Architecture Decision Records (ADRs), format after Michael Nygard. One file per decision:
`NNNN-short-title.md`, numbered in order, never renumbered.

## Rules

- Write an ADR for any change to scope, the `SasPing` interface, a physical model, a dependency, or
  project process.
- Status values: `Proposed` → `Accepted` → (`Deprecated` | `Superseded by NNNN`).
- An accepted ADR's Context and Decision are never edited. To change a decision, write a new ADR
  with `Supersedes: NNNN`, then change only the old ADR's Status line.
- Every new or superseded ADR gets a line in `CHANGELOG.md` and a row below in the same commit.

## Template

```
# NNNN — Title
- Status: Proposed
- Date: YYYY-MM-DD
- REQs: REQ-xxx
- Supersedes: — 
## Context
## Decision
## Consequences
```

## Index

| ADR | Title | Status | Date | Supersedes |
|-----|-------|--------|------|------------|
| 0001 | Sensor-only scope | Accepted | 2026-10-15 | — |
| 0002 | Raw echo output, no matched filter | Accepted | 2026-10-15 | — |
| 0003 | `SasPing` carries ground truth only | Accepted | 2026-10-15 | — |
| 0004 | Hash-based scatterers and deterministic noise | Accepted | 2026-10-15 | — |
| 0005 | No custom trajectory plugin | Accepted | 2026-10-15 | — |
| 0006 | Repository is the colcon workspace | Accepted | 2026-10-15 | — |
| 0007 | Change tracking: changelog, decision log, bug records | Accepted | 2026-10-15 | — |
| 0008 | License: Apache-2.0 | Accepted | 2026-10-15 | — |
| 0009 | Workspace-local FFTW dev files without root | Accepted | 2026-10-15 | — |
| 0010 | Ping timing and transmit-time state enforced by the plugin | Accepted | 2026-10-15 | — |
| 0011 | `gpu_ray` laser_retro read from the raw laser frame | Accepted | 2026-10-15 | — |
