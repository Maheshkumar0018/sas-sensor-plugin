# Bug records

One file per fixed defect: `BUG-NNN-short-title.md`, numbered in order. Policy: ADR-0007.

Write a record when a defect is found in code that already landed (merged or used in a test run).
Each fix must come with a regression test that failed before the fix.

## Template

```
# BUG-NNN — Title
- Status: Open | Fixed | Won't fix
- Found: YYYY-MM-DD, in <commit or plan NNN>
- Fixed: YYYY-MM-DD, in <commit>
- Severity: blocking | major | minor
- REQs: REQ-xxx
## Symptom
What was observed, with the shortest decisive log line or measurement.
## Reproduction
Command, world, parameters.
## Root cause
Why it happened, with file:line.
## Fix
What changed and why it is correct.
## Regression test
Test name and file that now covers it.
```

## Index

| BUG | Title | Severity | Status | REQs | Fixed in |
|-----|-------|----------|--------|------|----------|
| BUG-001 | `gpu_ray` ignores laser_retro (Gazebo 11 `Retro()` stub) | major | Fixed | REQ-007 | plan 007 |
