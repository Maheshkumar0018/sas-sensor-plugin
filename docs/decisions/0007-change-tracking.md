# 0007 — Change tracking: changelog, decision log, bug records

- Status: Accepted
- Date: 2026-10-15
- REQs: —
- Supersedes: —

## Context

The project needs a durable history of what changed, why, and which bugs were fixed. Git history
alone is too fine-grained for users and loses the reasoning behind decisions and fixes.

## Decision

Three records, each with one job:

1. **`CHANGELOG.md`** — what changed, for users of the sensor. Keep a Changelog format, SemVer.
   Every user-visible change adds a line under `[Unreleased]` in the same commit. On release, rename
   `[Unreleased]` to `[x.y.z] - YYYY-MM-DD` and tag `vx.y.z`. Per-package `CHANGELOG.rst` files are
   generated with `catkin_generate_changelog` at release time only.
2. **`docs/decisions/`** — why. ADRs plus the index in `docs/decisions/README.md`.
3. **`docs/bugs/`** — bug records `BUG-NNN-title.md` for every defect fixed after its code first
   landed. Each record has symptom, reproduction, root cause, fix, and the regression test. Trivial
   fixes inside an unmerged change need no record.

Commit messages follow Conventional Commits (`feat`, `fix`, `docs`, `refactor`, `test`, `perf`,
`build`, `chore`), with optional scope = package name, and a footer with references:

```
fix(sas_gazebo): use arrival-time receiver position in delay solver

Refs: REQ-008, BUG-003
```

A breaking `SasPing` change uses `feat(sas_msgs)!:` and a `BREAKING CHANGE:` footer, and needs its own ADR.

## Consequences

- One extra file touch per change; `/review` checks it.
- Release notes come straight from `CHANGELOG.md`.
- Bug records give root-cause history and link each fix to a regression test.
