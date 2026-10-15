---
description: Review current changes against requirements, ADRs, the active plan, and module rules
argument-hint: [path or plan number]
---

Review the current changes (or $ARGUMENTS if given).

Check, in order:

1. **Scope** — anything that belongs downstream (nav error, drift, trajectory, matched filter,
   imaging, waterfall) per ADR 0001? Flag as blocking.
2. **Requirements** — every change maps to a REQ; nothing implemented without one.
3. **Plan** — matches the steps of the active `docs/plans/NNN-*.md`; exit check addressed.
4. **Contract** — any change to `SasPing.msg` without a new ADR is blocking.
5. **Determinism** — stateful RNG in scatterers, thread-order-dependent sums, wall-clock time.
6. **Physics** — units, sign of carrier phase, delay model, amplitude terms vs
   `docs/reference/` Section 2.
7. **Module rules** — `src/*/CLAUDE.md`.
8. **Traceability** — `docs/traceability.md` updated for new code and tests.
9. **Change records** (ADR-0007) — `CHANGELOG.md` line under `[Unreleased]` with refs; new ADR has
   an index row; each bug fix has a `docs/bugs/BUG-NNN` record and a regression test; commit
   message follows Conventional Commits with `Refs:` footer.
10. Correctness bugs and missing tests.

Output one line per finding: `path:line: severity: problem. fix.` Severity is blocking, major, or
minor. No praise.
