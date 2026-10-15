---
description: Draft or update an implementation plan in docs/plans for given REQ IDs or topic
argument-hint: <REQ-xxx ... | topic>
---

Create or update an implementation plan for: $ARGUMENTS

1. Read `CLAUDE.md`, `docs/requirements.md`, `docs/architecture.md`, `docs/plans/000-roadmap.md`, and
   every ADR in `docs/decisions/`.
2. Confirm each target REQ exists and is in scope. If a needed requirement is missing, stop and
   propose the REQ text instead of planning around it.
3. If an existing plan covers these REQs, update it. Otherwise create `docs/plans/NNN-title.md` with
   the next free number, using the template in `000-roadmap.md` (Goal, Files, Steps, Exit check).
4. Every step names the file and function it touches. The exit check must be observable
   (a command and its expected output, or a test that passes).
5. Note any decision that needs a new ADR; do not write the ADR unless asked.
6. Update the table in `000-roadmap.md` and the Plan column in `docs/traceability.md`.
7. Do not write code.
