---
description: Audit docs/traceability.md against requirements, plans, code, and tests
---

1. Read `docs/requirements.md` and `docs/traceability.md`.
2. For each REQ, verify the listed ADR, plan, code location, and test actually exist. Search `src/`
   and `tests/` for the REQ ID and for the functions named in the plan.
3. Report: REQs with no plan, REQs with code but no test, rows pointing at missing files, code or
   tests that reference no REQ.
4. Propose the corrected table. Apply it only if the user agrees.
