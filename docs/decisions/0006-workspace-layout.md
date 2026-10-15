# 0006 — Repository is the colcon workspace; tests split by level

- Status: Accepted
- Date: 2026-10-15
- REQs: —

## Context

The project follows a docs-first layout (`docs/`, `src/`, `tests/`). ROS 2 expects packages under
`src/` and unit tests inside each package.

## Decision

- Repository root is the colcon workspace root; `src/` holds the ROS 2 packages.
- Unit tests: `src/<pkg>/test/`, run by `colcon test` (gtest / pytest).
- System validation that spans packages (V1–V7): `tests/validation/`, run by `pytest` against
  recorded bags.
- `build/`, `install/`, `log/` are build output and are not committed.

## Consequences

- Standard ROS tooling works unchanged.
- Two test entry points: `colcon test` and `pytest tests/validation`.
