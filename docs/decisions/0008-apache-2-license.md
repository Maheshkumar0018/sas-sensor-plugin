# 0008 — License: Apache-2.0

- Status: Accepted
- Date: 2026-10-15
- REQs: —
- Supersedes: —

## Context

Every `package.xml` needs a license, and the repository needs a `LICENSE` file before it is shared.

## Decision

The project uses the Apache License, Version 2.0. Full text is in `LICENSE` at the repository root.
Every package declares `<license>Apache-2.0</license>`.

## Consequences

- Matches the license used by most ROS 2 core packages.
- Includes an explicit patent grant.
- New source files may carry the short Apache-2.0 header; not required by this ADR.
