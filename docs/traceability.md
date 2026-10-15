# Traceability

Links each requirement to the decision that shaped it, the plan that builds it, the code that
implements it, and the test that verifies it. Update this file in the same change that adds code or
tests for a REQ. Code and test columns stay `—` until the work exists.

| REQ | ADR | Plan | Code | Test | Status |
|-----|-----|------|------|------|--------|
| REQ-001 | — | 003 | — | unit: chirp; V1 | Draft |
| REQ-002 | — | 003 | — | V1 | Draft |
| REQ-003 | — | 003 | — | unit: Sinc; V3 | Draft |
| REQ-004 | — | 004 | — | V4 | Draft |
| REQ-005 | 0004 | 004 | — | unit: CellKey/SplitMix64; V4 | Draft |
| REQ-006 | — | 005 | — | unit: Thorp; V2 | Draft |
| REQ-007 | — | 004 | — | V2 | Draft |
| REQ-008 | 0005 | 005 | — | unit: delay solver; V5 | Draft |
| REQ-009 | 0002 | 005 | — | V1, V7 | Draft |
| REQ-010 | 0002, 0003 | 002 | `src/sas_msgs/msg/SasPing.msg` | `src/sas_msgs/test/test_sasping_contract.py`; plan 002 exit check | Implemented |
| REQ-011 | 0003, 0005 | 002 (field), 006 (fill) | — | plan 006 exit check | Draft |
| REQ-012 | 0004 | 005, 006 | — | V6 | Draft |
| REQ-013 | — | 006 | — | plan 006 exit check | Draft |
| REQ-014 | 0005 | 007 | — | plan 007 exit check | Draft |
| REQ-101 | — | 001 | `src/*/package.xml`, `src/*/CMakeLists.txt` | build | Draft |
| REQ-102 | — | 007 | — | CPU run | Draft |
| REQ-103 | — | 009 | — | perf run | Draft |
