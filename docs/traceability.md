# Traceability

Links each requirement to the decision that shaped it, the plan that builds it, the code that
implements it, and the test that verifies it. Update this file in the same change that adds code or
tests for a REQ. Code and test columns stay `—` until the work exists.

| REQ | ADR | Plan | Code | Test | Status |
|-----|-----|------|------|------|--------|
| REQ-001 | — | 003 | `dsp.hpp` `ChirpSamples`; `sas_sonar_plugin.cpp` `SetupDsp` | `test_dsp` Chirp.UnitModulusAndSweep; V1 | Implemented |
| REQ-002 | — | 003 | `dsp.hpp` `ReceiverPositionsY`; `sas_sonar_plugin.cpp` `Load` | `test_dsp` Array.ReceiverPositionsCentred; V1 | Implemented |
| REQ-003 | — | 003, 005 | `dsp.hpp` `BeamPattern`; `synth.cpp` per-scatterer beams | `test_dsp` Beam.SincAndFirstNull; `test_synth` BeamNullSuppressesTarget; V3 | Implemented |
| REQ-004 | — | 004 | `scatter.cpp` `BuildScatterers`; plugin `ReadScan` | `test_scatter` FlatGroundLiesOnPlane, DepthDiscontinuityLeavesShadow, RangeGateApplied; V4 | Implemented |
| REQ-005 | 0004 | 004 | `scatter.hpp` `CellKey`, `VoxelIndex`, `VoxelScatterer` | `test_scatter` Hash.*, BoundaryAlignedSurfaceIsOneLayer, RevisitedVoxelsAreIdentical; V4 | Implemented |
| REQ-006 | — | 005 | `synth.hpp` `ThorpDbPerMeter`; `synth.cpp` spreading, Lambert | `test_synth` ThorpAt100kHz, SpreadingAndAbsorption; V2 | Implemented |
| REQ-007 | — | 004 | `scatter.cpp` retro → μ; plugin `default_backscatter_db` | `test_scatter` RetroSetsMuOtherwiseDefault; V2 | Implemented |
| REQ-008 | 0005 | 005, 006 | `synth.hpp` `SolveDelayFromTx` (velocity fed in plan 006) | `test_synth` DelaySolverStaticExactMovingSubMicron, ReceiverMotionShiftsDelayAsModelled; V5 | Implemented |
| REQ-009 | 0002 | 005 | `synth.cpp` `EchoSynthesizer::Synthesize` | `test_synth` PointTargetDelayOnEveryElement, NoiseStatisticsAndReproducibility; `tests/validation/check_delay.py`; V1, V7 | Implemented |
| REQ-010 | 0002, 0003 | 002 | `src/sas_msgs/msg/SasPing.msg` | `src/sas_msgs/test/test_sasping_contract.py`; plan 002 exit check | Implemented |
| REQ-011 | 0003, 0005 | 002 (field), 006 (fill) | — | plan 006 exit check | Draft |
| REQ-012 | 0004 | 005, 006 | `FFTW_ESTIMATE`; per-element threads; seeded noise | `test_synth` IndependentOfThreadCount; `test_scatter` SameInputSameOutputOrder; V6 | Draft |
| REQ-013 | — | 006 | — | plan 006 exit check | Draft |
| REQ-014 | 0005 | 007 | — | plan 007 exit check | Draft |
| REQ-101 | 0008, 0009 | 001 | `src/*/package.xml`, `src/*/CMakeLists.txt` | build | Draft |
| REQ-102 | — | 007 | — | CPU run | Draft |
| REQ-103 | — | 009 | — | perf run | Draft |
