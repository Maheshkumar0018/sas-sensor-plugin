# Traceability

Links each requirement to the decision that shaped it, the plan that builds it, the code that
implements it, and the test that verifies it. Update this file in the same change that adds code or
tests for a REQ. Code and test columns stay `—` until the work exists.

| REQ | ADR | Plan | Code | Test | Status |
|-----|-----|------|------|------|--------|
| REQ-001 | — | 003 | `dsp.hpp` `ChirpSamples`; `sas_sonar_plugin.cpp` `SetupDsp` | `test_dsp` Chirp.UnitModulusAndSweep; `tests/validation/test_v1_delay.py` | Verified |
| REQ-002 | — | 003 | `dsp.hpp` `ReceiverPositionsY`; `sas_sonar_plugin.cpp` `Load` | `test_dsp` Array.ReceiverPositionsCentred; `tests/validation/test_v1_delay.py` | Verified |
| REQ-003 | — | 003, 005 | `dsp.hpp` `BeamPattern`; `synth.cpp` per-scatterer beams | `test_dsp` Beam.SincAndFirstNull; `test_synth` BeamNullSuppressesTarget; `tests/validation/test_v3_beam.py` | Verified |
| REQ-004 | — | 004 | `scatter.cpp` `BuildScatterers`; plugin `ReadScan` | `test_scatter` FlatGroundLiesOnPlane, DepthDiscontinuityLeavesShadow, RangeGateApplied; `tests/validation/test_v4_coherence.py` | Verified |
| REQ-005 | 0004 | 004 | `scatter.hpp` `CellKey`, `VoxelIndex`, `VoxelScatterer` | `test_scatter` Hash.*, BoundaryAlignedSurfaceIsOneLayer, RevisitedVoxelsAreIdentical; `tests/validation/test_v4_coherence.py` | Verified |
| REQ-006 | — | 005 | `synth.hpp` `ThorpDbPerMeter`; `synth.cpp` spreading, Lambert | `test_synth` ThorpAt100kHz, SpreadingAndAbsorption; `tests/validation/test_v2_spreading.py` | Verified |
| REQ-007 | 0011 | 004, 007 | `scatter.cpp` retro → μ; plugin `default_backscatter_db`, `OnLaserFrame`/`ApplyGpuRetro` (BUG-001) | `test_scatter` RetroSetsMuOtherwiseDefault; `tests/validation/test_v2_spreading, test_v1_delay (gpu_ray).py` | Verified |
| REQ-008 | 0005 | 005, 006 | `synth.hpp` `SolveDelayFromTx` (velocity fed in plan 006) | `test_synth` DelaySolverStaticExactMovingSubMicron, ReceiverMotionShiftsDelayAsModelled; `tests/validation/test_v5_moving_receiver.py` | Verified |
| REQ-009 | 0002 | 005 | `synth.cpp` `EchoSynthesizer::Synthesize` | `test_synth` PointTargetDelayOnEveryElement, NoiseStatisticsAndReproducibility; `tests/validation/check_delay.py`; `tests/validation/test_v1_delay, test_v7_noise.py` | Verified |
| REQ-010 | 0002, 0003 | 002 | `src/sas_msgs/msg/SasPing.msg` | `src/sas_msgs/test/test_sasping_contract.py`; plan 002 exit check | Verified |
| REQ-011 | 0003, 0005, 0010 | 002 (field), 006 (fill) | plugin `OnWorldUpdateEnd`, `LookupState`, `OnScan` | `tests/validation/check_state.py`; plan 006 exit check | Verified |
| REQ-012 | 0004, 0010 | 005, 006 | `FFTW_ESTIMATE`; per-element threads; seeded noise; strict-rate gate / `--lockstep`; state history | `test_synth` IndependentOfThreadCount; `test_scatter` SameInputSameOutputOrder; `tests/validation/test_v6_determinism.py` | Verified |
| REQ-013 | — | 006 | `dsp.hpp` `RangeAmbiguous`, `DpcaViolated`; plugin `CheckDpca`, `CheckTiming` | `test_dsp` DesignChecks.*; plan 006 exit check | Verified |
| REQ-014 | 0005 | 007 | `sas_description/urdf/sas_array.xacro`, `sas_test_carrier.urdf.xacro` | `check_urdf`; plan 007 exit check | Verified |
| REQ-101 | 0008, 0009 | 001 | `src/*/package.xml`, `src/*/CMakeLists.txt` | build | Verified |
| REQ-102 | 0010 | 007 | `sensor_type:=ray`; strict-rate gate | plan 007 exit check (CPU `ray` runs) | Verified |
| REQ-103 | — | 009 | `synth.hpp` `FractionalDelayTaps`, closed-form `SolveDelayFromTx`; `synth.cpp` chunked tasks; `scatter.hpp` `KeySet`; `scatter.cpp` parallel build | `tests/validation/measure_rtf.sh`; `test_synth` Kernel.*, `test_scatter` KeySet.*, Build.IndependentOfThreadCount | Verified |
