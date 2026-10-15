# 003 — Plugin skeleton: waveform, array, beams

- Status: Done (2026-10-15)
- REQs: REQ-001, REQ-002, REQ-003
- Depends on: 002

## Goal

Plugin loads, parses parameters, and publishes correctly shaped `SasPing` messages with zero echo.

## Files

`src/sas_gazebo/src/sas_sonar_plugin.cpp`, `src/sas_gazebo/CMakeLists.txt`,
`src/sas_gazebo/test/test_dsp.cpp`.

## Steps

1. `Load()`: attach to `gpu_ray` or `ray` sensor; parse `sound_speed`, `center_frequency`,
   `bandwidth`, `pulse_length`, `sample_rate`, `num_elements`, `element_spacing`, `element_length`,
   `tx_length`, `source_level`, `noise_std`, `max_range`, `seed`, `topic`. Reject invalid values
   (e.g. `sample_rate < bandwidth`) with a clear error.
2. Receiver positions `rx_pos_[k] = (k − (N−1)/2) · d` on array y-axis; transmitter at origin.
3. `SetupDsp()`: size `nfft_`; build chirp `p(t) = exp(jπK(t − Tp/2)²)` spectrum once as
   `chirp_spec_`; create FFTW plans once.
4. `Sinc()`, `BeamTx(θ)`, `BeamRx(θ)` = `sinc(D sinθ / λ)`.
5. `ConnectUpdated()` → stub `OnScan()` that publishes header, waveform, geometry, zero echo.
6. Unit tests: chirp instantaneous frequency sweeps `−B/2…+B/2`; `Sinc(0)=1`, first null at 1.

## Exit check

`ros2 topic echo /sas/ping --once` shows correct fields; `data_real.size() == num_elements ·
num_samples`, all zero. `colcon test --packages-select sas_gazebo` passes.

## Progress log

- 2026-10-15: `include/sas_gazebo/dsp.hpp`: `Sinc`, `BeamPattern`, `CeilSamples`, `NextPow2`,
  `ChirpSamples`, `ReceiverPositionsY`, `SonarParams` (defaults from reference Section 2.3:
  fc 100 kHz, B 20 kHz, Tp 5 ms, fs 40 kHz, N 16, d 0.04 m, max_range 60 m), `Validate`.
- 2026-10-15: `test/test_dsp.cpp`: 5 tests (chirp modulus and -B/2..+B/2 sweep, sinc and first
  beam null at sin θ = λ/D, centred receiver positions, parameter validation, NextPow2). Pass when
  compiled directly with g++; `colcon test` pending the FFTW install.
- 2026-10-15: `src/sas_sonar_plugin.cpp`: `Load()` (gpu_ray/ray check, SDF params, validation,
  publisher on `topic`), `SetupDsp()` (single-precision FFTW, chirp spectrum kept resident),
  stub `OnScan()` (header with sim-time stamp, waveform, geometry, zero echo). `pose`/`velocity`
  left identity/zero for plan 006.
- 2026-10-15: Decision inside plan scope: FFTW plans use `FFTW_ESTIMATE`, not `FFTW_MEASURE`
  (reference used MEASURE). MEASURE picks algorithms from timing, so rounding could differ between
  runs and break REQ-012.
- 2026-10-15: `package.xml` exports `gazebo_ros plugin_path`; `test/worlds/sas_smoke.world` (CPU
  `ray`, 64×64 rays, static mount) for the exit check.
- 2026-10-15: Built with workspace-local FFTW (ADR-0009); no compiler warnings. Plugin links system
  `libfftw3f.so.3`. `colcon test`: 9 tests, 0 failures (5 in `test_dsp`).
- 2026-10-15: Exit check with `sas_smoke.world` under `gzserver --lockstep`: 5 consecutive pings,
  `ping_id` 0–4, stamps 0.1 s apart (PRF 10 Hz), `frame_id` `sas_array_link`, N = 16, Ns = 3200,
  `data_real`/`data_imag` length 51200 and all zero, waveform fields match defaults,
  `rx_positions.y` from −0.30 to +0.30 m in 0.04 m steps. Passed.
- Note: `ros2 topic echo` started together with `gzserver` reports "does not appear to be published
  yet" during plugin start-up; start the echo after the "SAS sonar ready" log line.
