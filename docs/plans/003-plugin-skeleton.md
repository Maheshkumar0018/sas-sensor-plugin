# 003 — Plugin skeleton: waveform, array, beams

- Status: Not started
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
