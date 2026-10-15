# 002 — `SasPing` message

- Status: Done (2026-10-15)
- REQs: REQ-010, REQ-011
- Depends on: 001
- ADRs: 0002, 0003

## Goal

Stable sensor-output contract. Frozen after this plan.

## Files

`src/sas_msgs/msg/SasPing.msg`, `CMakeLists.txt`, `package.xml`.

## Steps

1. Define the message:
   ```
   std_msgs/Header header             # stamp = transmit sim time, frame_id = array frame
   uint64 ping_id

   # Waveform and sampling
   float64 sound_speed                # m/s
   float64 center_frequency           # Hz
   float64 bandwidth                  # Hz
   float64 pulse_length               # s
   float64 sample_rate                # Hz, complex baseband
   float64 t0                         # s, time of sample 0 relative to transmit
   float64 source_level               # dB re 1 uPa @ 1 m
   float64 noise_std                  # per-component std of added receiver noise

   # Array geometry in array frame (x = boresight, y = along-track, z = up)
   geometry_msgs/Point tx_position
   geometry_msgs/Point[] rx_positions
   float64 element_length             # m, along-track receiver aperture
   float64 tx_length                  # m, along-track transmitter aperture

   # Platform state at transmit time — ground truth only
   geometry_msgs/Pose pose            # array frame in world
   geometry_msgs/Vector3 velocity     # world-frame linear velocity of array origin

   # Raw echo, element-major: index = k * num_samples + n
   uint32 num_elements
   uint32 num_samples
   float32[] data_real
   float32[] data_imag
   ```
2. Build with `rosidl_generate_interfaces`; depend on `std_msgs`, `geometry_msgs`.

## Exit check

`ros2 interface show sas_msgs/msg/SasPing` prints the definition above.

## Progress log

- 2026-10-15: `msg/SasPing.msg` written as defined above, with units in comments.
  `rosidl_generate_interfaces` with `std_msgs`, `geometry_msgs`. `ros2 interface show
  sas_msgs/msg/SasPing` prints the definition. Exit check passed.
- 2026-10-15: Added `test/test_sasping_contract.py` (ament_cmake_pytest). It fails if any field is
  added, removed, renamed, retyped, or reordered. `colcon test --packages-select sas_msgs`: 0 failures.
- 2026-10-15: Environment issue: user-site `langsmith` pytest plugin breaks `python3 -m pytest`, so
  ament silently skipped the test. Workaround `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`, recorded in
  `CLAUDE.md` "Environment notes". Not a project defect, so no BUG record.
- Contract is now frozen (ADR-0003).
