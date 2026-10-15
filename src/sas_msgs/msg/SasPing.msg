# One SAS ping: raw complex baseband echoes for every receiver element,
# plus the ground-truth platform state at transmit time.
# Frozen contract with the downstream processing package (ADR-0003).
# Any field change needs a new ADR.

std_msgs/Header header             # stamp = transmit sim time, frame_id = array frame
uint64 ping_id                     # monotonic, starts at 0

# Waveform and sampling
float64 sound_speed                # m/s
float64 center_frequency           # Hz
float64 bandwidth                  # Hz
float64 pulse_length               # s
float64 sample_rate                # Hz, complex baseband
float64 t0                         # s, time of sample 0 relative to transmit
float64 source_level               # dB re 1 uPa @ 1 m
float64 noise_std                  # per-component std of added receiver noise, same units as data

# Array geometry in array frame (x = boresight, y = along-track, z = up)
geometry_msgs/Point tx_position    # m
geometry_msgs/Point[] rx_positions # m, one per element
float64 element_length             # m, along-track receiver aperture
float64 tx_length                  # m, along-track transmitter aperture

# Platform state at transmit time, ground truth only (no navigation error)
geometry_msgs/Pose pose            # array frame in world
geometry_msgs/Vector3 velocity     # m/s, world-frame linear velocity of array origin

# Raw echo, element-major: index = k * num_samples + n
uint32 num_elements
uint32 num_samples
float32[] data_real
float32[] data_imag
