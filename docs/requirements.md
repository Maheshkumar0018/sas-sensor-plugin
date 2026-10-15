# Requirements

Source of truth for what the SAS sensor plugin must do. Every plan step, commit, and test refers to
these IDs. Old feature numbers (F#) from `reference/SAS_Gazebo_ROS2_Implementation.md` are listed for
cross-reference.

Status values: `Draft` → `Approved` → `Implemented` → `Verified`.

## Functional

| ID | Requirement | Old F# | Verified by | Status |
|----|-------------|--------|-------------|--------|
| REQ-001 | Transmit an LFM chirp with configurable centre frequency, bandwidth, and pulse length. Simulate in complex baseband. | F1 | V1 | Implemented |
| REQ-002 | Model a transmitter and an N-element along-track receiver array with configurable element count, spacing, and element length. | F2 | V1 | Implemented |
| REQ-003 | Apply transmit and receive beam patterns `b(θ) = sinc(D sinθ / λ)` at the centre frequency. | F3 | V3 | Draft |
| REQ-004 | Derive scene geometry from Gazebo ray casting (`gpu_ray` or `ray`). Reconstruct surfaces and produce acoustic shadows by rejecting grazing and occluded faces. | F4 | V4 | Draft |
| REQ-005 | Represent the scene as world-fixed voxel scatterers. Position jitter and complex speckle derive from a hash of the voxel index, so a voxel is identical in every ping. | F5 | V4 | Draft |
| REQ-006 | Model two-way spherical spreading, Thorp absorption (or SDF override), and Lambertian backscatter. | F6 | V2 | Draft |
| REQ-007 | Take per-surface backscatter strength from SDF `<laser_retro>`, with a configurable default. | F7 | V2 | Draft |
| REQ-008 | Account for receiver motion during propagation (no stop-and-hop) when computing each element's delay. | F8 | V5 | Draft |
| REQ-009 | Output the raw received baseband echo per element with additive complex Gaussian receiver noise. No matched filtering in the sensor. | F9 | V1, V7 | Draft |
| REQ-010 | Publish one `sas_msgs/SasPing` per transmit on a configurable topic. The message carries all waveform and geometry parameters needed to interpret the echo without the SDF. | F12 | plan 002 exit check | Implemented |
| REQ-011 | Report the ground-truth world pose and linear velocity of the array frame at transmit time. Never add navigation error. | F10 (part) | plan 006 exit check | Draft |
| REQ-012 | Produce bit-identical output for the same world, seed, and parameters, under lockstep simulation. | F15 | V6 | Draft |
| REQ-013 | Warn (without changing behaviour) on range ambiguity (`2·max_range/c > PRI`) and DPCA under-sampling (`|v|·PRI > N·d/2`). | F15 | plan 006 exit check | Draft |
| REQ-014 | Provide a reusable `sas_array.xacro` macro so any vehicle model can mount the sensor by including it with a parent link. | — | plan 007 exit check | Draft |

## Non-functional

| ID | Requirement | Verified by | Status |
|----|-------------|-------------|--------|
| REQ-101 | Ubuntu 22.04, ROS 2 Humble, Gazebo Classic 11, C++17. | Build | Draft |
| REQ-102 | Run on CPU (`ray`) when no OpenGL GPU is available. | plan 007 run with `sensor_type:=ray` | Draft |
| REQ-103 | Performance target: **TBD** (real-time factor and max ping latency at default parameters). | plan 009 | Draft — needs value |

## Out of scope (downstream package)

Navigation / INS drift models, vehicle trajectory and sway / heave / yaw disturbances, matched
filtering, micronavigation, motion compensation, image formation, waterfall display, PSF focus tests.
See `decisions/0001-sensor-only-scope.md`.
