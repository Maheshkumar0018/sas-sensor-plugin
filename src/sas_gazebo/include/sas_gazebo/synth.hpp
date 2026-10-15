// Copyright 2026 The SAS Sensor Plugin Authors
// SPDX-License-Identifier: Apache-2.0
//
// Propagation and raw echo synthesis (REQ-003, REQ-006, REQ-008, REQ-009, REQ-012).
// Depends on ignition::math and FFTW only, so gtest can exercise it without Gazebo.

#ifndef SAS_GAZEBO__SYNTH_HPP_
#define SAS_GAZEBO__SYNTH_HPP_

#include <fftw3.h>

#include <complex>
#include <cstdint>
#include <vector>

#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

#include "sas_gazebo/scatter.hpp"

namespace sas_gazebo
{
namespace synth
{

using ignition::math::Pose3d;
using ignition::math::Vector3d;

// Thorp absorption. Input in Hz, output in dB/m.  REQ-006
inline double ThorpDbPerMeter(double f_hz)
{
  const double f = f_hz / 1000.0;
  const double f2 = f * f;
  const double db_per_km =
    0.11 * f2 / (1.0 + f2) + 44.0 * f2 / (4100.0 + f2) + 2.75e-4 * f2 + 0.003;
  return db_per_km / 1000.0;
}

// Two-way delay with the receiver moving at `vel` while the echo travels (no stop-and-hop).
// Solves tau = (r_tx + |P - (R + v tau)|) / c by fixed-point iteration from the static delay.
// Each iteration shrinks the error by about |v|/c. One iteration leaves ~40 um of path error at
// 3 m/s and 30 m; two leave < 1e-6 m.  REQ-008
// Returns tau; `r_rx` receives the receive path length at arrival.
inline double SolveDelayFromTx(
  const Vector3d & p, double r_tx, const Vector3d & rx, const Vector3d & vel, double c,
  double & r_rx)
{
  double tau = (r_tx + p.Distance(rx)) / c;
  for (int it = 0; it < 2; ++it) {
    r_rx = p.Distance(rx + vel * tau);
    tau = (r_tx + r_rx) / c;
  }
  return tau;
}

inline double SolveDelay(
  const Vector3d & p, const Vector3d & tx, const Vector3d & rx, const Vector3d & vel, double c,
  double & r_rx)
{
  return SolveDelayFromTx(p, p.Distance(tx), rx, vel, c, r_rx);
}

struct Params
{
  double sound_speed = 1500.0;
  double center_frequency = 100e3;
  double bandwidth = 20e3;
  double pulse_length = 5e-3;
  double sample_rate = 40e3;
  double min_range = 2.0;
  double max_range = 60.0;
  double element_length = 0.04;   // m, along-track receiver aperture
  double element_height = 0.015;  // m, vertical receiver aperture
  double tx_length = 0.04;        // m, along-track transmitter aperture
  double tx_height = 0.015;       // m, vertical transmitter aperture
  double source_level = 0.0;      // dB; amplitude scale 10^(SL/20)
  double absorption_db_per_m = -1.0;  // < 0: use Thorp at center_frequency
  double noise_std = 1e-7;
  double cell_size = 0.04;        // m; scatterer area = cell_size^2
  uint64_t seed = 12345;
};

// Owns the FFTW plans, per-element buffers, and the resident chirp spectrum.
class EchoSynthesizer
{
public:
  // tx_pos and rx_pos are in the array frame (x = boresight, y = along-track, z = up).
  EchoSynthesizer(const Params & p, const Vector3d & tx_pos, const std::vector<Vector3d> & rx_pos);
  ~EchoSynthesizer();
  EchoSynthesizer(const EchoSynthesizer &) = delete;
  EchoSynthesizer & operator=(const EchoSynthesizer &) = delete;

  int num_samples() const {return ns_;}
  int nfft() const {return nfft_;}
  double absorption_db_per_m() const {return alpha_db_m_;}

  // Raw complex baseband echo for one ping, element-major (k * num_samples + n).
  // `array_pose` is the array frame in world at transmit, `vel` its world linear velocity.
  // Output depends only on the inputs and ping_id, not on thread count or order.
  void Synthesize(
    const std::vector<scatter::Scatterer> & scatterers, const Pose3d & array_pose,
    const Vector3d & vel, uint64_t ping_id, std::vector<float> & out_real,
    std::vector<float> & out_imag);

private:
  Params p_;
  Vector3d tx_pos_;
  std::vector<Vector3d> rx_pos_;
  int ns_ = 0;
  int nfft_ = 0;
  double alpha_db_m_ = 0.0;
  std::vector<fftwf_complex *> buf_;
  fftwf_plan fwd_ = nullptr;
  fftwf_plan inv_ = nullptr;
  std::vector<std::complex<float>> chirp_spec_;
  std::vector<double> pre_rtx_;
  std::vector<double> pre_amp_;
};

}  // namespace synth
}  // namespace sas_gazebo

#endif  // SAS_GAZEBO__SYNTH_HPP_
