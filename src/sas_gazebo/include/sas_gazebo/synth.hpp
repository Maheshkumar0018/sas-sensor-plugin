// Copyright 2026 The SAS Sensor Plugin Authors
// SPDX-License-Identifier: Apache-2.0
//
// Propagation and raw echo synthesis (REQ-003, REQ-006, REQ-008, REQ-009, REQ-012).
// Depends on ignition::math and FFTW only, so gtest can exercise it without Gazebo.

#ifndef SAS_GAZEBO__SYNTH_HPP_
#define SAS_GAZEBO__SYNTH_HPP_

#include <fftw3.h>

#include <cmath>
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

// Two-way delay with the receiver moving at `vel` while the echo travels (no stop-and-hop).  REQ-008
// Solves tau = (r_tx + rho) / c with rho = |D - v tau|, D = P - R, exactly: substituting tau gives
//   (1 - b2) rho^2 + 2 (g - b2 r_tx) rho + (2 g r_tx - b2 r_tx^2 - |D|^2) = 0,
//   g = D.v / c, b2 = |v|^2 / c^2,
// whose positive root is rho. One sqrt instead of the three of a two-step fixed-point iteration
// (plan 009), and exact rather than < 1e-6 m. Returns tau; `r_rx` receives rho.
inline double SolveDelayFromTx(
  const Vector3d & p, double r_tx, const Vector3d & rx, const Vector3d & vel, double c,
  double & r_rx)
{
  const Vector3d d = p - rx;
  const double g = d.Dot(vel) / c;
  const double b2 = vel.SquaredLength() / (c * c);
  const double qa = 1.0 - b2;
  const double qb = 2.0 * (g - b2 * r_tx);
  const double qc = 2.0 * g * r_tx - b2 * r_tx * r_tx - d.SquaredLength();
  r_rx = (-qb + std::sqrt(qb * qb - 4.0 * qa * qc)) / (2.0 * qa);
  return (r_tx + r_rx) / c;
}

inline double SolveDelay(
  const Vector3d & p, const Vector3d & tx, const Vector3d & rx, const Vector3d & vel, double c,
  double & r_rx)
{
  return SolveDelayFromTx(p, p.Distance(tx), rx, vel, c, r_rx);
}

// 8-tap Hann-windowed sinc fractional-delay kernel for taps m = n0-3 .. n0+4, with f = nf - n0 in
// [0, 1): w[i] = sinc(x) * 0.5 * (1 + cos(pi x / 4)), x = f + 3 - i.
// The taps are one sample apart, so sin(pi x) = (-1)^(3-i) sin(pi f) and cos(pi x / 4) follows from
// one sin/cos pair by angle addition: 3 trig calls instead of 16 (plan 009). Same values as the
// direct formula up to rounding.
inline void FractionalDelayTaps(double f, double w[8])
{
  const double s_pf = std::sin(M_PI * f);
  const double c4 = std::cos(M_PI * f / 4.0);
  const double s4 = std::sin(M_PI * f / 4.0);
  constexpr double kC[8] = {  // cos(pi k / 4), k = 3 - i
    -0.70710678118654752, 0.0, 0.70710678118654752, 1.0,
    0.70710678118654752, 0.0, -0.70710678118654752, -1.0};
  constexpr double kS[8] = {  // sin(pi k / 4), k = 3 - i
    0.70710678118654752, 1.0, 0.70710678118654752, 0.0,
    -0.70710678118654752, -1.0, -0.70710678118654752, 0.0};
  for (int i = 0; i < 8; ++i) {
    const int k = 3 - i;
    const double x = f + k;
    const double sinc = std::abs(x) < 1e-9 ? 1.0 :
      ((k & 1) ? -s_pf : s_pf) / (M_PI * x);  // sin(pi (f + k)) = (-1)^k sin(pi f)
    const double cos_x4 = c4 * kC[i] - s4 * kS[i];  // cos(pi f / 4 + pi k / 4)
    w[i] = sinc * 0.5 * (1.0 + cos_x4);
  }
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
  static constexpr int kChunks = 4;  // scatterer slices per element (fixed: determinism)
  std::vector<fftwf_complex *> buf_;
  std::vector<fftwf_complex *> chunk_buf_;  // n_el * kChunks impulse-train buffers
  fftwf_plan fwd_ = nullptr;
  fftwf_plan inv_ = nullptr;
  std::vector<std::complex<float>> chirp_spec_;
  std::vector<double> pre_rtx_;
  std::vector<double> pre_amp_;
};

}  // namespace synth
}  // namespace sas_gazebo

#endif  // SAS_GAZEBO__SYNTH_HPP_
