// Copyright 2026 The SAS Sensor Plugin Authors
// SPDX-License-Identifier: Apache-2.0
//
// Pure maths helpers for the SAS sensor plugin. No Gazebo or FFTW dependency,
// so gtest can exercise them directly.

#ifndef SAS_GAZEBO__DSP_HPP_
#define SAS_GAZEBO__DSP_HPP_

#include <cmath>
#include <complex>
#include <string>
#include <vector>

namespace sas_gazebo
{
namespace dsp
{

// Normalised sinc: sin(pi x) / (pi x), sinc(0) = 1.
inline double Sinc(double x)
{
  if (std::abs(x) < 1e-9) {
    return 1.0;
  }
  const double px = M_PI * x;
  return std::sin(px) / px;
}

// Uniform line-aperture beam pattern b(theta) = sinc(D sin(theta) / lambda).  REQ-003
inline double BeamPattern(double aperture, double wavelength, double sin_theta)
{
  return Sinc(aperture * sin_theta / wavelength);
}

// Number of samples covering `seconds` at rate `fs`, rounded up.
inline int CeilSamples(double seconds, double fs)
{
  return static_cast<int>(std::ceil(seconds * fs - 1e-9));
}

// Smallest power of two >= n (n >= 1).
inline int NextPow2(int n)
{
  int p = 1;
  while (p < n) {
    p <<= 1;
  }
  return p;
}

// Complex baseband LFM chirp p(t) = exp(j pi K (t - Tp/2)^2), t = n / fs, 0 <= t < Tp,
// K = B / Tp. Instantaneous frequency sweeps -B/2 .. +B/2.  REQ-001
inline std::vector<std::complex<float>> ChirpSamples(double fs, double bandwidth, double pulse_length)
{
  const int np = CeilSamples(pulse_length, fs);
  const double k_rate = bandwidth / pulse_length;
  std::vector<std::complex<float>> p(static_cast<size_t>(np));
  for (int n = 0; n < np; ++n) {
    const double t = n / fs - 0.5 * pulse_length;
    const double ph = M_PI * k_rate * t * t;
    p[static_cast<size_t>(n)] = std::complex<float>(
      static_cast<float>(std::cos(ph)), static_cast<float>(std::sin(ph)));
  }
  return p;
}

// Along-track (array y-axis) receiver positions, centred on the array origin:
// y_k = (k - (N - 1) / 2) * d.  REQ-002
inline std::vector<double> ReceiverPositionsY(int num_elements, double spacing)
{
  std::vector<double> y(static_cast<size_t>(num_elements));
  for (int k = 0; k < num_elements; ++k) {
    y[static_cast<size_t>(k)] = (k - 0.5 * (num_elements - 1)) * spacing;
  }
  return y;
}

// Design checks (REQ-013). They only drive warnings and never change the output.

// Echoes from max_range must arrive before the next ping: 2 R_max / c <= PRI.
inline bool RangeAmbiguous(double max_range, double sound_speed, double pri)
{
  return 2.0 * max_range / sound_speed > pri;
}

// DPCA along-track sampling: the array may advance at most half its length per ping,
// v_along * PRI <= N d / 2, or the synthetic aperture is under-sampled (aliasing).
inline bool DpcaViolated(double v_along, double pri, int num_elements, double spacing)
{
  return std::abs(v_along) * pri > 0.5 * num_elements * spacing;
}

// Ping timing: true when the measured interval differs from the PRI by more than `rel_tol`
// (Gazebo without --lockstep may skip or stretch sensor updates).
inline bool PingIntervalOff(double interval, double pri, double rel_tol = 0.01)
{
  return std::abs(interval - pri) > rel_tol * pri;
}

// Sensor parameters read from SDF. Defaults follow reference doc Section 2.3.
struct SonarParams
{
  double sound_speed = 1500.0;        // m/s
  double center_frequency = 100e3;    // Hz
  double bandwidth = 20e3;            // Hz
  double pulse_length = 5e-3;         // s
  double sample_rate = 40e3;          // Hz, complex baseband
  double min_range = 2.0;             // m
  double max_range = 60.0;            // m
  int num_elements = 16;
  double element_spacing = 0.04;      // m
  double element_length = 0.04;       // m, along-track receiver aperture
  double tx_length = 0.04;            // m, along-track transmitter aperture
  double source_level = 0.0;          // dB re 1 uPa @ 1 m
  double noise_std = 1e-7;            // per-component std of receiver noise
  int seed = 12345;
  std::string topic = "/sas/ping";
  std::string frame_id = "sas_array_link";
};

// Returns one message per invalid parameter; empty when valid.
inline std::vector<std::string> Validate(const SonarParams & p)
{
  std::vector<std::string> e;
  if (!(p.sound_speed > 0.0)) {e.push_back("sound_speed must be > 0");}
  if (!(p.center_frequency > 0.0)) {e.push_back("center_frequency must be > 0");}
  if (!(p.bandwidth > 0.0)) {e.push_back("bandwidth must be > 0");}
  if (!(p.bandwidth < 2.0 * p.center_frequency)) {
    e.push_back("bandwidth must be < 2 * center_frequency");
  }
  if (!(p.pulse_length > 0.0)) {e.push_back("pulse_length must be > 0");}
  if (!(p.sample_rate >= p.bandwidth)) {
    e.push_back("sample_rate must be >= bandwidth (complex baseband)");
  }
  if (!(p.min_range >= 0.0)) {e.push_back("min_range must be >= 0");}
  if (!(p.max_range > p.min_range)) {e.push_back("max_range must be > min_range");}
  if (p.num_elements < 1) {e.push_back("num_elements must be >= 1");}
  if (!(p.element_spacing > 0.0)) {e.push_back("element_spacing must be > 0");}
  if (!(p.element_length > 0.0)) {e.push_back("element_length must be > 0");}
  if (!(p.tx_length > 0.0)) {e.push_back("tx_length must be > 0");}
  if (!(p.noise_std >= 0.0)) {e.push_back("noise_std must be >= 0");}
  if (p.topic.empty()) {e.push_back("topic must not be empty");}
  return e;
}

}  // namespace dsp
}  // namespace sas_gazebo

#endif  // SAS_GAZEBO__DSP_HPP_
