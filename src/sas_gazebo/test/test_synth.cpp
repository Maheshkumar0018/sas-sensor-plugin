// Copyright 2026 The SAS Sensor Plugin Authors
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>
#include <fftw3.h>
#include <omp.h>

#include <cmath>
#include <complex>
#include <vector>

#include "sas_gazebo/dsp.hpp"
#include "sas_gazebo/scatter.hpp"
#include "sas_gazebo/synth.hpp"

namespace sy = sas_gazebo::synth;
namespace sc = sas_gazebo::scatter;
using ignition::math::Pose3d;
using ignition::math::Vector3d;

namespace
{

sy::Params NoNoise()
{
  sy::Params p;
  p.noise_std = 0.0;
  return p;
}

// Point scatterer facing the array origin: mu = 1, speckle = 1.
sc::Scatterer Point(const Vector3d & pos)
{
  sc::Scatterer s;
  s.pos = pos;
  s.normal = (-pos).Normalized();
  s.mu = 1.0;
  s.speckle = 1.0;
  return s;
}

// Offline matched filter of one element row; returns |y| peak index and the refined
// (parabolic) fractional index and magnitude.
struct Peak
{
  int n = 0;
  double frac = 0.0;
  double mag = 0.0;
};

Peak MatchedPeak(const sy::Params & p, const float * re, const float * im, int ns)
{
  const auto chirp = sas_gazebo::dsp::ChirpSamples(p.sample_rate, p.bandwidth, p.pulse_length);
  const int nfft = sas_gazebo::dsp::NextPow2(ns + static_cast<int>(chirp.size()) + 8);
  // Single precision: only libfftw3f is guaranteed (the plugin links it too).
  std::vector<std::complex<float>> x(nfft), h(nfft);
  for (int n = 0; n < ns; ++n) {x[n] = {re[n], im[n]};}
  for (size_t n = 0; n < chirp.size(); ++n) {h[n] = chirp[n];}
  auto fft = [nfft](std::vector<std::complex<float>> & v, int dir) {
      auto * d = reinterpret_cast<fftwf_complex *>(v.data());
      fftwf_plan pl = fftwf_plan_dft_1d(nfft, d, d, dir, FFTW_ESTIMATE);
      fftwf_execute(pl);
      fftwf_destroy_plan(pl);
    };
  fft(x, FFTW_FORWARD);
  fft(h, FFTW_FORWARD);
  for (int n = 0; n < nfft; ++n) {x[n] *= std::conj(h[n]) / static_cast<float>(nfft);}
  fft(x, FFTW_BACKWARD);
  Peak pk;
  for (int n = 0; n < ns; ++n) {
    if (std::abs(x[n]) > pk.mag) {pk.mag = std::abs(x[n]); pk.n = n;}
  }
  pk.frac = pk.n;
  if (pk.n > 0 && pk.n + 1 < ns) {
    const double a = std::abs(x[pk.n - 1]), b = std::abs(x[pk.n]);
    const double c = std::abs(x[pk.n + 1]);
    const double den = a - 2.0 * b + c;
    if (std::abs(den) > 0.0) {
      const double d = 0.5 * (a - c) / den;
      pk.frac = pk.n + d;
      pk.mag = b - 0.25 * (a - c) * d;
    }
  }
  return pk;
}

std::vector<Vector3d> Array(int n, double d)
{
  std::vector<Vector3d> rx;
  for (double y : sas_gazebo::dsp::ReceiverPositionsY(n, d)) {rx.emplace_back(0.0, y, 0.0);}
  return rx;
}

}  // namespace

TEST(Propagation, ThorpAt100kHz)  // REQ-006
{
  // 0.11 f2/(1+f2) + 44 f2/(4100+f2) + 2.75e-4 f2 + 0.003 at f = 100 kHz = 34.0687 dB/km.
  EXPECT_NEAR(sy::ThorpDbPerMeter(100e3), 0.0340687, 1e-6);
}

TEST(Propagation, DelaySolverStaticExactMovingSubMicron)  // REQ-008
{
  const Vector3d p(25.0, 12.0, -8.0), tx(0, 0, 0), rx(0, 0.3, 0);
  double r_rx = 0.0;
  const double tau0 = sy::SolveDelay(p, tx, rx, Vector3d::Zero, 1500.0, r_rx);
  EXPECT_DOUBLE_EQ(tau0, (p.Distance(tx) + p.Distance(rx)) / 1500.0);
  const Vector3d v(0.0, 3.0, 0.0);
  const double tau1 = sy::SolveDelay(p, tx, rx, v, 1500.0, r_rx);
  double tau = tau0;  // converge by iteration for the exact answer
  for (int i = 0; i < 50; ++i) {tau = (p.Distance(tx) + p.Distance(rx + v * tau)) / 1500.0;}
  EXPECT_LT(std::abs(tau1 - tau) * 1500.0, 1e-6);  // path error < 1 um
  EXPECT_GT(std::abs(tau1 - tau0) * 1500.0, 1e-3);  // and motion does matter
}

TEST(Synthesis, PointTargetDelayOnEveryElement)  // REQ-001, REQ-002, REQ-009
{
  const auto p = NoNoise();
  const auto rx = Array(16, 0.04);
  sy::EchoSynthesizer s(p, Vector3d::Zero, rx);
  const Vector3d target(20.0, 0.5, -1.0);
  std::vector<float> re, im;
  s.Synthesize({Point(target)}, Pose3d::Zero, Vector3d::Zero, 0, re, im);
  ASSERT_EQ(re.size(), 16u * s.num_samples());
  for (int k = 0; k < 16; ++k) {
    const double tau = (target.Length() + target.Distance(rx[k])) / p.sound_speed;
    const auto pk = MatchedPeak(p, &re[k * s.num_samples()], &im[k * s.num_samples()],
      s.num_samples());
    EXPECT_NEAR(pk.frac, tau * p.sample_rate, 0.1) << "element " << k;
  }
}

TEST(Synthesis, SpreadingAndAbsorption)  // REQ-006
{
  auto p = NoNoise();
  std::vector<float> re, im;
  sy::EchoSynthesizer s(p, Vector3d::Zero, {Vector3d::Zero});
  // R = 15 m and 30 m give integer sample delays (800, 1600), so no scalloping.
  s.Synthesize({Point({15.0, 0, 0})}, Pose3d::Zero, Vector3d::Zero, 0, re, im);
  const double a15 = MatchedPeak(p, re.data(), im.data(), s.num_samples()).mag;
  s.Synthesize({Point({30.0, 0, 0})}, Pose3d::Zero, Vector3d::Zero, 0, re, im);
  const double a30 = MatchedPeak(p, re.data(), im.data(), s.num_samples()).mag;
  const double expected_db = 20.0 * std::log10(30.0 * 30.0 / (15.0 * 15.0)) +
    s.absorption_db_per_m() * (60.0 - 30.0);
  EXPECT_NEAR(20.0 * std::log10(a15 / a30), expected_db, 0.05);
}

TEST(Synthesis, BeamNullSuppressesTarget)  // REQ-003
{
  const auto p = NoNoise();
  std::vector<float> re, im;
  sy::EchoSynthesizer s(p, Vector3d::Zero, {Vector3d::Zero});
  const double lambda = p.sound_speed / p.center_frequency;
  const double sin_null = lambda / p.element_length;  // first null, tx_length = element_length
  const double r = 20.0;
  s.Synthesize({Point({r, 0, 0})}, Pose3d::Zero, Vector3d::Zero, 0, re, im);
  const double on_axis = MatchedPeak(p, re.data(), im.data(), s.num_samples()).mag;
  s.Synthesize({Point({r * std::sqrt(1 - sin_null * sin_null), r * sin_null, 0})}, Pose3d::Zero,
    Vector3d::Zero, 0, re, im);
  const double at_null = MatchedPeak(p, re.data(), im.data(), s.num_samples()).mag;
  EXPECT_LT(at_null, 1e-6 * on_axis);
}

TEST(Synthesis, ReceiverMotionShiftsDelayAsModelled)  // REQ-008
{
  const auto p = NoNoise();
  std::vector<float> re, im;
  const Vector3d rx(0.0, 0.3, 0.0);
  sy::EchoSynthesizer s(p, Vector3d::Zero, {rx});
  const Vector3d target(20.0, 10.0, 0.0);  // squinted, so along-track motion changes range
  const Vector3d v(0.0, 1.5, 0.0);
  s.Synthesize({Point(target)}, Pose3d::Zero, Vector3d::Zero, 0, re, im);
  const double n_static = MatchedPeak(p, re.data(), im.data(), s.num_samples()).frac;
  s.Synthesize({Point(target)}, Pose3d::Zero, v, 0, re, im);
  const double n_moving = MatchedPeak(p, re.data(), im.data(), s.num_samples()).frac;
  double r_rx = 0.0;
  const double shift = (sy::SolveDelay(target, Vector3d::Zero, rx, v, p.sound_speed, r_rx) -
    sy::SolveDelay(target, Vector3d::Zero, rx, Vector3d::Zero, p.sound_speed, r_rx)) *
    p.sample_rate;
  ASSERT_LT(shift, -0.3);  // measurable: about -0.7 sample
  EXPECT_NEAR(n_moving - n_static, shift, 0.1);
}

TEST(Synthesis, NoiseStatisticsAndReproducibility)  // REQ-009, REQ-012
{
  auto p = NoNoise();
  p.noise_std = 0.01;
  sy::EchoSynthesizer s(p, Vector3d::Zero, Array(4, 0.04));
  std::vector<float> re, im, re2, im2;
  s.Synthesize({}, Pose3d::Zero, Vector3d::Zero, 7, re, im);
  double sum = 0.0, sq = 0.0;
  for (size_t i = 0; i < re.size(); ++i) {sum += re[i] + im[i]; sq += re[i] * re[i] + im[i] * im[i];}
  const double n = 2.0 * re.size();
  EXPECT_NEAR(sum / n, 0.0, 3e-4);
  EXPECT_NEAR(std::sqrt(sq / n), 0.01, 2e-4);
  s.Synthesize({}, Pose3d::Zero, Vector3d::Zero, 7, re2, im2);
  EXPECT_EQ(re, re2);
  EXPECT_EQ(im, im2);
  s.Synthesize({}, Pose3d::Zero, Vector3d::Zero, 8, re2, im2);
  EXPECT_NE(re, re2);
}

TEST(Synthesis, IndependentOfThreadCount)  // REQ-012
{
  auto p = NoNoise();
  p.noise_std = 1e-6;
  std::vector<sc::Scatterer> field;
  for (int i = 0; i < 3000; ++i) {
    sc::Scatterer s;
    sc::VoxelScatterer(i % 60, i / 60 - 25, -200, 0.04, 99, s);
    s.pos += Vector3d(15.0, 0.0, 3.0);  // flat patch 15-18 m ahead, 5 m below the array
    s.normal = Vector3d(0, 0, 1);
    s.mu = 0.01;
    field.push_back(s);
  }
  const Pose3d pose(0.3, -0.2, 0.0, 0.0, 0.3, 0.05);
  sy::EchoSynthesizer s(p, Vector3d::Zero, Array(16, 0.04));
  std::vector<float> re1, im1, re8, im8;
  omp_set_num_threads(1);
  s.Synthesize(field, pose, Vector3d(0, 1.5, 0), 3, re1, im1);
  omp_set_num_threads(8);
  s.Synthesize(field, pose, Vector3d(0, 1.5, 0), 3, re8, im8);
  float peak = 0.0f;
  for (float v : re1) {peak = std::max(peak, std::abs(v));}
  ASSERT_GT(peak, 10.0f * static_cast<float>(p.noise_std));  // echo present, not just noise
  EXPECT_EQ(re1, re8);
  EXPECT_EQ(im1, im8);
}
