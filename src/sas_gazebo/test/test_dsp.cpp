// Copyright 2026 The SAS Sensor Plugin Authors
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <cmath>
#include <complex>
#include <vector>

#include "sas_gazebo/dsp.hpp"

using sas_gazebo::dsp::BeamPattern;
using sas_gazebo::dsp::DpcaViolated;
using sas_gazebo::dsp::PingIntervalOff;
using sas_gazebo::dsp::RangeAmbiguous;
using sas_gazebo::dsp::ChirpSamples;
using sas_gazebo::dsp::NextPow2;
using sas_gazebo::dsp::ReceiverPositionsY;
using sas_gazebo::dsp::Sinc;
using sas_gazebo::dsp::SonarParams;
using sas_gazebo::dsp::Validate;

// REQ-001: unit modulus, sweep from -B/2 to +B/2.
TEST(Chirp, UnitModulusAndSweep)
{
  const double fs = 40e3, bw = 20e3, tp = 5e-3;
  const auto p = ChirpSamples(fs, bw, tp);
  ASSERT_EQ(p.size(), 200u);
  for (const auto & s : p) {
    EXPECT_NEAR(std::abs(s), 1.0, 1e-6);
  }
  // Instantaneous frequency from phase difference of neighbouring samples.
  auto inst_f = [&](size_t n) {
      return std::arg(p[n + 1] * std::conj(p[n])) * fs / (2.0 * M_PI);
    };
  const double k_rate = bw / tp;
  const double step = k_rate / fs;  // frequency change per sample
  EXPECT_NEAR(inst_f(0), -bw / 2.0, step);
  EXPECT_NEAR(inst_f(p.size() - 2), bw / 2.0, 2.0 * step);
  for (size_t n = 0; n + 2 < p.size(); ++n) {
    EXPECT_GT(inst_f(n + 1), inst_f(n));  // monotonic up-chirp
  }
}

// REQ-003
TEST(Beam, SincAndFirstNull)
{
  EXPECT_DOUBLE_EQ(Sinc(0.0), 1.0);
  EXPECT_NEAR(Sinc(1.0), 0.0, 1e-12);
  EXPECT_NEAR(Sinc(0.5), 2.0 / M_PI, 1e-12);
  const double lambda = 1500.0 / 100e3, d = 0.04;
  EXPECT_DOUBLE_EQ(BeamPattern(d, lambda, 0.0), 1.0);
  EXPECT_NEAR(BeamPattern(d, lambda, lambda / d), 0.0, 1e-12);  // first null
  EXPECT_GT(BeamPattern(d, lambda, 0.5 * lambda / d), 0.5);
}

// REQ-002
TEST(Array, ReceiverPositionsCentred)
{
  const auto y = ReceiverPositionsY(16, 0.04);
  ASSERT_EQ(y.size(), 16u);
  EXPECT_NEAR(y.front(), -0.30, 1e-12);
  EXPECT_NEAR(y.back(), 0.30, 1e-12);
  double sum = 0.0;
  for (double v : y) {sum += v;}
  EXPECT_NEAR(sum, 0.0, 1e-12);
  EXPECT_NEAR(y[1] - y[0], 0.04, 1e-12);
  EXPECT_DOUBLE_EQ(ReceiverPositionsY(1, 0.04)[0], 0.0);
}

TEST(Params, DefaultsValidAndBadValuesRejected)
{
  SonarParams p;
  EXPECT_TRUE(Validate(p).empty());
  p.sample_rate = p.bandwidth / 2.0;
  EXPECT_EQ(Validate(p).size(), 1u);
  p = SonarParams{};
  p.num_elements = 0;
  p.max_range = 1.0;  // < min_range
  EXPECT_EQ(Validate(p).size(), 2u);
}

TEST(Fft, NextPow2)
{
  EXPECT_EQ(NextPow2(1), 1);
  EXPECT_EQ(NextPow2(3200 + 200 + 8), 4096);
  EXPECT_EQ(NextPow2(4096), 4096);
}

// REQ-013: defaults (60 m, 1500 m/s, 10 Hz) are unambiguous; 80 m is not (2*80/1500 > 0.1 s).
TEST(DesignChecks, RangeAmbiguity)
{
  EXPECT_FALSE(RangeAmbiguous(60.0, 1500.0, 0.1));
  EXPECT_FALSE(RangeAmbiguous(75.0, 1500.0, 0.1));  // exactly at the limit
  EXPECT_TRUE(RangeAmbiguous(80.0, 1500.0, 0.1));
}

// REQ-013: 16 x 0.04 m array at 10 Hz allows N d / (2 PRI) = 3.2 m/s along-track.
TEST(DesignChecks, DpcaLimit)
{
  EXPECT_FALSE(DpcaViolated(1.5, 0.1, 16, 0.04));
  EXPECT_FALSE(DpcaViolated(3.19, 0.1, 16, 0.04));
  EXPECT_TRUE(DpcaViolated(3.21, 0.1, 16, 0.04));
  EXPECT_TRUE(DpcaViolated(-3.21, 0.1, 16, 0.04));  // direction does not matter
}

TEST(DesignChecks, PingInterval)
{
  EXPECT_FALSE(PingIntervalOff(0.1, 0.1));
  EXPECT_FALSE(PingIntervalOff(0.1009, 0.1));
  EXPECT_TRUE(PingIntervalOff(0.2, 0.1));  // a skipped update
}
