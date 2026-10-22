// Copyright 2026 The SAS Sensor Plugin Authors
// SPDX-License-Identifier: Apache-2.0

#include "sas_gazebo/synth.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <random>
#include <vector>

#include "sas_gazebo/dsp.hpp"

namespace sas_gazebo
{
namespace synth
{

EchoSynthesizer::EchoSynthesizer(
  const Params & p, const Vector3d & tx_pos, const std::vector<Vector3d> & rx_pos)
: p_(p), tx_pos_(tx_pos), rx_pos_(rx_pos)
{
  alpha_db_m_ = p_.absorption_db_per_m >= 0.0 ?
    p_.absorption_db_per_m : ThorpDbPerMeter(p_.center_frequency);

  // REQ-001: chirp spectrum computed once and kept resident.
  ns_ = dsp::CeilSamples(2.0 * p_.max_range / p_.sound_speed, p_.sample_rate);
  const auto chirp = dsp::ChirpSamples(p_.sample_rate, p_.bandwidth, p_.pulse_length);
  // Room for the latest echo (ns) plus the chirp tail and the 8-tap kernel: no circular wrap.
  nfft_ = dsp::NextPow2(ns_ + static_cast<int>(chirp.size()) + 8);

  buf_.resize(rx_pos_.size());
  for (auto & b : buf_) {
    b = static_cast<fftwf_complex *>(fftwf_malloc(sizeof(fftwf_complex) * nfft_));
  }
  chunk_buf_.resize(rx_pos_.size() * kChunks);
  for (auto & b : chunk_buf_) {
    b = static_cast<fftwf_complex *>(fftwf_malloc(sizeof(fftwf_complex) * nfft_));
  }
  // FFTW_ESTIMATE, not FFTW_MEASURE: the plan (and so the rounding) must not depend on timing,
  // otherwise reruns are not bit-identical (REQ-012). Plans are reused on every element buffer
  // with fftwf_execute_dft, which is thread-safe; fftwf_malloc gives equal alignment.
  fwd_ = fftwf_plan_dft_1d(nfft_, buf_[0], buf_[0], FFTW_FORWARD, FFTW_ESTIMATE);
  inv_ = fftwf_plan_dft_1d(nfft_, buf_[0], buf_[0], FFTW_BACKWARD, FFTW_ESTIMATE);

  std::memset(buf_[0], 0, sizeof(fftwf_complex) * nfft_);
  for (size_t n = 0; n < chirp.size(); ++n) {
    buf_[0][n][0] = chirp[n].real();
    buf_[0][n][1] = chirp[n].imag();
  }
  fftwf_execute_dft(fwd_, buf_[0], buf_[0]);
  chirp_spec_.resize(static_cast<size_t>(nfft_));
  for (int n = 0; n < nfft_; ++n) {
    chirp_spec_[static_cast<size_t>(n)] = std::complex<float>(buf_[0][n][0], buf_[0][n][1]);
  }
}

EchoSynthesizer::~EchoSynthesizer()
{
  for (auto * b : buf_) {
    fftwf_free(b);
  }
  for (auto * b : chunk_buf_) {
    fftwf_free(b);
  }
  if (fwd_) {fftwf_destroy_plan(fwd_);}
  if (inv_) {fftwf_destroy_plan(inv_);}
}

void EchoSynthesizer::Synthesize(
  const std::vector<scatter::Scatterer> & scatterers, const Pose3d & pose, const Vector3d & vel,
  uint64_t ping_id, std::vector<float> & out_real, std::vector<float> & out_imag)
{
  using cd = std::complex<double>;
  const int n_el = static_cast<int>(rx_pos_.size());
  const int64_t n_sc = static_cast<int64_t>(scatterers.size());
  const Vector3d tx_w = pose.CoordPositionAdd(tx_pos_);
  std::vector<Vector3d> rx_w(rx_pos_.size());
  for (int k = 0; k < n_el; ++k) {
    rx_w[k] = pose.CoordPositionAdd(rx_pos_[k]);
  }

  const double c = p_.sound_speed;
  const double fc = p_.center_frequency;
  const double fs = p_.sample_rate;
  const double lambda = c / fc;
  const double src_amp = std::pow(10.0, p_.source_level / 20.0);
  const double alpha_np = alpha_db_m_ * std::log(10.0) / 20.0;  // amplitude attenuation, Np/m
  const double cell_area = p_.cell_size * p_.cell_size;

  // Per-scatterer terms shared by all elements: transmit range, beams, Lambert.  REQ-003, REQ-006
  pre_rtx_.resize(scatterers.size());
  pre_amp_.resize(scatterers.size());
  #pragma omp parallel for schedule(static)
  for (int64_t i = 0; i < n_sc; ++i) {
    const auto & sc = scatterers[static_cast<size_t>(i)];
    const Vector3d d = sc.pos - tx_w;
    const double r_tx = d.Length();
    pre_rtx_[i] = r_tx;
    pre_amp_[i] = 0.0;
    if (r_tx < p_.min_range || r_tx > p_.max_range) {
      continue;
    }
    const Vector3d u = pose.Rot().RotateVectorReverse(d / r_tx);  // direction in array frame
    if (u.X() <= 0.0) {
      continue;  // behind the array
    }
    // b(theta) = sinc(D sin(theta) / lambda), along-track (y) and vertical (z) apertures.
    const double b_tx = dsp::BeamPattern(p_.tx_length, lambda, u.Y()) *
      dsp::BeamPattern(p_.tx_height, lambda, u.Z());
    const double b_rx = dsp::BeamPattern(p_.element_length, lambda, u.Y()) *
      dsp::BeamPattern(p_.element_height, lambda, u.Z());
    // Lambert: sigma = mu cos^2(incidence) A_cell; normal points toward the sonar.
    const double cos_inc = std::max(0.0, -sc.normal.Dot(d / r_tx));
    const double sigma = sc.mu * cos_inc * cos_inc * cell_area;
    pre_amp_[i] = src_amp * std::sqrt(sigma) * b_tx * b_rx / r_tx;
  }

  const size_t total = static_cast<size_t>(n_el) * static_cast<size_t>(ns_);
  out_real.assign(total, 0.0f);
  out_imag.assign(total, 0.0f);
  const float inv_nfft = 1.0f / static_cast<float>(nfft_);

  // Impulse trains: each (element, chunk) task sums a fixed slice of the scatterers in index order
  // into its own buffer (plan 009: kChunks x elements tasks keep all cores busy). The partition does
  // not depend on the thread count, and chunks are added in chunk order below, so the result is
  // the same for any number of threads (REQ-012).
  const int64_t chunk_len = (n_sc + kChunks - 1) / kChunks;
  #pragma omp parallel for schedule(dynamic)
  for (int task = 0; task < n_el * kChunks; ++task) {
    const int k = task / kChunks;
    const int chunk = task % kChunks;
    fftwf_complex * h = chunk_buf_[static_cast<size_t>(task)];
    double w8[8];
    std::memset(h, 0, sizeof(fftwf_complex) * nfft_);
    const Vector3d r0 = rx_w[static_cast<size_t>(k)];
    const int64_t i_end = std::min(n_sc, (chunk + 1) * chunk_len);

    for (int64_t i = chunk * chunk_len; i < i_end; ++i) {
      const double amp0 = pre_amp_[i];
      if (amp0 == 0.0) {
        continue;
      }
      const auto & sc = scatterers[static_cast<size_t>(i)];
      // REQ-008: receiver keeps moving while the echo propagates.
      const double r_tx = pre_rtx_[i];
      double r_rx = 0.0;
      const double tau = SolveDelayFromTx(sc.pos, r_tx, r0, vel, c, r_rx);

      // Spreading 1/(R_tx R_rx) (R_tx already applied), Thorp, carrier phase e^{-j 2 pi fc tau}.
      const double amp = amp0 / r_rx * std::exp(-alpha_np * (r_tx + r_rx));
      const cd a = amp * sc.speckle * std::polar(1.0, -2.0 * M_PI * fc * tau);

      // Band-limited fractional delay: 8-tap Hann-windowed sinc centred on tau * fs.
      const double nf = tau * fs;
      const int n0 = static_cast<int>(std::floor(nf));
      if (n0 - 3 < 0 || n0 + 4 >= nfft_) {
        continue;
      }
      FractionalDelayTaps(nf - n0, w8);
      for (int i = 0; i < 8; ++i) {
        h[n0 - 3 + i][0] += static_cast<float>(a.real() * w8[i]);
        h[n0 - 3 + i][1] += static_cast<float>(a.imag() * w8[i]);
      }
    }
  }

  #pragma omp parallel for schedule(dynamic)
  for (int k = 0; k < n_el; ++k) {
    fftwf_complex * h = buf_[static_cast<size_t>(k)];
    std::memcpy(h, chunk_buf_[static_cast<size_t>(k * kChunks)], sizeof(fftwf_complex) * nfft_);
    for (int chunk = 1; chunk < kChunks; ++chunk) {
      const fftwf_complex * g = chunk_buf_[static_cast<size_t>(k * kChunks + chunk)];
      for (int n = 0; n < nfft_; ++n) {
        h[n][0] += g[n][0];
        h[n][1] += g[n][1];
      }
    }

    // Raw echo = impulse train convolved with the chirp (FFT multiply).  REQ-009
    fftwf_execute_dft(fwd_, h, h);
    for (int n = 0; n < nfft_; ++n) {
      const std::complex<float> v =
        std::complex<float>(h[n][0], h[n][1]) * chirp_spec_[static_cast<size_t>(n)];
      h[n][0] = v.real() * inv_nfft;
      h[n][1] = v.imag() * inv_nfft;
    }
    fftwf_execute_dft(inv_, h, h);

    // Receiver noise, seeded per (seed, ping, element): reproducible, thread-order independent.
    std::mt19937_64 gen(p_.seed ^ scatter::SplitMix64(ping_id * 1315423911ULL + k));
    std::normal_distribution<float> noise(0.0f, static_cast<float>(p_.noise_std));
    const size_t off = static_cast<size_t>(k) * static_cast<size_t>(ns_);
    for (int n = 0; n < ns_; ++n) {
      out_real[off + n] = h[n][0] + noise(gen);
      out_imag[off + n] = h[n][1] + noise(gen);
    }
  }
}

}  // namespace synth
}  // namespace sas_gazebo
