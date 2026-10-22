// Copyright 2026 The SAS Sensor Plugin Authors
// SPDX-License-Identifier: Apache-2.0
//
// Scene reconstruction and world-fixed scatterer field (REQ-004, REQ-005, REQ-007, ADR-0004).
// Depends only on ignition::math, so gtest can exercise it without Gazebo.

#ifndef SAS_GAZEBO__SCATTER_HPP_
#define SAS_GAZEBO__SCATTER_HPP_

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdint>
#include <vector>

#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

namespace sas_gazebo
{
namespace scatter
{

using ignition::math::Pose3d;
using ignition::math::Vector3d;

// SplitMix64 finaliser: stateless 64-bit hash.
inline uint64_t SplitMix64(uint64_t x)
{
  x += 0x9E3779B97F4A7C15ULL;
  x = (x ^ (x >> 30)) * 0xBF58476D1CE4E5B9ULL;
  x = (x ^ (x >> 27)) * 0x94D049BB133111EBULL;
  return x ^ (x >> 31);
}

// Uniform in the open interval (0, 1) from a 64-bit hash.
inline double U01(uint64_t h)
{
  return (static_cast<double>(h >> 11) + 0.5) * (1.0 / 9007199254740992.0);
}

// Packs a signed voxel index into 63 bits, 21 bits per axis (±2^20 cells per axis).
inline uint64_t CellKey(int64_t ix, int64_t iy, int64_t iz)
{
  constexpr int64_t kOffset = 1 << 20;
  constexpr uint64_t kMask = (1ULL << 21) - 1;
  return ((static_cast<uint64_t>(ix + kOffset) & kMask) << 42) |
         ((static_cast<uint64_t>(iy + kOffset) & kMask) << 21) |
         (static_cast<uint64_t>(iz + kOffset) & kMask);
}

// One range image from the ray sensor. Index = v * nh + h.
struct Scan
{
  int nh = 0;
  int nv = 0;
  double hmin = 0.0, hres = 0.0, vmin = 0.0, vres = 0.0;
  double rmin = 0.0, rmax = 0.0;
  std::vector<double> ranges;
  std::vector<double> retro;
};

// One world-fixed voxel scatterer.
struct Scatterer
{
  uint64_t key = 0;            // CellKey of the voxel
  Vector3d pos;                // world position: voxel centre + deterministic jitter
  Vector3d normal;             // unit surface normal, oriented toward the sonar
  double mu = 0.0;             // Lambert backscatter coefficient, linear
  std::complex<double> speckle;  // unit-variance complex Gaussian, fixed per voxel
};

struct Params
{
  double cell_size = 0.04;         // m
  // rad (88 deg). Quads seen more edge-on are rejected, which creates shadows. A depth jump dR
  // between rays dtheta apart at range R is only rejected when R * dtheta / dR < cos(max_incidence),
  // so shadows need fine ray resolution (512 rays over the fan: ~0.002 rad).
  double max_incidence = 1.5359;
  double mu_default = 0.002;       // linear; used when laser_retro is 0 (-27 dB)
  double min_range = 2.0;          // m
  double max_range = 60.0;         // m
  uint64_t seed = 12345;
};

// Voxel index of coordinate v. Voxels are centred on multiples of `cell`, so surfaces at round
// heights (z = 0, z = -10) sit mid-voxel instead of on a boundary where float noise would split
// them across two voxel layers from ping to ping.
inline int64_t VoxelIndex(double v, double cell)
{
  return static_cast<int64_t>(std::floor(v / cell + 0.5));
}

// Inverse of the z part of CellKey (for tests and debugging).
inline int64_t CellKeyZ(uint64_t key)
{
  constexpr int64_t kOffset = 1 << 20;
  constexpr uint64_t kMask = (1ULL << 21) - 1;
  return static_cast<int64_t>(key & kMask) - kOffset;
}

// Deterministic position and speckle of voxel (ix, iy, iz). Same inputs -> same output. REQ-005
inline void VoxelScatterer(
  int64_t ix, int64_t iy, int64_t iz, double cell, uint64_t seed, Scatterer & sc)
{
  sc.key = CellKey(ix, iy, iz);
  const uint64_t h0 = SplitMix64(sc.key ^ seed);
  const uint64_t h1 = SplitMix64(h0);
  const uint64_t h2 = SplitMix64(h1);
  const uint64_t h3 = SplitMix64(h2);
  const uint64_t h4 = SplitMix64(h3);
  // Jitter within ±0.4 cell of the voxel centre (ix, iy, iz) * cell breaks the regular grid.
  sc.pos = Vector3d(
    (ix + 0.8 * (U01(h0) - 0.5)) * cell,
    (iy + 0.8 * (U01(h1) - 0.5)) * cell,
    (iz + 0.8 * (U01(h2) - 0.5)) * cell);
  // |xi|^2 ~ Exp(1), uniform phase -> unit-variance circular complex Gaussian.
  const double rad = std::sqrt(-std::log(U01(h3)));
  const double ang = 2.0 * M_PI * U01(h4);
  sc.speckle = std::complex<double>(rad * std::cos(ang), rad * std::sin(ang));
}

// Set of 64-bit keys for per-ping voxel de-duplication (plan 009). Open addressing with linear
// probing; a generation counter empties it in O(1) per ping, so it never reallocates once it is
// large enough. Only answers "seen before?", so the scatterer order is unchanged.
class KeySet
{
public:
  void Clear()
  {
    if (++gen_ == 0) {  // wrapped: reset stamps once every 2^32 pings
      std::fill(stamp_.begin(), stamp_.end(), 0u);
      gen_ = 1;
    }
    size_ = 0;
  }

  // Inserts key; returns true if it was not present.
  bool Insert(uint64_t key)
  {
    if (2 * (size_ + 1) > keys_.size()) {
      Grow();
    }
    const size_t mask = keys_.size() - 1;
    for (size_t i = SplitMix64(key) & mask;; i = (i + 1) & mask) {
      if (stamp_[i] != gen_) {
        stamp_[i] = gen_;
        keys_[i] = key;
        ++size_;
        return true;
      }
      if (keys_[i] == key) {
        return false;
      }
    }
  }

  size_t size() const {return size_;}

private:
  void Grow()
  {
    std::vector<uint64_t> old_keys;
    std::vector<uint32_t> old_stamp;
    old_keys.swap(keys_);
    old_stamp.swap(stamp_);
    const uint32_t old_gen = gen_;
    const size_t cap = std::max<size_t>(1u << 16, 2 * old_keys.size());
    keys_.assign(cap, 0);
    stamp_.assign(cap, 0u);
    gen_ = 1;
    size_ = 0;
    for (size_t i = 0; i < old_keys.size(); ++i) {
      if (old_stamp[i] == old_gen) {
        Insert(old_keys[i]);
      }
    }
  }

  std::vector<uint64_t> keys_;
  std::vector<uint32_t> stamp_;
  uint32_t gen_ = 1;
  size_t size_ = 0;
};

// One voxel hit by a quad sample, before de-duplication across rows.
struct Candidate
{
  int64_t ix, iy, iz;
  uint64_t key;
  Vector3d normal;
  double mu;
};

// Reusable buffers so a ping allocates nothing once warmed up.
struct Workspace
{
  std::vector<Vector3d> pts;
  std::vector<char> valid;
  KeySet seen;                               // global, merge step
  std::vector<KeySet> row_sets;              // one per OpenMP thread
  std::vector<std::vector<Candidate>> rows;  // candidates per quad row
  std::vector<const Candidate *> kept;       // merged, in serial-scan order
};

// Reconstructs the visible surface from `scan` (taken at `array_pose`, array frame in world)
// and quantises it into world-fixed voxels. Output order depends only on the scan. REQ-004/005/007
void BuildScatterers(
  const Scan & scan, const Pose3d & array_pose, const Params & p, Workspace & ws,
  std::vector<Scatterer> & out);

}  // namespace scatter
}  // namespace sas_gazebo

#endif  // SAS_GAZEBO__SCATTER_HPP_
