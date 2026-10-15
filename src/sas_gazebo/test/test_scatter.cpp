// Copyright 2026 The SAS Sensor Plugin Authors
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <cmath>
#include <complex>
#include <limits>
#include <algorithm>
#include <map>
#include <vector>

#include "sas_gazebo/scatter.hpp"

namespace sc = sas_gazebo::scatter;
using ignition::math::Pose3d;
using ignition::math::Vector3d;

namespace
{

constexpr double kGroundZ = -10.0;

// Optional solid box standing on the ground (top at z), to create a depth discontinuity.
struct Box
{
  bool on = false;
  double z = -7.0, x0 = 8.0, x1 = 11.0, y0 = -3.0, y1 = 3.0;
};

// Ray / axis-aligned box intersection (slab method). Returns entry distance or +inf.
double HitBox(const Vector3d & o, const Vector3d & d, const Vector3d & lo, const Vector3d & hi)
{
  double t0 = 0.0, t1 = std::numeric_limits<double>::infinity();
  for (int k = 0; k < 3; ++k) {
    if (std::abs(d[k]) < 1e-12) {
      if (o[k] < lo[k] || o[k] > hi[k]) {return std::numeric_limits<double>::infinity();}
      continue;
    }
    double ta = (lo[k] - o[k]) / d[k], tb = (hi[k] - o[k]) / d[k];
    if (ta > tb) {std::swap(ta, tb);}
    t0 = std::max(t0, ta);
    t1 = std::min(t1, tb);
  }
  return t0 <= t1 ? t0 : std::numeric_limits<double>::infinity();
}

// Analytic range image of a flat ground (+ optional box) seen from `pose`.
sc::Scan MakeScan(const Pose3d & pose, double retro, const Box & box = {}, int n = 64)
{
  sc::Scan s;
  s.nh = n;
  s.nv = n;
  s.hmin = -0.40;
  s.hres = 0.80 / (n - 1);
  s.vmin = -0.80;
  s.vres = 1.00 / (n - 1);
  s.rmin = 0.5;
  s.rmax = 100.0;
  s.ranges.assign(static_cast<size_t>(n) * n, std::numeric_limits<double>::infinity());
  s.retro.assign(static_cast<size_t>(n) * n, 0.0);
  const Vector3d o = pose.Pos();
  for (int j = 0; j < n; ++j) {
    const double el = s.vmin + j * s.vres;
    for (int i = 0; i < n; ++i) {
      const double az = s.hmin + i * s.hres;
      const Vector3d d = pose.Rot().RotateVector(
        Vector3d(std::cos(el) * std::cos(az), std::cos(el) * std::sin(az), std::sin(el)));
      if (d.Z() >= -1e-9) {
        continue;
      }
      double t = (kGroundZ - o.Z()) / d.Z();
      if (box.on) {
        t = std::min(t, HitBox(o, d, Vector3d(box.x0, box.y0, kGroundZ),
          Vector3d(box.x1, box.y1, box.z)));
      }
      const size_t idx = static_cast<size_t>(j) * n + i;
      s.ranges[idx] = t;
      s.retro[idx] = retro;
    }
  }
  return s;
}

// Array at the origin, boresight pitched 0.35 rad down (as in sas_smoke.world).
Pose3d DownLooking(double y = 0.0) {return Pose3d(0.0, y, 0.0, 0.0, 0.35, 0.0);}

sc::Params TestParams()
{
  sc::Params p;
  p.cell_size = 0.10;  // coarse cells keep the test fast
  p.min_range = 1.0;
  p.max_range = 60.0;
  return p;
}

std::vector<sc::Scatterer> Build(const sc::Scan & s, const Pose3d & pose, const sc::Params & p)
{
  sc::Workspace ws;
  std::vector<sc::Scatterer> out;
  sc::BuildScatterers(s, pose, p, ws, out);
  return out;
}

}  // namespace

TEST(Hash, CellKeyDistinctForNeighboursAndNegatives)
{
  std::map<uint64_t, int> keys;
  for (int ix = -2; ix <= 2; ++ix) {
    for (int iy = -2; iy <= 2; ++iy) {
      for (int iz = -2; iz <= 2; ++iz) {
        ++keys[sc::CellKey(ix, iy, iz)];
      }
    }
  }
  EXPECT_EQ(keys.size(), 125u);
}

TEST(Hash, VoxelScattererDeterministicAndBounded)
{
  sc::Scatterer a, b, c;
  sc::VoxelScatterer(-7, 3, -250, 0.04, 12345, a);
  sc::VoxelScatterer(-7, 3, -250, 0.04, 12345, b);
  sc::VoxelScatterer(-7, 3, -250, 0.04, 54321, c);
  EXPECT_EQ(a.pos, b.pos);
  EXPECT_EQ(a.speckle, b.speckle);
  EXPECT_NE(a.speckle, c.speckle);  // seed changes the field
  const Vector3d centre(-7 * 0.04, 3 * 0.04, -250 * 0.04);
  EXPECT_LE(std::abs(a.pos.X() - centre.X()), 0.4 * 0.04 + 1e-12);
  EXPECT_LE(std::abs(a.pos.Y() - centre.Y()), 0.4 * 0.04 + 1e-12);
  EXPECT_LE(std::abs(a.pos.Z() - centre.Z()), 0.4 * 0.04 + 1e-12);
}

TEST(Hash, SpeckleIsUnitVarianceZeroMean)
{
  const int n = 200000;
  std::complex<double> mean = 0.0;
  double power = 0.0;
  sc::Scatterer s;
  for (int i = 0; i < n; ++i) {
    sc::VoxelScatterer(i, -i / 3, 17, 0.04, 99, s);
    mean += s.speckle;
    power += std::norm(s.speckle);
  }
  EXPECT_NEAR(std::abs(mean / static_cast<double>(n)), 0.0, 0.01);
  EXPECT_NEAR(power / n, 1.0, 0.02);
}

TEST(Build, FlatGroundLiesOnPlane)  // REQ-004
{
  const auto p = TestParams();
  const auto out = Build(MakeScan(DownLooking(), 0.0), DownLooking(), p);
  ASSERT_GT(out.size(), 1000u);
  for (const auto & s : out) {
    EXPECT_NEAR(s.pos.Z(), kGroundZ, p.cell_size);
    EXPECT_GT(s.normal.Z(), 0.99);  // up-facing, toward the sonar
  }
}

// Regression: with floor() voxels, ground at z = -10 (a multiple of cell) straddled two voxel
// layers and voxels flickered between pings. Found in the plan 004 smoke run.
TEST(Build, BoundaryAlignedSurfaceIsOneLayer)  // REQ-005
{
  const auto p = TestParams();
  for (double y : {0.0, 0.05, 0.15}) {
    const auto out = Build(MakeScan(DownLooking(y), 0.0), DownLooking(y), p);
    for (const auto & s : out) {
      ASSERT_EQ(sc::CellKeyZ(s.key), static_cast<int64_t>(std::llround(kGroundZ / p.cell_size)));
    }
  }
}

TEST(Build, RevisitedVoxelsAreIdentical)  // REQ-005
{
  const auto p = TestParams();
  const auto a = Build(MakeScan(DownLooking(0.0), 0.0), DownLooking(0.0), p);
  const auto b = Build(MakeScan(DownLooking(0.15), 0.0), DownLooking(0.15), p);
  std::map<uint64_t, const sc::Scatterer *> first;
  for (const auto & s : a) {first[s.key] = &s;}
  size_t common = 0;
  for (const auto & s : b) {
    auto it = first.find(s.key);
    if (it == first.end()) {continue;}
    ++common;
    EXPECT_EQ(s.pos, it->second->pos);
    EXPECT_EQ(s.speckle, it->second->speckle);
  }
  EXPECT_GT(common, a.size() / 2);  // most of the footprint is seen again
}

TEST(Build, DepthDiscontinuityLeavesShadow)  // REQ-004
{
  // The box's front face faces the sonar and is kept. Behind the top far
  // edge the range jumps from the box to the ground: that quad is grazing and is rejected, so
  // the ground the box hides has no scatterers.
  const auto p = TestParams();
  Box box;
  box.on = true;
  // 256 rays (~0.004 rad step). At 64 rays the gap quad is only ~87.7 deg from the view ray and
  // passes the 88 deg limit: shadow detection needs R * dtheta / dR < cos(max_incidence).
  const auto out = Build(MakeScan(DownLooking(), 0.0, box, 256), DownLooking(), p);
  // Ray through the far edge (x1, box.z) meets the ground at x1 * kGroundZ / box.z (y ~ 0).
  const double shadow_end = box.x1 * kGroundZ / box.z;
  size_t on_box = 0, ground_beyond = 0;
  for (const auto & s : out) {
    const bool central = std::abs(s.pos.Y()) < 1.0;
    const bool in_gap = s.pos.Z() > kGroundZ + 0.5 && s.pos.Z() < box.z - 0.5;
    EXPECT_FALSE(central && in_gap && s.pos.X() > box.x1 + 0.2) << "curtain behind box " << s.pos;
    const bool on_ground = std::abs(s.pos.Z() - kGroundZ) < p.cell_size;
    EXPECT_FALSE(central && on_ground && s.pos.X() > box.x1 + 0.3 &&
      s.pos.X() < shadow_end - 0.3) << "ground inside shadow " << s.pos;
    if (std::abs(s.pos.Z() - box.z) < p.cell_size) {++on_box;}
    if (central && on_ground && s.pos.X() > shadow_end + 0.5) {++ground_beyond;}
  }
  EXPECT_GT(on_box, 100u);
  EXPECT_GT(ground_beyond, 100u);  // ground visible again past the shadow
}

TEST(Build, RetroSetsMuOtherwiseDefault)  // REQ-007
{
  const auto p = TestParams();
  for (const auto & s : Build(MakeScan(DownLooking(), 0.0), DownLooking(), p)) {
    ASSERT_DOUBLE_EQ(s.mu, p.mu_default);
  }
  for (const auto & s : Build(MakeScan(DownLooking(), 0.3), DownLooking(), p)) {
    ASSERT_DOUBLE_EQ(s.mu, 0.3);
  }
}

TEST(Build, RangeGateApplied)
{
  auto p = TestParams();
  p.max_range = 13.0;
  const auto out = Build(MakeScan(DownLooking(), 0.0), DownLooking(), p);
  ASSERT_FALSE(out.empty());
  for (const auto & s : out) {
    EXPECT_LT(s.pos.Length(), 13.0 + 2.0 * p.cell_size);
  }
}

TEST(Build, SameInputSameOutputOrder)  // REQ-012
{
  const auto p = TestParams();
  const auto s = MakeScan(DownLooking(), 0.0);
  const auto a = Build(s, DownLooking(), p);
  const auto b = Build(s, DownLooking(), p);
  ASSERT_EQ(a.size(), b.size());
  for (size_t i = 0; i < a.size(); ++i) {
    ASSERT_EQ(a[i].key, b[i].key);
    ASSERT_EQ(a[i].pos, b[i].pos);
    ASSERT_EQ(a[i].speckle, b[i].speckle);
  }
}
