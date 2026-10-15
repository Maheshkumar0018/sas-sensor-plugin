// Copyright 2026 The SAS Sensor Plugin Authors
// SPDX-License-Identifier: Apache-2.0

#include "sas_gazebo/scatter.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace sas_gazebo
{
namespace scatter
{

void BuildScatterers(
  const Scan & s, const Pose3d & array_pose, const Params & p, Workspace & ws,
  std::vector<Scatterer> & out)
{
  out.clear();
  const int nh = s.nh;
  const int nv = s.nv;
  if (nh < 2 || nv < 2) {
    return;
  }
  const size_t n_rays = static_cast<size_t>(nh) * static_cast<size_t>(nv);

  // 1. Ray hits -> world points. Rays outside the range gate are invalid.  REQ-004
  ws.pts.assign(n_rays, Vector3d::Zero);
  ws.valid.assign(n_rays, 0);
  const double r_hi = std::min(s.rmax * 0.999, p.max_range);
  const double r_lo = std::max(s.rmin, p.min_range);
  for (int j = 0; j < nv; ++j) {
    const double el = s.vmin + j * s.vres;
    for (int i = 0; i < nh; ++i) {
      const size_t idx = static_cast<size_t>(j) * nh + i;
      const double r = s.ranges[idx];
      if (!std::isfinite(r) || r >= r_hi || r <= r_lo) {
        continue;
      }
      const double az = s.hmin + i * s.hres;
      const Vector3d p_local(
        r * std::cos(el) * std::cos(az), r * std::cos(el) * std::sin(az), r * std::sin(el));
      ws.pts[idx] = array_pose.CoordPositionAdd(p_local);
      ws.valid[idx] = 1;
    }
  }

  ws.seen.clear();
  const Vector3d origin = array_pose.Pos();
  const double step = 0.5 * p.cell_size;
  const double cos_max = std::cos(p.max_incidence);

  // 2. Quads on the range-image grid.
  for (int j = 0; j < nv - 1; ++j) {
    for (int i = 0; i < nh - 1; ++i) {
      const size_t a = static_cast<size_t>(j) * nh + i;
      const size_t b = a + 1;
      const size_t c = a + nh;
      const size_t d = c + 1;
      if (!(ws.valid[a] && ws.valid[b] && ws.valid[c] && ws.valid[d])) {
        continue;
      }
      const Vector3d & p00 = ws.pts[a];
      const Vector3d & p10 = ws.pts[b];
      const Vector3d & p01 = ws.pts[c];
      const Vector3d & p11 = ws.pts[d];

      Vector3d n = (p11 - p00).Cross(p01 - p10);
      const double n_len = n.Length();
      if (n_len < 1e-12) {
        continue;
      }
      n /= n_len;
      const Vector3d centroid = 0.25 * (p00 + p10 + p01 + p11);
      const Vector3d view = (origin - centroid).Normalized();
      if (n.Dot(view) < 0.0) {
        n = -n;
      }
      // 3. Quad almost parallel to the ray = grazing face or depth discontinuity.
      //    Rejecting it leaves the region behind an object empty: acoustic shadow.
      if (n.Dot(view) < cos_max) {
        continue;
      }

      // REQ-007: per-surface strength from <laser_retro>, default when zero.
      const double retro = 0.25 * (s.retro[a] + s.retro[b] + s.retro[c] + s.retro[d]);
      const double mu = retro > 0.0 ? retro : p.mu_default;

      // 4. Subdivide at cell/2 so every voxel the quad crosses is sampled.
      const int nu = std::clamp(
        static_cast<int>(std::ceil(std::max(p00.Distance(p10), p01.Distance(p11)) / step)),
        1, 512);
      const int nw = std::clamp(
        static_cast<int>(std::ceil(std::max(p00.Distance(p01), p10.Distance(p11)) / step)),
        1, 512);

      for (int u = 0; u < nu; ++u) {
        const double fu = (u + 0.5) / nu;
        for (int w = 0; w < nw; ++w) {
          const double fw = (w + 0.5) / nw;
          const Vector3d q = (1 - fu) * (1 - fw) * p00 + fu * (1 - fw) * p10 +
            (1 - fu) * fw * p01 + fu * fw * p11;
          const int64_t ix = VoxelIndex(q.X(), p.cell_size);
          const int64_t iy = VoxelIndex(q.Y(), p.cell_size);
          const int64_t iz = VoxelIndex(q.Z(), p.cell_size);
          if (!ws.seen.insert(CellKey(ix, iy, iz)).second) {
            continue;  // one scatterer per voxel per ping
          }
          // 5. Position and speckle from the voxel hash only (REQ-005, ADR-0004).
          Scatterer sc;
          VoxelScatterer(ix, iy, iz, p.cell_size, p.seed, sc);
          sc.normal = n;
          sc.mu = mu;
          out.push_back(sc);
        }
      }
    }
  }
}

}  // namespace scatter
}  // namespace sas_gazebo
