// Copyright 2026 The SAS Sensor Plugin Authors
// SPDX-License-Identifier: Apache-2.0

#include "sas_gazebo/scatter.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include <omp.h>

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
  #pragma omp parallel for schedule(static)
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

  const Vector3d origin = array_pose.Pos();
  const double step = 0.5 * p.cell_size;
  const double cos_max = std::cos(p.max_incidence);

  // Parallel and deterministic (plan 009): each quad row lists its voxel candidates, keeping only
  // the first occurrence within the row; rows are then merged in order against one global set, so
  // the kept voxels and their order are exactly those of a serial scan; finally the per-voxel
  // hash work runs in parallel on the kept list.
  const int n_rows = nv - 1;
  ws.rows.resize(static_cast<size_t>(n_rows));
  ws.row_sets.resize(static_cast<size_t>(omp_get_max_threads()));

  // 2. Quads on the range-image grid.
  #pragma omp parallel for schedule(dynamic, 4)
  for (int j = 0; j < n_rows; ++j) {
    auto & row = ws.rows[static_cast<size_t>(j)];
    row.clear();
    KeySet & row_seen = ws.row_sets[static_cast<size_t>(omp_get_thread_num())];
    row_seen.Clear();
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
          Candidate cand;
          cand.ix = VoxelIndex(q.X(), p.cell_size);
          cand.iy = VoxelIndex(q.Y(), p.cell_size);
          cand.iz = VoxelIndex(q.Z(), p.cell_size);
          cand.key = CellKey(cand.ix, cand.iy, cand.iz);
          if (!row_seen.Insert(cand.key)) {
            continue;
          }
          cand.normal = n;
          cand.mu = mu;
          row.push_back(cand);
        }
      }
    }
  }

  // Merge rows in order: one scatterer per voxel per ping, first occurrence wins.
  ws.seen.Clear();
  ws.kept.clear();
  for (const auto & row : ws.rows) {
    for (const auto & cand : row) {
      if (ws.seen.Insert(cand.key)) {
        ws.kept.push_back(&cand);
      }
    }
  }

  // 5. Position and speckle from the voxel hash only (REQ-005, ADR-0004).
  out.resize(ws.kept.size());
  #pragma omp parallel for schedule(static)
  for (int64_t k = 0; k < static_cast<int64_t>(ws.kept.size()); ++k) {
    const Candidate & cand = *ws.kept[static_cast<size_t>(k)];
    Scatterer & sc = out[static_cast<size_t>(k)];
    VoxelScatterer(cand.ix, cand.iy, cand.iz, p.cell_size, p.seed, sc);
    sc.normal = cand.normal;
    sc.mu = cand.mu;
  }
}

}  // namespace scatter
}  // namespace sas_gazebo
