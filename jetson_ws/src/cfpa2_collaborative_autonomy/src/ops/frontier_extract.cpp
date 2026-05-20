// frontier_extract.cpp — see frontier_extract.hpp for the API contract.
//
// Implementation lifted unchanged from the pre-Phase-A
// cfpa2_grid_ops.cpp::extract_frontiers — kernel preserved verbatim, just
// hoisted into the cfpa2::ops namespace.

#include "cfpa2_collaborative_autonomy/ops/frontier_extract.hpp"

#include <queue>
#include <vector>

#include "cfpa2_collaborative_autonomy/ops/grid_offsets.hpp"

namespace cfpa2 {
namespace ops {

int extract_frontiers(
    const int8_t * grid,
    int W, int H,
    float res, float origin_x, float origin_y,
    int stride,
    float min_cluster_area,
    int clearance_cells,
    int8_t free_val, int8_t unknown_val, int8_t occ_threshold,
    float * out_x, float * out_y,
    int max_out)
{
  (void)free_val;  // kept for ABI symmetry with the legacy ctypes signature

  if (!grid || W <= 2 || H <= 2 || max_out <= 0) return 0;

  const int N = W * H;
  std::vector<uint8_t> frontier_mask(N, 0);
  std::vector<int> frontier_indices;
  frontier_indices.reserve(N / 10);

  // Step 1: identify frontier cells (free cells 8-conn adjacent to unknown).
  for (int gy = 1; gy < H - 1; ++gy) {
    const int row = gy * W;
    for (int gx = 1; gx < W - 1; ++gx) {
      const int idx = row + gx;
      // Treat any non-unknown cell below occ_threshold as free.
      const int8_t v = grid[idx];
      if (v == unknown_val || v < 0 || v >= occ_threshold) continue;

      bool found_unknown = false;
      for (int d = 0; d < 8; ++d) {
        const int ni = (gy + DY8[d]) * W + (gx + DX8[d]);
        if (grid[ni] == unknown_val) {
          found_unknown = true;
          break;
        }
      }
      if (!found_unknown) continue;

      frontier_mask[idx] = 1;
      frontier_indices.push_back(idx);
    }
  }

  if (frontier_indices.empty()) return 0;

  // Step 2: BFS clustering + filtered/strided output.
  std::vector<uint8_t> visited(N, 0);
  std::queue<int> q;
  std::vector<int> component;
  int out_count = 0;
  const float cell_area = res * res;

  for (int seed_idx : frontier_indices) {
    if (visited[seed_idx] || !frontier_mask[seed_idx]) continue;

    visited[seed_idx] = 1;
    q.push(seed_idx);
    component.clear();

    while (!q.empty()) {
      const int ci = q.front();
      q.pop();
      component.push_back(ci);

      const int cx = ci % W;
      const int cy = ci / W;
      for (int d = 0; d < 8; ++d) {
        const int nx = cx + DX8[d];
        const int ny = cy + DY8[d];
        if (nx <= 0 || ny <= 0 || nx >= W - 1 || ny >= H - 1) continue;
        const int ni = ny * W + nx;
        if (visited[ni] || !frontier_mask[ni]) continue;
        visited[ni] = 1;
        q.push(ni);
      }
    }

    // Filter by cluster area.
    const float cluster_area = static_cast<float>(component.size()) * cell_area;
    if (cluster_area + 1e-9f < min_cluster_area) continue;

    // Output stride-subsampled members with clearance check.
    for (int i = 0; i < static_cast<int>(component.size()); ++i) {
      if ((i % stride) != 0) continue;

      const int ci = component[i];
      const int gx = ci % W;
      const int gy = ci / W;

      if (clearance_cells > 0) {
        bool too_close = false;
        for (int dy = -clearance_cells; dy <= clearance_cells && !too_close; ++dy) {
          const int ny = gy + dy;
          if (ny < 0 || ny >= H) continue;
          for (int dx = -clearance_cells; dx <= clearance_cells; ++dx) {
            const int nx = gx + dx;
            if (nx < 0 || nx >= W) continue;
            if (grid[ny * W + nx] >= occ_threshold) {
              too_close = true;
              break;
            }
          }
        }
        if (too_close) continue;
      }

      out_x[out_count] = origin_x + (static_cast<float>(gx) + 0.5f) * res;
      out_y[out_count] = origin_y + (static_cast<float>(gy) + 0.5f) * res;
      ++out_count;

      if (out_count >= max_out) return out_count;
    }
  }

  return out_count;
}

}  // namespace ops
}  // namespace cfpa2
