// distance_transform.cpp — see distance_transform.hpp for API contract.
// Implementation lifted unchanged from pre-Phase-A cfpa2_grid_ops.cpp.

#include "cfpa2_collaborative_autonomy/ops/distance_transform.hpp"

#include <cstring>
#include <queue>

#include "cfpa2_collaborative_autonomy/ops/grid_offsets.hpp"

namespace cfpa2 {
namespace ops {

void distance_transform_range(
    const int8_t * grid,
    int W, int H,
    int sx, int sy,
    int8_t unknown_val, int8_t occ_threshold,
    int * dist_out)
{
  const int N = W * H;
  std::memset(dist_out, 0xFF, N * sizeof(int));  // -1 = unreachable

  if (sx < 0 || sx >= W || sy < 0 || sy >= H) return;

  auto is_free = [&](int idx) -> bool {
    const int8_t v = grid[idx];
    return v != unknown_val && v >= 0 && v < occ_threshold;
  };

  int sidx = sy * W + sx;

  // If start is not free, search nearby for a free cell (snap-in fallback).
  if (!is_free(sidx)) {
    bool found = false;
    for (int r = 1; r <= 12 && !found; ++r) {
      for (int dy = -r; dy <= r && !found; ++dy) {
        const int ny = sy + dy;
        if (ny < 0 || ny >= H) continue;
        for (int dx = -r; dx <= r; ++dx) {
          const int nx = sx + dx;
          if (nx < 0 || nx >= W) continue;
          const int ni = ny * W + nx;
          if (is_free(ni)) {
            sx = nx;
            sy = ny;
            sidx = ni;
            found = true;
            break;
          }
        }
      }
    }
    if (!found) return;
  }

  std::queue<int> q;
  dist_out[sidx] = 0;
  q.push(sidx);

  while (!q.empty()) {
    const int ci = q.front();
    q.pop();
    const int cx = ci % W;
    const int cy = ci / W;

    for (int d = 0; d < 4; ++d) {
      const int nx = cx + DX4[d];
      const int ny = cy + DY4[d];
      if (nx < 0 || ny < 0 || nx >= W || ny >= H) continue;
      const int ni = ny * W + nx;
      if (dist_out[ni] != -1) continue;
      if (!is_free(ni)) continue;
      dist_out[ni] = dist_out[ci] + 1;
      q.push(ni);
    }
  }
}

void distance_transform(
    const int8_t * grid,
    int W, int H,
    int sx, int sy,
    int8_t free_val,
    int * dist_out)
{
  (void)free_val;  // kept for ABI symmetry with the legacy ctypes signature
  // Treat any cell with 0 <= v < 50 as traversable (matches frontier
  // extraction logic in the original ctypes-loaded library).
  distance_transform_range(
      grid, W, H, sx, sy,
      /*unknown_val=*/ -1, /*occ_threshold=*/ 50, dist_out);
}

}  // namespace ops
}  // namespace cfpa2
