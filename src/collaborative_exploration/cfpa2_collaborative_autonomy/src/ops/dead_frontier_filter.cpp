// dead_frontier_filter.cpp — see dead_frontier_filter.hpp for API contract.
// Implementation lifted unchanged from pre-Phase-A cfpa2_grid_ops.cpp.

#include "cfpa2_collaborative_autonomy/ops/dead_frontier_filter.hpp"

#include <algorithm>

namespace cfpa2 {
namespace ops {

int filter_dead_frontiers(
    const int8_t * grid,
    int W, int H,
    float res, float origin_x, float origin_y,
    int8_t occ_threshold,
    int radius_cells,
    int min_live_unknowns,
    const float * in_x, const float * in_y, int n_in,
    float * out_x, float * out_y)
{
  if (!grid || W <= 0 || H <= 0 || res <= 0.0f || n_in <= 0) return 0;
  if (min_live_unknowns <= 0) {
    // Filter disabled: pass-through.
    for (int i = 0; i < n_in; ++i) {
      out_x[i] = in_x[i];
      out_y[i] = in_y[i];
    }
    return n_in;
  }
  if (radius_cells < 1) radius_cells = 1;

  int n_out = 0;
  for (int i = 0; i < n_in; ++i) {
    const int gx = static_cast<int>((in_x[i] - origin_x) / res);
    const int gy = static_cast<int>((in_y[i] - origin_y) / res);

    const int y0 = std::max(1, gy - radius_cells);
    const int y1 = std::min(H - 2, gy + radius_cells);
    const int x0 = std::max(1, gx - radius_cells);
    const int x1 = std::min(W - 2, gx + radius_cells);

    int live_n = 0;
    bool done = false;
    for (int ny = y0; ny <= y1 && !done; ++ny) {
      for (int nx = x0; nx <= x1; ++nx) {
        // Live-unknown check (inlined for speed): cell is unknown, and
        // no 8-neighbour is occupied.
        const int idx = ny * W + nx;
        if (grid[idx] >= 0) continue;  // not unknown
        bool wall_touch = false;
        for (int ddy = -1; ddy <= 1 && !wall_touch; ++ddy) {
          const int yy = ny + ddy;
          if (yy < 0 || yy >= H) continue;
          for (int ddx = -1; ddx <= 1; ++ddx) {
            if (ddx == 0 && ddy == 0) continue;
            const int xx = nx + ddx;
            if (xx < 0 || xx >= W) continue;
            if (grid[yy * W + xx] >= occ_threshold) {
              wall_touch = true;
              break;
            }
          }
        }
        if (wall_touch) continue;
        ++live_n;
        if (live_n >= min_live_unknowns) {
          done = true;
          break;
        }
      }
    }
    if (live_n >= min_live_unknowns) {
      out_x[n_out] = in_x[i];
      out_y[n_out] = in_y[i];
      ++n_out;
    }
  }
  return n_out;
}

}  // namespace ops
}  // namespace cfpa2
