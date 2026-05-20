// info_gain.cpp — see info_gain.hpp for API contract.
// Implementation lifted unchanged from pre-Phase-A cfpa2_grid_ops.cpp.

#include "cfpa2_collaborative_autonomy/ops/info_gain.hpp"

#include <algorithm>
#include <cstring>
#include <vector>

#include "cfpa2_collaborative_autonomy/ops/grid_offsets.hpp"

namespace cfpa2 {
namespace ops {

void batch_info_gain(
    const int8_t * grid,
    int W, int H,
    float res, float origin_x, float origin_y,
    const float * goal_x, const float * goal_y, int n_goals,
    int radius,
    int8_t unknown_val,
    float * gains_out)
{
  for (int i = 0; i < n_goals; ++i) {
    // World → grid.
    const int gx = static_cast<int>((goal_x[i] - origin_x) / res);
    const int gy = static_cast<int>((goal_y[i] - origin_y) / res);

    float gain = 0.0f;
    const int y0 = std::max(0, gy - radius);
    const int y1 = std::min(H, gy + radius + 1);
    const int x0 = std::max(0, gx - radius);
    const int x1 = std::min(W, gx + radius + 1);

    for (int yy = y0; yy < y1; ++yy) {
      const int row = yy * W;
      for (int xx = x0; xx < x1; ++xx) {
        if (grid[row + xx] == unknown_val) {
          gain += 1.0f;
        }
      }
    }
    gains_out[i] = gain;
  }
}

void batch_info_gain_floodfill(
    const int8_t * grid,
    int W, int H,
    float res, float origin_x, float origin_y,
    const float * goal_x, const float * goal_y, int n_goals,
    int budget, int max_radius_cells,
    int8_t unknown_val,
    int32_t * visited_scratch,
    float * gains_out)
{
  const int N = W * H;
  std::vector<int> q;
  q.reserve(budget + 4);

  // Generation counter so we can skip the per-goal zeroing of the
  // visited buffer. A cell counts as "visited for goal gi" iff its stored
  // tag equals gen_counter at the time we look at it. Wraparound (which
  // never realistically happens in a single process lifetime) triggers a
  // single full reset.
  static int32_t gen_counter = 0;

  for (int gi = 0; gi < n_goals; ++gi) {
    ++gen_counter;
    if (gen_counter <= 0) {
      std::memset(visited_scratch, 0, sizeof(int32_t) * N);
      gen_counter = 1;
    }

    const int sx = static_cast<int>((goal_x[gi] - origin_x) / res);
    const int sy = static_cast<int>((goal_y[gi] - origin_y) / res);
    if (sx < 0 || sy < 0 || sx >= W || sy >= H) {
      gains_out[gi] = 0.0f;
      continue;
    }

    const int seed_idx = sy * W + sx;
    visited_scratch[seed_idx] = gen_counter;
    q.clear();
    q.push_back(seed_idx);

    // Seed counts only if itself unknown (matches Python).
    int gain = (grid[seed_idx] == unknown_val) ? 1 : 0;
    std::size_t head = 0;

    while (head < q.size() && gain < budget) {
      const int ci = q[head++];
      const int cx = ci % W;
      const int cy = ci / W;

      for (int d = 0; d < 4; ++d) {
        const int nx = cx + DX4[d];
        const int ny = cy + DY4[d];
        if (nx < 0 || ny < 0 || nx >= W || ny >= H) continue;

        // Chebyshev radius cap on offset from start (sx, sy).
        const int adx = nx - sx;
        const int ady = ny - sy;
        const int abx = adx < 0 ? -adx : adx;
        const int aby = ady < 0 ? -ady : ady;
        const int chev = abx > aby ? abx : aby;
        if (chev > max_radius_cells) continue;

        const int nidx = ny * W + nx;
        if (visited_scratch[nidx] == gen_counter) continue;
        visited_scratch[nidx] = gen_counter;

        const int8_t v = grid[nidx];
        if (v != unknown_val) {
          // Only expand THROUGH unknown territory; free cells stop the
          // wave (otherwise it would flood the entire free map).
          continue;
        }
        ++gain;
        if (gain >= budget) break;
        q.push_back(nidx);
      }
    }
    gains_out[gi] = static_cast<float>(gain);
  }
}

}  // namespace ops
}  // namespace cfpa2
