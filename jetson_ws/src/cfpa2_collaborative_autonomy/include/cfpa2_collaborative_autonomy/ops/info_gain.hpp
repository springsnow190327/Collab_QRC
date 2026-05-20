// info_gain.hpp — batched information-gain estimators on an OccupancyGrid.
//
// Two flavours:
//
//  - `batch_info_gain`           — local-window unknown-cell count in a
//    (2·radius+1)² square around each goal.  O(radius²) per goal.  Tends
//    to undervalue large rooms (the count saturates at the window area).
//
//  - `batch_info_gain_floodfill` — 4-conn BFS through UNKNOWN cells from
//    each goal, capped by a global cell budget and a Chebyshev radius.
//    Big rooms saturate the budget; small nooks stop at room boundary.
//    Caller passes a reusable scratch buffer of `int32_t[W*H]` to avoid
//    a per-call zero-fill; a generation counter inside the kernel takes
//    care of "freshness" across consecutive goals.

#pragma once

#include <cstdint>

namespace cfpa2 {
namespace ops {

/// Count unknown cells in a (2·radius + 1)² square around each goal.
void batch_info_gain(
    const int8_t * grid,
    int W, int H,
    float res, float origin_x, float origin_y,
    const float * goal_x, const float * goal_y, int n_goals,
    int radius,
    int8_t unknown_val,
    float * gains_out);

/// 4-conn BFS through UNKNOWN cells from each goal, capped by `budget` and
/// `max_radius_cells` (Chebyshev from start). Caller owns a
/// length-W*H int32 scratch buffer that the kernel reuses across calls via
/// an internal generation counter.
void batch_info_gain_floodfill(
    const int8_t * grid,
    int W, int H,
    float res, float origin_x, float origin_y,
    const float * goal_x, const float * goal_y, int n_goals,
    int budget, int max_radius_cells,
    int8_t unknown_val,
    int32_t * visited_scratch,
    float * gains_out);

}  // namespace ops
}  // namespace cfpa2
