// dead_frontier_filter.hpp — drop frontier candidates whose surrounding
// has too few "live" unknown cells.
//
// A LIVE unknown is one whose 8-neighbour kernel has NO occupied cell
// (i.e. it sits ≥ 1 cell away from any wall). Trap unknowns sitting right
// outside an arena boundary fail this test (one of their 8-neighbours is
// the wall), keeping the robot from chasing forever-invisible frontier
// cells (e.g. cells just beyond a closed arena wall).

#pragma once

#include <cstdint>

namespace cfpa2 {
namespace ops {

/// Returns the number of survivors written to out_x / out_y (≤ n_in).
///
/// \param min_live_unknowns  >= 1 to activate; 0 disables (pass-through).
int filter_dead_frontiers(
    const int8_t * grid,
    int W, int H,
    float res, float origin_x, float origin_y,
    int8_t occ_threshold,
    int radius_cells,
    int min_live_unknowns,
    const float * in_x, const float * in_y, int n_in,
    float * out_x, float * out_y);

}  // namespace ops
}  // namespace cfpa2
