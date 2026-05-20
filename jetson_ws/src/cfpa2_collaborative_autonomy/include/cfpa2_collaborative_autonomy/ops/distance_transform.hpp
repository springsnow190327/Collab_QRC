// distance_transform.hpp — 4-conn BFS shortest-path distance from a seed
// over an OccupancyGrid, with the seed snap-to-nearest-free fallback.
//
// `distance_transform` (legacy) treats any cell with 0 ≤ v < 50 as
// traversable; `distance_transform_range` parameterises the threshold via
// `unknown_val` (cells == this are non-traversable) and `occ_threshold`
// (cells >= this are non-traversable). Both write -1 to unreachable cells.

#pragma once

#include <cstdint>

namespace cfpa2 {
namespace ops {

/// Legacy fixed-threshold BFS (unknown = -1, occ_threshold = 50).
/// Provided for binary compat with old callers; new code should call
/// `distance_transform_range` and pass the explicit thresholds.
void distance_transform(
    const int8_t * grid,
    int W, int H,
    int sx, int sy,
    int8_t free_val,
    int * dist_out);

/// Range-thresholded BFS.
///
/// \param grid           row-major OccupancyGrid data, length >= W*H.
/// \param W, H           grid width / height in cells.
/// \param sx, sy         seed cell. If non-traversable, snap to nearest
///                       free cell within a 12-cell radius. If still none,
///                       the entire output is left as -1.
/// \param unknown_val    cells with this value are non-traversable.
/// \param occ_threshold  cells with value >= this are non-traversable.
/// \param dist_out       caller-owned buffer of length W*H; -1 = unreachable.
void distance_transform_range(
    const int8_t * grid,
    int W, int H,
    int sx, int sy,
    int8_t unknown_val, int8_t occ_threshold,
    int * dist_out);

}  // namespace ops
}  // namespace cfpa2
