// frontier_extract.hpp — 2D frontier extraction on a row-major OccupancyGrid.
//
// A frontier cell is FREE (cell value in [0, occ_threshold)) and 8-conn
// adjacent to at least one UNKNOWN cell (cell value == unknown_val).
// Detected frontier cells are 8-conn BFS-clustered, clusters smaller than
// `min_cluster_area` (in m²) are dropped, and the surviving members are
// stride-subsampled with a square clearance check against any cell
// >= occ_threshold within `clearance_cells`.
//
// Behaviour matches the original ctypes function in cfpa2_grid_ops.cpp
// pre-Phase-A — the kernel is preserved verbatim across the refactor.

#pragma once

#include <cstdint>

namespace cfpa2 {
namespace ops {

/// Run frontier extraction.
///
/// \param grid           row-major OccupancyGrid data, length >= W*H.
/// \param W, H           grid width / height in cells.
/// \param res            cell size (m); must be > 0.
/// \param origin_x, _y   world coords of grid cell (0, 0) lower-left corner.
/// \param stride         subsampling stride for the per-cluster output.
/// \param min_cluster_area  drop clusters with area below this (m²).
/// \param clearance_cells   reject points within this many cells of an
///                          occupied (>= occ_threshold) cell. 0 disables.
/// \param free_val       grid value that means "free" (reference only;
///                       traversability is decided by occ_threshold instead).
/// \param unknown_val    grid value that means "unknown" (exact match).
/// \param occ_threshold  grid values >= this are "occupied".
/// \param out_x, out_y   caller-owned buffers of length max_out.
/// \param max_out        capacity of the output buffers.
/// \returns number of points written to out_x / out_y (≤ max_out).
int extract_frontiers(
    const int8_t * grid,
    int W, int H,
    float res, float origin_x, float origin_y,
    int stride,
    float min_cluster_area,
    int clearance_cells,
    int8_t free_val, int8_t unknown_val, int8_t occ_threshold,
    float * out_x, float * out_y,
    int max_out);

}  // namespace ops
}  // namespace cfpa2
