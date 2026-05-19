// cfpa2_grid_ops_c_api.cpp — extern "C" wrappers for ctypes consumers.
//
// The native C++ kernels live in cfpa2::ops::*. These wrappers preserve
// the ABI exposed by the pre-Phase-A cfpa2_grid_ops.so so the existing
// Python coordinator's ctypes binding keeps working unmodified during
// the transition to a pure-C++ rclcpp Node.

#include <cstdint>

#include "cfpa2_collaborative_autonomy/ops/ops.hpp"

extern "C" {

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
  return cfpa2::ops::extract_frontiers(
      grid, W, H, res, origin_x, origin_y,
      stride, min_cluster_area, clearance_cells,
      free_val, unknown_val, occ_threshold,
      out_x, out_y, max_out);
}

void distance_transform(
    const int8_t * grid,
    int W, int H,
    int sx, int sy,
    int8_t free_val,
    int * dist_out)
{
  cfpa2::ops::distance_transform(grid, W, H, sx, sy, free_val, dist_out);
}

void distance_transform_range(
    const int8_t * grid,
    int W, int H,
    int sx, int sy,
    int8_t unknown_val, int8_t occ_threshold,
    int * dist_out)
{
  cfpa2::ops::distance_transform_range(
      grid, W, H, sx, sy, unknown_val, occ_threshold, dist_out);
}

void batch_info_gain(
    const int8_t * grid,
    int W, int H,
    float res, float origin_x, float origin_y,
    const float * goal_x, const float * goal_y, int n_goals,
    int radius,
    int8_t unknown_val,
    float * gains_out)
{
  cfpa2::ops::batch_info_gain(
      grid, W, H, res, origin_x, origin_y,
      goal_x, goal_y, n_goals,
      radius, unknown_val, gains_out);
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
  cfpa2::ops::batch_info_gain_floodfill(
      grid, W, H, res, origin_x, origin_y,
      goal_x, goal_y, n_goals,
      budget, max_radius_cells,
      unknown_val, visited_scratch, gains_out);
}

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
  return cfpa2::ops::filter_dead_frontiers(
      grid, W, H, res, origin_x, origin_y,
      occ_threshold, radius_cells, min_live_unknowns,
      in_x, in_y, n_in, out_x, out_y);
}

int cluster_representatives(
    const float * in_x, const float * in_y, int n_in,
    float cluster_radius_m,
    float * out_x, float * out_y)
{
  return cfpa2::ops::cluster_representatives(
      in_x, in_y, n_in, cluster_radius_m, out_x, out_y);
}

}  // extern "C"
