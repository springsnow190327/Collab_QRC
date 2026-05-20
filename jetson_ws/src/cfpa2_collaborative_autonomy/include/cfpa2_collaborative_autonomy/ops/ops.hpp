// ops.hpp — aggregator header for all cfpa2::ops grid kernels.
//
// Future pure-C++ rclcpp nodes consume kernels through this single
// include. The kernels themselves live in individual headers so callers
// can pull just one in for unit tests.

#pragma once

#include "cfpa2_collaborative_autonomy/ops/cluster.hpp"
#include "cfpa2_collaborative_autonomy/ops/dead_frontier_filter.hpp"
#include "cfpa2_collaborative_autonomy/ops/distance_transform.hpp"
#include "cfpa2_collaborative_autonomy/ops/frontier_extract.hpp"
#include "cfpa2_collaborative_autonomy/ops/grid_offsets.hpp"
#include "cfpa2_collaborative_autonomy/ops/info_gain.hpp"
