// cluster.hpp — greedy O(N²) spatial clustering with running-mean centroid.
//
// Each new point joins the first existing cluster whose centroid is within
// `cluster_radius_m`, updating the centroid as a running mean; else it
// seeds a new cluster.  The representative emitted per cluster is the
// MEDOID (raw member nearest the centroid), not the centroid itself —
// raw members have already passed the obstacle-clearance check upstream,
// while a synthesised centroid could land closer to an obstacle than any
// sampled member.

#pragma once

namespace cfpa2 {
namespace ops {

/// Returns the number of representatives written to out_x / out_y (≤ n_in).
///
/// \param cluster_radius_m  ≤ 0 disables (returns inputs unchanged).
int cluster_representatives(
    const float * in_x, const float * in_y, int n_in,
    float cluster_radius_m,
    float * out_x, float * out_y);

}  // namespace ops
}  // namespace cfpa2
