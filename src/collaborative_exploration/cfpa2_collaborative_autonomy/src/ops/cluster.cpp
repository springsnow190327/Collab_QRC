// cluster.cpp — see cluster.hpp for API contract.
// Implementation lifted unchanged from pre-Phase-A cfpa2_grid_ops.cpp.

#include "cfpa2_collaborative_autonomy/ops/cluster.hpp"

#include <cstddef>
#include <vector>

namespace cfpa2 {
namespace ops {

int cluster_representatives(
    const float * in_x, const float * in_y, int n_in,
    float cluster_radius_m,
    float * out_x, float * out_y)
{
  if (n_in <= 0) return 0;
  if (cluster_radius_m <= 0.0f) {
    for (int i = 0; i < n_in; ++i) {
      out_x[i] = in_x[i];
      out_y[i] = in_y[i];
    }
    return n_in;
  }
  const float r2 = cluster_radius_m * cluster_radius_m;

  // Per-cluster running centroid + member-index list.
  std::vector<float> cx;
  cx.reserve(64);
  std::vector<float> cy;
  cy.reserve(64);
  std::vector<std::vector<int>> members;
  members.reserve(64);

  for (int i = 0; i < n_in; ++i) {
    const float px = in_x[i];
    const float py = in_y[i];
    bool joined = false;
    for (std::size_t c = 0; c < cx.size(); ++c) {
      const float dx = px - cx[c];
      const float dy = py - cy[c];
      if (dx * dx + dy * dy <= r2) {
        members[c].push_back(i);
        // Update running mean centroid.
        double sx = 0.0;
        double sy = 0.0;
        for (int mi : members[c]) {
          sx += in_x[mi];
          sy += in_y[mi];
        }
        cx[c] = static_cast<float>(sx / members[c].size());
        cy[c] = static_cast<float>(sy / members[c].size());
        joined = true;
        break;
      }
    }
    if (!joined) {
      cx.push_back(px);
      cy.push_back(py);
      members.emplace_back();
      members.back().push_back(i);
    }
  }

  // Emit the medoid (raw member closest to centroid) per cluster.
  int n_out = 0;
  for (std::size_t c = 0; c < cx.size(); ++c) {
    float best_d2 = 0.0f;
    int best_mi = members[c][0];
    bool init = false;
    for (int mi : members[c]) {
      const float dx = in_x[mi] - cx[c];
      const float dy = in_y[mi] - cy[c];
      const float d2 = dx * dx + dy * dy;
      if (!init || d2 < best_d2) {
        best_d2 = d2;
        best_mi = mi;
        init = true;
      }
    }
    out_x[n_out] = in_x[best_mi];
    out_y[n_out] = in_y[best_mi];
    ++n_out;
  }
  return n_out;
}

}  // namespace ops
}  // namespace cfpa2
