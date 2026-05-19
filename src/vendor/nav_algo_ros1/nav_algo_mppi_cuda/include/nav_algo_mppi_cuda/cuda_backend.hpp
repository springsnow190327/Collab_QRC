// Copyright 2026 Collab_QRC
// SPDX-License-Identifier: Apache-2.0
//
// Concrete GPU implementation of nav_algo_core::mppi::ICudaBackend.
//
// Lifecycle hygiene (2026-05-19 v2):
//   - All device buffers + the cudaStream wrapped in RAII smart pointers
//     (see device_memory.hpp). Ctor-half-failure is safe: any partially
//     allocated members get freed by their dtors as the exception unwinds.
//   - Per-instance cudaStream so multiple concurrent CudaBackends (e.g.
//     dual-robot deployments) don't serialise through the default stream.
//   - Pinned host staging buffers for the largest H2D transfers (state +
//     costmap) — true async DMA, no pageable-memory sync fallback.
//   - Sticky-cuda-error reset after every kernel launch keeps a single
//     failure from poisoning subsequent CUDA API calls in the process.
//   - Probe files gated by NAV_ALGO_CUDA_PROBE (off by default).

#ifndef NAV_ALGO_MPPI_CUDA__CUDA_BACKEND_HPP_
#define NAV_ALGO_MPPI_CUDA__CUDA_BACKEND_HPP_

#include <cstdint>
#include <vector>

#include "nav_algo_core/mppi/cuda_backend.hpp"
#include "nav_algo_core/mppi/tools/parameters_handler.hpp"
#include "nav_algo_mppi_cuda/critics.cuh"
#include "nav_algo_mppi_cuda/integrate.cuh"
#include "nav_algo_mppi_cuda/control_update.cuh"
#include "nav_algo_mppi_cuda/device_memory.hpp"

#include <string>

namespace mppi { class Optimizer; }

namespace nav_algo_mppi_cuda
{

struct CudaBackendConfig
{
  unsigned int batch_size;          // B (Nav2 canonical 2000)
  unsigned int time_steps;          // T (canonical 56)
  unsigned int path_max_points;     // upper bound on P; 256 covers Nav2 prune
  unsigned int costmap_max_cells;   // size_x × size_y upper bound
  unsigned int footprint_max_n;     // typically 4 for our Go2W rectangle
};

// Per-critic yaml-loaded parameters. loadCriticParams() reads these from
// rosparam (same keys + defaults as nav_algo_core's CriticManager loads
// for the CPU path); each `optimize()` call uses them to (1) launch each
// kernel with the right cost_power / cost_weight, and (2) check host-side
// gates that the CPU critics check before running.
struct CriticParams
{
  // ConstraintCritic (no gate; always runs when enabled)
  int   constraint_power           = 1;
  float constraint_weight          = 4.0f;

  // ObstaclesCritic (no gate; near_goal flag is set from
  // within_position_goal_tolerance(near_goal_distance))
  bool  obs_consider_footprint     = false;
  int   obs_power                  = 1;
  float obs_repulsion_weight       = 1.5f;
  float obs_critical_weight        = 20.0f;
  float obs_collision_cost         = 10000.0f;
  float obs_collision_margin       = 0.10f;
  float obs_near_goal_distance     = 0.5f;
  float obs_inflation_scale_factor = 10.0f;
  float obs_inflation_radius       = 0.55f;
  float obs_circumscribed_radius   = 0.20f;   // not yaml-loaded; from costmap_ros
  float obs_possibly_inscribed_cost = 100.0f; // not yaml-loaded; computed in CPU
  bool  obs_tracking_unknown       = true;

  // GoalCritic (gate: within_position_goal_tolerance(threshold))
  int   goal_power                 = 1;
  float goal_weight                = 5.0f;
  float goal_threshold             = 1.4f;

  // GoalAngleCritic (same gate as GoalCritic)
  int   goal_angle_power           = 1;
  float goal_angle_weight          = 3.0f;
  float goal_angle_threshold       = 0.5f;
  bool  goal_angle_symmetric       = false;

  // PathAlignCritic (gate: !within(threshold) + path_segments_count ≥ offset
  //                  + invalid_ratio < max_path_occupancy_ratio)
  int   path_align_power           = 1;
  float path_align_weight          = 10.0f;
  float path_align_max_path_occupancy = 0.07f;
  unsigned int path_align_offset_from_furthest = 20;
  unsigned int path_align_trajectory_point_step = 4;
  float path_align_threshold       = 0.5f;
  bool  path_align_use_orientations = false;

  // PathFollowCritic (gate: !within(threshold) + path size ≥ 2)
  int   path_follow_power          = 1;
  float path_follow_weight         = 5.0f;
  float path_follow_threshold      = 1.4f;
  unsigned int path_follow_offset_from_furthest = 6;

  // PathAngleCritic (gate: !within(threshold) + posePointAngle ≥ max_angle)
  unsigned int path_angle_offset_from_furthest = 4;
  int   path_angle_power           = 1;
  float path_angle_weight          = 2.0f;
  float path_angle_threshold       = 0.5f;
  float path_angle_max_angle_to_furthest = 1.2f;
  bool  path_angle_forward_preference = true;

  // PreferForwardCritic (gate: !within(threshold))
  int   prefer_forward_power       = 1;
  float prefer_forward_weight      = 5.0f;
  float prefer_forward_threshold   = 0.5f;
};

class CudaBackend : public mppi::ICudaBackend
{
public:
  // Allocate device buffers + stream + pinned staging to the configured max
  // sizes. Throws std::runtime_error on any CUDA failure; partial allocations
  // unwind via member dtors (RAII smart pointers).
  explicit CudaBackend(const CudaBackendConfig & cfg);
  ~CudaBackend() override = default;  // all members are RAII-owning

  CudaBackend(const CudaBackend &) = delete;
  CudaBackend & operator=(const CudaBackend &) = delete;

  // Implements ICudaBackend. Replicates Optimizer::optimize() iteration
  // body on the GPU. iteration_count loops are honoured.
  void optimize(mppi::Optimizer & optimizer) override;

  // Footprint upload — called once during plugin init (or whenever the
  // costmap_ros_ footprint changes). Vertices are in robot frame.
  void setFootprint(const std::vector<float> & fp_x, const std::vector<float> & fp_y);

  // Load all 8 critics' params from the same ROS rosparam tree the CPU
  // CriticManager reads. parent_name is the plugin's name (e.g.
  // "MPPIControllerROS") used as the parameters_handler namespace prefix.
  // Reads `<parent>.<CriticName>.<param>` — keys + defaults are identical
  // to nav_algo_core's per-critic initialize() bodies.
  void loadCriticParams(mppi::ParametersHandler * handler,
                        const std::string & parent_name);

  // Access the loaded params (for tests / inspection).
  const CriticParams & criticParams() const { return crit_; }

private:
  CudaBackendConfig cfg_;

  // Per-instance stream — kernel launches + memcpyAsync go through it.
  Stream stream_;

  // ── Device buffers (all RAII; freed by member dtors on object destruction
  //    AND on exception during construction of the containing object) ──
  DevicePtr<float>   d_traj_x_;
  DevicePtr<float>   d_traj_y_;
  DevicePtr<float>   d_traj_yaws_;
  DevicePtr<float>   d_state_vx_;
  DevicePtr<float>   d_state_vy_;
  DevicePtr<float>   d_state_wz_;
  DevicePtr<float>   d_state_cvx_;
  DevicePtr<float>   d_state_cvy_;
  DevicePtr<float>   d_state_cwz_;
  DevicePtr<float>   d_ctrl_vx_;
  DevicePtr<float>   d_ctrl_vy_;
  DevicePtr<float>   d_ctrl_wz_;
  DevicePtr<float>   d_costs_;
  DevicePtr<float>   d_softmax_;
  DevicePtr<float>   d_path_x_;
  DevicePtr<float>   d_path_y_;
  DevicePtr<float>   d_path_yaws_;
  DevicePtr<float>   d_path_int_dist_;
  DevicePtr<uint8_t> d_path_pts_valid_;
  DevicePtr<uint8_t> d_costmap_;
  DevicePtr<float>   d_fp_x_;
  DevicePtr<float>   d_fp_y_;

  // ── Pinned host staging for the largest H2D transfers ─────────────────
  // (state.* tensors and costmap). Other small uploads (ctrl, path) bypass
  // the staging path — their size is tiny enough that pageable cudaMemcpy
  // is already fast.
  HostPinnedPtr<uint8_t> h_costmap_stage_;
  HostPinnedPtr<float>   h_state_stage_;   // big enough for one B×T float row

  unsigned int fp_n_{0};
  CriticParams crit_;
};

}  // namespace nav_algo_mppi_cuda

#endif  // NAV_ALGO_MPPI_CUDA__CUDA_BACKEND_HPP_
