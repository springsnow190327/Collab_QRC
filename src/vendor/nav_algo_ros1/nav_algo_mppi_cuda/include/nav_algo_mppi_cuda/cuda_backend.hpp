// Copyright 2026 Collab_QRC
// SPDX-License-Identifier: Apache-2.0
//
// Concrete GPU implementation of nav_algo_core::mppi::ICudaBackend.
//
// One CudaBackend instance per MPPI plugin. The MPPI controller plugin
// allocates the backend once (sizing from yaml + costmap dims), wires it
// into Optimizer via setCudaBackend(), and the optimize() hook then drives
// the full 11-kernel pipeline every evalControl() cycle.
//
// State / memory:
//   - All persistent GPU buffers are allocated in ctor and freed in dtor.
//   - optimize() does upload → kernel chain → download on the default
//     stream (sufficient for correctness; multi-stream pipelining is a
//     future optimisation).

#ifndef NAV_ALGO_MPPI_CUDA__CUDA_BACKEND_HPP_
#define NAV_ALGO_MPPI_CUDA__CUDA_BACKEND_HPP_

#include <cstdint>
#include <vector>

#include "nav_algo_core/mppi/cuda_backend.hpp"
#include "nav_algo_mppi_cuda/critics.cuh"
#include "nav_algo_mppi_cuda/integrate.cuh"
#include "nav_algo_mppi_cuda/control_update.cuh"

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

class CudaBackend : public mppi::ICudaBackend
{
public:
  // Allocate device buffers to the max sizes specified. Per-cycle the
  // active sizes (current P, current costmap dims) get re-checked; smaller
  // is fine, larger throws.
  explicit CudaBackend(const CudaBackendConfig & cfg);
  ~CudaBackend() override;

  CudaBackend(const CudaBackend &) = delete;
  CudaBackend & operator=(const CudaBackend &) = delete;

  // Implements ICudaBackend. Replicates Optimizer::optimize() iteration
  // body on the GPU. iteration_count loops are honoured.
  void optimize(mppi::Optimizer & opt) override;

  // Footprint upload — called once during plugin init (or whenever the
  // costmap_ros_ footprint changes). Vertices are in robot frame.
  void setFootprint(const std::vector<float> & fp_x, const std::vector<float> & fp_y);

private:
  CudaBackendConfig cfg_;

  // Per-iteration device buffers (sized to cfg_ maxes).
  // Trajectories
  float * d_traj_x_   {nullptr};   // B × T
  float * d_traj_y_   {nullptr};
  float * d_traj_yaws_{nullptr};
  // State velocity + noised controls
  float * d_state_vx_ {nullptr};   // B × T (post motion_model->predict)
  float * d_state_vy_ {nullptr};
  float * d_state_wz_ {nullptr};
  float * d_state_cvx_{nullptr};   // B × T (control + noise)
  float * d_state_cvy_{nullptr};
  float * d_state_cwz_{nullptr};
  // Control sequence
  float * d_ctrl_vx_  {nullptr};   // T
  float * d_ctrl_vy_  {nullptr};
  float * d_ctrl_wz_  {nullptr};
  // Costs / softmax
  float * d_costs_    {nullptr};   // B
  float * d_softmax_  {nullptr};   // B
  // Path
  float * d_path_x_         {nullptr};   // path_max_points
  float * d_path_y_         {nullptr};
  float * d_path_yaws_      {nullptr};
  float * d_path_int_dist_  {nullptr};
  uint8_t * d_path_pts_valid_{nullptr};
  // Costmap
  uint8_t * d_costmap_{nullptr};   // costmap_max_cells
  // Footprint
  float * d_fp_x_     {nullptr};
  float * d_fp_y_     {nullptr};
  unsigned int fp_n_  {0};
};

}  // namespace nav_algo_mppi_cuda

#endif  // NAV_ALGO_MPPI_CUDA__CUDA_BACKEND_HPP_
