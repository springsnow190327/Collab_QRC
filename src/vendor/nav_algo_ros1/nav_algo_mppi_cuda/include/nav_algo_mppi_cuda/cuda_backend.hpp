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
#include "nav_algo_mppi_cuda/critics.cuh"
#include "nav_algo_mppi_cuda/integrate.cuh"
#include "nav_algo_mppi_cuda/control_update.cuh"
#include "nav_algo_mppi_cuda/device_memory.hpp"

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
};

}  // namespace nav_algo_mppi_cuda

#endif  // NAV_ALGO_MPPI_CUDA__CUDA_BACKEND_HPP_
