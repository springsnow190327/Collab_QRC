// Copyright 2026 Collab_QRC
// SPDX-License-Identifier: Apache-2.0
//
// Abstract interface that lets an out-of-tree CUDA implementation
// intercept mppi::Optimizer::optimize() and replace the xtensor hot loop
// with GPU kernels. This header is CUDA-free; the concrete impl lives in
// nav_algo_mppi_cuda. nav2_mppi_controller_cuda retains its CPU-only
// build profile.
//
// IMPORTANT: this header MUST stay byte-equivalent in declaration to
// `src/vendor/nav_algo_ros1/nav_algo_core/include/nav_algo_core/mppi/cuda_backend.hpp`
// so a single CudaBackend translation unit satisfies both the ROS 1 and
// the ROS 2 builds.

#ifndef NAV2_MPPI_CONTROLLER__CUDA_BACKEND_HPP_
#define NAV2_MPPI_CONTROLLER__CUDA_BACKEND_HPP_

namespace mppi
{

class Optimizer;  // fwd-decl — interface only takes Optimizer& by reference

// Implementations must reproduce Optimizer::optimize()'s side-effects:
//   1. Fill optimizer.state() (cvx/cwz/cvy + propagated vx/wz/vy)
//   2. Fill optimizer.generated_trajectories() (x, y, yaws)
//   3. Run all enabled critics → accumulate into optimizer.costs()
//   4. Update optimizer.control_sequence() via softmax-weighted average
//   5. Call optimizer.applyControlSequenceConstraints() (or inline equivalent)
class ICudaBackend
{
public:
  virtual ~ICudaBackend() = default;
  virtual void optimize(Optimizer & optimizer) = 0;
};

}  // namespace mppi

#endif  // NAV2_MPPI_CONTROLLER__CUDA_BACKEND_HPP_
