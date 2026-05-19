// Copyright 2026 Collab_QRC
// SPDX-License-Identifier: Apache-2.0
//
// CudaMPPIController — nav2_core::Controller plugin that wraps a vendored,
// patched nav2_mppi_controller_cuda::MPPIController and, when `use_cuda` is
// true, constructs a nav_algo_mppi_cuda::CudaBackend and injects it into
// the inherited optimizer_ at configure() time. From the controller_server's
// perspective this is a normal Nav2 controller; the GPU path is transparent.

#ifndef NAV2_MPPI_CONTROLLER_CUDA_PLUGIN__CUDA_MPPI_CONTROLLER_HPP_
#define NAV2_MPPI_CONTROLLER_CUDA_PLUGIN__CUDA_MPPI_CONTROLLER_HPP_

#include <memory>
#include <string>

#include "nav2_mppi_controller/controller.hpp"  // patched upstream copy
#include "nav_algo_mppi_cuda/cuda_backend.hpp"  // CudaBackend impl

namespace nav2_mppi_controller_cuda_plugin
{

class CudaMPPIController : public nav2_mppi_controller::MPPIController
{
public:
  CudaMPPIController() = default;
  ~CudaMPPIController() override = default;

  // configure() runs the base MPPIController setup, then optionally
  // builds a CudaBackend and attaches it to the inherited optimizer_.
  // All other lifecycle hooks (cleanup/activate/deactivate/computeVelocity-
  // Commands/setPlan/setSpeedLimit) come from the base class unchanged.
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    const std::shared_ptr<tf2_ros::Buffer> tf,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;

private:
  std::unique_ptr<nav_algo_mppi_cuda::CudaBackend> cuda_backend_;
  bool cuda_enabled_{false};
};

}  // namespace nav2_mppi_controller_cuda_plugin

#endif  // NAV2_MPPI_CONTROLLER_CUDA_PLUGIN__CUDA_MPPI_CONTROLLER_HPP_
