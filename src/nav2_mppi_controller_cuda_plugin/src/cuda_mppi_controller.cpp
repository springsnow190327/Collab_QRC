// Copyright 2026 Collab_QRC
// SPDX-License-Identifier: Apache-2.0

#include "nav2_mppi_controller_cuda_plugin/cuda_mppi_controller.hpp"

#include <stdexcept>
#include <vector>

#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace nav2_mppi_controller_cuda_plugin
{

void CudaMPPIController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  const std::shared_ptr<tf2_ros::Buffer> tf,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  // Run the upstream MPPIController setup first. After this, the inherited
  // optimizer_, parameters_handler_, costmap_ros_, name_, logger_, etc. are
  // all valid and the CPU optimisation path is fully wired.
  MPPIController::configure(parent, name, tf, costmap_ros);

  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error(
      "CudaMPPIController::configure: parent LifecycleNode expired");
  }

  // `use_cuda` is plugin-private: lives under "<name>.use_cuda" in the yaml
  // tree, matching the convention the rest of nav2_mppi_controller uses.
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".use_cuda", rclcpp::ParameterValue(false));
  node->get_parameter(name + ".use_cuda", cuda_enabled_);

  if (!cuda_enabled_) {
    RCLCPP_INFO(
      logger_, "CudaMPPIController [%s]: use_cuda=false → upstream xtensor "
      "CPU path will run unchanged.", name.c_str());
    return;
  }

  // ── Build the CUDA backend. Sizing comes from the now-configured
  //    Optimizer settings + the costmap dimensions.
  nav_algo_mppi_cuda::CudaBackendConfig bcfg{};
  bcfg.batch_size       = optimizer_.settings().batch_size;
  bcfg.time_steps       = optimizer_.settings().time_steps;
  bcfg.path_max_points  = 1024;
  // 4 M cells (~4 MB uint8) covers any reasonable nav2 local costmap.
  bcfg.costmap_max_cells = 4u * 1024u * 1024u;
  bcfg.footprint_max_n  = 16;

  try {
    cuda_backend_ = std::make_unique<nav_algo_mppi_cuda::CudaBackend>(bcfg);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      logger_,
      "CudaMPPIController [%s]: failed to construct CudaBackend (%s). "
      "Falling back to CPU path.", name.c_str(), e.what());
    cuda_enabled_ = false;
    return;
  }

  // Upload the robot footprint so ObstaclesCritic's footprint-aware kernel
  // path can run (consider_footprint=true case).
  if (costmap_ros_) {
    std::vector<float> fp_x, fp_y;
    for (const auto & p : costmap_ros_->getRobotFootprint()) {
      fp_x.push_back(static_cast<float>(p.x));
      fp_y.push_back(static_cast<float>(p.y));
    }
    if (!fp_x.empty()) {
      cuda_backend_->setFootprint(fp_x, fp_y);
    }
  }

  // Pull yaml-loaded per-critic weights + gates from the same tree the
  // CPU CriticManager already read. Matches nav_algo_core CPU semantics.
  cuda_backend_->loadCriticParams(parameters_handler_.get(), name_);

  // Attach. From this point on, Optimizer::optimize() routes through the
  // CudaBackend instead of the xtensor iteration loop.
  optimizer_.setCudaBackend(cuda_backend_.get());

  RCLCPP_INFO(
    logger_,
    "CudaMPPIController [%s]: CUDA backend ENABLED (B=%u T=%u "
    "footprint_n=%zu, critic params loaded from yaml).",
    name.c_str(), bcfg.batch_size, bcfg.time_steps,
    costmap_ros_ ? costmap_ros_->getRobotFootprint().size() : 0u);
}

void CudaMPPIController::cleanup()
{
  // Detach before the underlying optimiser tears its references down.
  if (cuda_enabled_) {
    optimizer_.setCudaBackend(nullptr);
  }
  cuda_backend_.reset();
  cuda_enabled_ = false;
  MPPIController::cleanup();
}

}  // namespace nav2_mppi_controller_cuda_plugin

PLUGINLIB_EXPORT_CLASS(
  nav2_mppi_controller_cuda_plugin::CudaMPPIController, nav2_core::Controller)
