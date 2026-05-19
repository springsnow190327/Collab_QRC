// Copyright 2026 Collab_QRC
// SPDX-License-Identifier: Apache-2.0
//
// ROS 1 nav_core::BaseLocalPlanner wrapper around the ported Nav2 Humble
// MPPIController. All algorithm computation (Optimizer + CriticManager +
// PathHandler) is byte-identical to Humble upstream and lives under
// nav_algo_core; this wrapper handles:
//   - move_base coupling (initialize/setPlan/computeVelocityCommands/isGoalReached)
//   - current robot pose acquisition (TF map→base_link via Costmap2DROS)
//   - current velocity tracking (odom subscription, used as initial state)
//   - GoalChecker glue (xy_tolerance + yaw_tolerance fed into Optimizer.prepare)

#ifndef NAV_ALGO_MPPI_ROS1__MPPI_CONTROLLER_ROS_H_
#define NAV_ALGO_MPPI_ROS1__MPPI_CONTROLLER_ROS_H_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <nav_core/base_local_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include "nav_algo_core/compat.hpp"
#include "nav_algo_core/mppi/optimizer.hpp"
#include "nav_algo_core/mppi/tools/path_handler.hpp"
#include "nav_algo_core/mppi/tools/parameters_handler.hpp"

#ifdef NAV_ALGO_MPPI_HAS_CUDA
#include "nav_algo_mppi_cuda/cuda_backend.hpp"
#endif

namespace nav_algo_mppi_ros1
{

// Minimal GoalChecker that just feeds Optimizer.prepare() the user-configured
// xy + yaw tolerances. Matches nav2_controller::SimpleGoalChecker semantics
// for what the Optimizer actually consumes (it only ever reads tolerance
// fields, never the full goal-reached predicate).
class SimpleGoalChecker : public nav2_core::GoalChecker
{
public:
  SimpleGoalChecker(double xy, double yaw)
  : xy_tolerance_(xy), yaw_tolerance_(yaw) {}
  void getTolerances(geometry_msgs::Pose & pose_tol, geometry_msgs::Twist & vel_tol) override
  {
    pose_tol.position.x = xy_tolerance_;
    pose_tol.position.y = xy_tolerance_;
    pose_tol.orientation.z = std::sin(yaw_tolerance_ / 2.0);
    pose_tol.orientation.w = std::cos(yaw_tolerance_ / 2.0);
    vel_tol.linear.x = 1e9;
    vel_tol.linear.y = 1e9;
    vel_tol.angular.z = 1e9;
  }
private:
  double xy_tolerance_;
  double yaw_tolerance_;
};

class MPPIControllerROS : public nav_core::BaseLocalPlanner
{
public:
  MPPIControllerROS();
  ~MPPIControllerROS() override;

  // nav_core::BaseLocalPlanner
  void initialize(
    std::string name, tf2_ros::Buffer * tf,
    costmap_2d::Costmap2DROS * costmap_ros) override;
  bool setPlan(const std::vector<geometry_msgs::PoseStamped> & plan) override;
  bool computeVelocityCommands(geometry_msgs::Twist & cmd_vel) override;
  bool isGoalReached() override;

private:
  void odomCallback(const nav_msgs::Odometry::ConstPtr & msg);

  // Composed Nav2 algorithm objects (instantiated from nav_algo_core).
  mppi::Optimizer optimizer_;
  mppi::PathHandler path_handler_;
  std::unique_ptr<mppi::ParametersHandler> parameters_handler_;
  std::unique_ptr<SimpleGoalChecker> goal_checker_;

  costmap_2d::Costmap2DROS * costmap_ros_{nullptr};
  std::shared_ptr<costmap_2d::Costmap2DROS> costmap_ros_alias_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_alias_;
  std::string name_;
  std::string global_frame_;

  // Tolerances (used both for the GoalChecker shim and isGoalReached check).
  double xy_goal_tolerance_{0.25};
  double yaw_goal_tolerance_{0.25};

  // Latest /odom for velocity feedback. Nav2 receives this as part of the
  // computeVelocityCommands signature; move_base does not, so we subscribe.
  std::mutex odom_mutex_;
  nav_msgs::Odometry latest_odom_;
  ros::Subscriber odom_sub_;

  // Cached goal pose (used by isGoalReached).
  geometry_msgs::PoseStamped goal_pose_;
  bool have_plan_{false};

  // Pseudo-LifecycleNode so the optimizer can fetch params through our shim.
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> lc_node_;

  // Optional GPU acceleration. When `use_cuda: true` in yaml AND the plugin
  // was built with NAV_ALGO_MPPI_HAS_CUDA, a CudaBackend is allocated and
  // wired into the optimizer's optimize() dispatch hook. Otherwise the
  // xtensor CPU path runs unchanged.
#ifdef NAV_ALGO_MPPI_HAS_CUDA
  std::unique_ptr<nav_algo_mppi_cuda::CudaBackend> cuda_backend_;
#endif

  bool initialized_{false};
};

}  // namespace nav_algo_mppi_ros1

#endif  // NAV_ALGO_MPPI_ROS1__MPPI_CONTROLLER_ROS_H_
