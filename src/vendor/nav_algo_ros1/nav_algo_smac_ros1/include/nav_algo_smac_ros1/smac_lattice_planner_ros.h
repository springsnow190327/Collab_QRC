// Copyright 2026 Collab_QRC
// SPDX-License-Identifier: Apache-2.0
//
// ROS 1 nav_core::BaseGlobalPlanner wrapper around the ported Nav2 Humble
// SmacPlannerLattice. The wrapper handles ROS 1-side plumbing only — param
// loading, costmap binding, output Path packaging. The algorithm core
// (AStarAlgorithm<NodeLattice> + AnalyticExpansion + Smoother) lives in
// nav_algo_core and is byte-identical to Humble upstream.

#ifndef NAV_ALGO_SMAC_ROS1__SMAC_LATTICE_PLANNER_ROS_H_
#define NAV_ALGO_SMAC_ROS1__SMAC_LATTICE_PLANNER_ROS_H_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <nav_core/base_global_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#include "nav_algo_core/compat.hpp"
#include "nav_algo_core/smac/a_star.hpp"
#include "nav_algo_core/smac/node_lattice.hpp"
#include "nav_algo_core/smac/collision_checker.hpp"
#include "nav_algo_core/smac/smoother.hpp"
#include "nav_algo_core/smac/types.hpp"
#include "nav_algo_core/smac/utils.hpp"

namespace nav_algo_smac_ros1
{

class SmacLatticePlannerROS : public nav_core::BaseGlobalPlanner
{
public:
  SmacLatticePlannerROS();
  ~SmacLatticePlannerROS() override;

  // nav_core::BaseGlobalPlanner
  void initialize(std::string name, costmap_2d::Costmap2DROS * costmap_ros) override;
  bool makePlan(
    const geometry_msgs::PoseStamped & start,
    const geometry_msgs::PoseStamped & goal,
    std::vector<geometry_msgs::PoseStamped> & plan) override;

private:
  // Same parameter order/semantics as Nav2 SmacPlannerLattice::configure().
  void loadParams(ros::NodeHandle & pnh, const std::string & name);

  // Mirrors Nav2's `_a_star->createPath` invocation + path walk-back.
  // Returns nav_msgs/Path in the global frame, smoothed if smoother enabled.
  // Internally clones Nav2's createPlan() function from Humble verbatim.
  nav_msgs::Path createLatticePlan(
    const geometry_msgs::PoseStamped & start,
    const geometry_msgs::PoseStamped & goal);

  // ── State ────────────────────────────────────────────────────────────
  std::unique_ptr<nav2_smac_planner::AStarAlgorithm<nav2_smac_planner::NodeLattice>> a_star_;
  std::unique_ptr<nav2_smac_planner::Smoother> smoother_;
  std::shared_ptr<nav2_smac_planner::GridCollisionChecker> collision_checker_;

  costmap_2d::Costmap2D * costmap_;
  costmap_2d::Costmap2DROS * costmap_ros_;
  std::string global_frame_;
  std::string name_;

  // Params (mirroring Nav2 SmacPlannerLattice yaml schema)
  float tolerance_{0.25f};
  bool allow_unknown_{true};
  int max_iterations_{1000000};
  int max_on_approach_iterations_{1000};
  double max_planning_time_{5.0};
  double lookup_table_size_{20.0};
  bool smooth_path_{true};
  nav2_smac_planner::SearchInfo search_info_;
  nav2_smac_planner::LatticeMetadata metadata_;
  nav2_smac_planner::MotionModel motion_model_{nav2_smac_planner::MotionModel::STATE_LATTICE};

  std::mutex mutex_;
  bool initialized_{false};

  ros::Publisher raw_plan_pub_;
};

}  // namespace nav_algo_smac_ros1

#endif  // NAV_ALGO_SMAC_ROS1__SMAC_LATTICE_PLANNER_ROS_H_
