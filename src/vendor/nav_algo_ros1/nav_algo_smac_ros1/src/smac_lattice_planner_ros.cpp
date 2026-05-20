// Copyright 2026 Collab_QRC
// SPDX-License-Identifier: Apache-2.0
//
// Implementation faithfully mirrors nav2_smac_planner::SmacPlannerLattice
// from Humble: same sequence of operations in configure/createPlan, same
// numerical defaults and clamps. Only the wire-up to ROS is different
// (rosparam vs declare_parameter, raw Costmap2D* vs Costmap2DROS::SharedPtr,
// nav_msgs::Path vs nav_msgs::msg::Path).

#include "nav_algo_smac_ros1/smac_lattice_planner_ros.h"

#include <algorithm>
#include <chrono>
#include <limits>

#include <pluginlib/class_list_macros.h>
#include <ros/package.h>
#include <tf2/utils.h>
#include <angles/angles.h>

namespace nav_algo_smac_ros1
{

using namespace nav2_smac_planner;  // NOLINT

SmacLatticePlannerROS::SmacLatticePlannerROS()
: a_star_(nullptr),
  smoother_(nullptr),
  collision_checker_(nullptr),
  costmap_(nullptr),
  costmap_ros_(nullptr) {}

SmacLatticePlannerROS::~SmacLatticePlannerROS() {}

void SmacLatticePlannerROS::initialize(
  std::string name, costmap_2d::Costmap2DROS * costmap_ros)
{
  if (initialized_) {
    ROS_WARN("SmacLatticePlannerROS [%s] already initialized; skipping.", name.c_str());
    return;
  }
  name_ = name;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  // Plugin's private nodehandle: under move_base it is ~/<name>.
  ros::NodeHandle pnh("~/" + name);
  loadParams(pnh, name);

  raw_plan_pub_ = pnh.advertise<nav_msgs::Path>("unsmoothed_plan", 1, /*latch=*/false);

  // Lookup-table dimension math, copied verbatim from Nav2.
  float lookup_table_dim =
    static_cast<float>(lookup_table_size_) /
    static_cast<float>(costmap_->getResolution());
  lookup_table_dim = static_cast<float>(static_cast<int>(lookup_table_dim));
  if (static_cast<int>(lookup_table_dim) % 2 == 0) {
    ROS_INFO(
      "Even sized heuristic lookup table size %.1f; bumping to next odd to keep parity.",
      lookup_table_dim);
    lookup_table_dim += 1.0f;
  }

  // Collision checker: 72 quantizations (5° bins). Matches Nav2 exactly.
  // The shared_ptr wraps a LifecycleNode bound to our pnh for clock/logger.
  auto lc_node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(name_, pnh);
  collision_checker_ = std::make_shared<GridCollisionChecker>(
    costmap_, 72u, lc_node);

  // Footprint detection: Nav2 calls costmap_ros_->getUseRadius() but Noetic
  // costmap_2d has no such method. Use the heuristic: if the user gave a
  // single-element radius footprint (i.e. costmap converted robot_radius into
  // a circle) treat as radius mode. Otherwise polygon mode.
  std::vector<geometry_msgs::Point> footprint = costmap_ros_->getRobotFootprint();
  bool use_radius = false;
  pnh.param<bool>("use_radius", use_radius, false);

  // findCircumscribedCost in nav_algo_core takes a Costmap2DROS::SharedPtr.
  // Construct an aliasing shared_ptr that doesn't own the move_base-owned
  // costmap_ros pointer; the destructor is a no-op.
  std::shared_ptr<costmap_2d::Costmap2DROS> ros_alias(
    costmap_ros_, [](costmap_2d::Costmap2DROS *) {});
  collision_checker_->setFootprint(
    footprint, use_radius, findCircumscribedCost(ros_alias));

  // Build A* — same template instantiation Nav2 uses for STATE_LATTICE.
  a_star_ = std::make_unique<AStarAlgorithm<NodeLattice>>(motion_model_, search_info_);
  a_star_->initialize(
    allow_unknown_,
    max_iterations_,
    max_on_approach_iterations_,
    max_planning_time_,
    lookup_table_dim,
    metadata_.number_of_headings);

  if (smooth_path_) {
    SmootherParams sp;
    // SmootherParams::get(node, name) in Nav2 reads from rclcpp. The Noetic
    // version reads from rosparam under <name>.smoother.*. nav_algo_core's
    // SmootherParams::get() goes through our LifecycleNode shim → rosparam.
    sp.get(lc_node, name_);
    smoother_ = std::make_unique<Smoother>(sp);
    smoother_->initialize(metadata_.min_turning_radius);
  }

  ROS_INFO(
    "SmacLatticePlannerROS [%s] initialised. tolerance=%.2f, max_iter=%d, "
    "lattice=%s, headings=%u, turning_radius=%.2fm",
    name_.c_str(), tolerance_, max_iterations_,
    search_info_.lattice_filepath.c_str(), metadata_.number_of_headings,
    metadata_.min_turning_radius);

  initialized_ = true;
}

void SmacLatticePlannerROS::loadParams(ros::NodeHandle & pnh, const std::string & name)
{
  double analytic_expansion_max_length_m;

  pnh.param<float>("tolerance",                     tolerance_,                         0.25f);
  pnh.param<bool>("allow_unknown",                  allow_unknown_,                     true);
  pnh.param<int>("max_iterations",                  max_iterations_,                    1000000);
  pnh.param<int>("max_on_approach_iterations",      max_on_approach_iterations_,        1000);
  pnh.param<bool>("smooth_path",                    smooth_path_,                       true);
  pnh.param<double>("max_planning_time",            max_planning_time_,                 5.0);
  pnh.param<double>("lookup_table_size",            lookup_table_size_,                 20.0);

  // SearchInfo: same defaults as Nav2 SmacPlannerLattice::configure.
  std::string default_lattice =
    ros::package::getPath("nav_algo_smac_ros1") +
    "/sample_primitives/5cm_resolution/0.5m_turning_radius/diff/output.json";
  pnh.param<std::string>("lattice_filepath", search_info_.lattice_filepath, default_lattice);
  // Empty string in yaml means "use the shipped default"; resolve here so
  // operators can leave the field blank without crashing the planner.
  if (search_info_.lattice_filepath.empty()) {
    search_info_.lattice_filepath = default_lattice;
  }

  pnh.param<bool>("cache_obstacle_heuristic", search_info_.cache_obstacle_heuristic, false);
  // SearchInfo uses float for penalties — load via double then cast.
  double rev_pen, change_pen, ns_pen, cost_pen, retro_pen, rot_pen, expansion_ratio;
  pnh.param<double>("reverse_penalty",          rev_pen,           2.0);
  pnh.param<double>("change_penalty",           change_pen,        0.05);
  pnh.param<double>("non_straight_penalty",     ns_pen,            1.05);
  pnh.param<double>("cost_penalty",             cost_pen,          2.0);
  pnh.param<double>("retrospective_penalty",    retro_pen,         0.015);
  pnh.param<double>("rotation_penalty",         rot_pen,           5.0);
  pnh.param<double>("analytic_expansion_ratio", expansion_ratio,   3.5);
  pnh.param<double>("analytic_expansion_max_length", analytic_expansion_max_length_m, 3.0);
  pnh.param<bool>("allow_reverse_expansion", search_info_.allow_reverse_expansion, false);
  search_info_.reverse_penalty            = static_cast<float>(rev_pen);
  search_info_.change_penalty             = static_cast<float>(change_pen);
  search_info_.non_straight_penalty       = static_cast<float>(ns_pen);
  search_info_.cost_penalty               = static_cast<float>(cost_pen);
  search_info_.retrospective_penalty      = static_cast<float>(retro_pen);
  search_info_.rotation_penalty           = static_cast<float>(rot_pen);
  search_info_.analytic_expansion_ratio   = static_cast<float>(expansion_ratio);
  search_info_.analytic_expansion_max_length =
    static_cast<float>(analytic_expansion_max_length_m / costmap_->getResolution());

  // Load lattice JSON metadata + derive minimum_turning_radius in cells.
  metadata_ = LatticeMotionTable::getLatticeMetadata(search_info_.lattice_filepath);
  search_info_.minimum_turning_radius =
    metadata_.min_turning_radius / static_cast<float>(costmap_->getResolution());

  if (max_on_approach_iterations_ <= 0) {
    max_on_approach_iterations_ = std::numeric_limits<int>::max();
  }
  if (max_iterations_ <= 0) {
    max_iterations_ = std::numeric_limits<int>::max();
  }

  motion_model_ = MotionModel::STATE_LATTICE;
  (void)name;
}

bool SmacLatticePlannerROS::makePlan(
  const geometry_msgs::PoseStamped & start,
  const geometry_msgs::PoseStamped & goal,
  std::vector<geometry_msgs::PoseStamped> & plan)
{
  if (!initialized_) {
    ROS_ERROR("SmacLatticePlannerROS::makePlan called before initialize()");
    return false;
  }
  plan.clear();
  nav_msgs::Path raw = createLatticePlan(start, goal);
  if (raw.poses.empty()) { return false; }
  plan = raw.poses;
  return true;
}

// Verbatim port of Nav2 SmacPlannerLattice::createPlan, ROS 1 message types.
nav_msgs::Path SmacLatticePlannerROS::createLatticePlan(
  const geometry_msgs::PoseStamped & start,
  const geometry_msgs::PoseStamped & goal)
{
  std::lock_guard<std::mutex> lock_reinit(mutex_);
  auto t_start = std::chrono::steady_clock::now();

  std::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));

  // Refresh footprint each plan call (mirrors Nav2).
  std::shared_ptr<costmap_2d::Costmap2DROS> ros_alias(
    costmap_ros_, [](costmap_2d::Costmap2DROS *) {});
  bool use_radius = false;
  ros::NodeHandle("~/" + name_).param<bool>("use_radius", use_radius, false);
  collision_checker_->setFootprint(
    costmap_ros_->getRobotFootprint(),
    use_radius,
    findCircumscribedCost(ros_alias));
  a_star_->setCollisionChecker(collision_checker_.get());

  // Start
  unsigned int mx_start, my_start, mx_goal, my_goal;
  costmap_->worldToMap(start.pose.position.x, start.pose.position.y, mx_start, my_start);
  unsigned int start_bin =
    NodeLattice::motion_table.getClosestAngularBin(tf2::getYaw(start.pose.orientation));
  a_star_->setStart(mx_start, my_start, start_bin);

  // Goal
  costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, mx_goal, my_goal);
  unsigned int goal_bin =
    NodeLattice::motion_table.getClosestAngularBin(tf2::getYaw(goal.pose.orientation));
  a_star_->setGoal(mx_goal, my_goal, goal_bin);

  nav_msgs::Path plan;
  plan.header.stamp = ros::Time::now();
  plan.header.frame_id = global_frame_;
  geometry_msgs::PoseStamped pose;
  pose.header = plan.header;
  pose.pose.position.z = 0.0;
  pose.pose.orientation.w = 1.0;

  // Same-cell corner case.
  if (mx_start == mx_goal && my_start == my_goal && start_bin == goal_bin) {
    pose.pose = start.pose;
    pose.pose.orientation = goal.pose.orientation;
    plan.poses.push_back(pose);
    if (raw_plan_pub_.getNumSubscribers() > 0) raw_plan_pub_.publish(plan);
    return plan;
  }

  // Run A*.
  NodeLattice::CoordinateVector path;
  int num_iterations = 0;
  std::string error;
  try {
    if (!a_star_->createPath(
        path, num_iterations,
        tolerance_ / static_cast<float>(costmap_->getResolution())))
    {
      error = (num_iterations < a_star_->getMaxIterations())
        ? "no valid path found" : "exceeded maximum iterations";
    }
  } catch (const std::runtime_error & e) {
    error = std::string("invalid use: ") + e.what();
  }

  if (!error.empty()) {
    ROS_WARN("SmacLatticePlannerROS [%s] failed: %s", name_.c_str(), error.c_str());
    return plan;
  }

  // Walk back: A* returns goal→start order, we publish start→goal.
  plan.poses.reserve(path.size());
  geometry_msgs::PoseStamped last_pose = pose;
  for (int i = static_cast<int>(path.size()) - 1; i >= 0; --i) {
    pose.pose = getWorldCoords(path[i].x, path[i].y, costmap_);
    pose.pose.orientation = getWorldOrientation(path[i].theta);
    // Drop duplicate consecutive poses (rare but possible at junctions).
    if (std::fabs(pose.pose.position.x - last_pose.pose.position.x) < 1e-4 &&
        std::fabs(pose.pose.position.y - last_pose.pose.position.y) < 1e-4 &&
        std::fabs(
          tf2::getYaw(pose.pose.orientation) - tf2::getYaw(last_pose.pose.orientation)) < 1e-4)
    {
      continue;
    }
    last_pose = pose;
    plan.poses.push_back(pose);
  }

  if (raw_plan_pub_.getNumSubscribers() > 0) raw_plan_pub_.publish(plan);

  // Smooth using whatever's left of max_planning_time.
  auto t_end_search = std::chrono::steady_clock::now();
  std::chrono::duration<double> elapsed = t_end_search - t_start;
  double time_remaining = max_planning_time_ - elapsed.count();

  if (smoother_ && num_iterations > 1) {
    smoother_->smooth(plan, costmap_, time_remaining);
  }

  return plan;
}

}  // namespace nav_algo_smac_ros1

PLUGINLIB_EXPORT_CLASS(nav_algo_smac_ros1::SmacLatticePlannerROS,
                      nav_core::BaseGlobalPlanner)
