// Copyright 2026 Collab_QRC
// SPDX-License-Identifier: Apache-2.0
//
// Implementation: thin BaseLocalPlanner adapter around the ported Nav2
// MPPIController. Every per-cycle invocation flows through the exact same
// Optimizer.evalControl() call Nav2 uses, with identical inputs (robot pose,
// robot speed, transformed plan, goal checker). Math equivalence to Humble
// is preserved by construction.

#include "nav_algo_mppi_ros1/mppi_controller_ros.h"

#include <pluginlib/class_list_macros.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace nav_algo_mppi_ros1
{

MPPIControllerROS::MPPIControllerROS() = default;
MPPIControllerROS::~MPPIControllerROS() { optimizer_.shutdown(); }

void MPPIControllerROS::initialize(
  std::string name, tf2_ros::Buffer * tf, costmap_2d::Costmap2DROS * costmap_ros)
{
  if (initialized_) {
    ROS_WARN("MPPIControllerROS [%s] already initialized; skipping.", name.c_str());
    return;
  }
  name_ = name;
  costmap_ros_ = costmap_ros;
  // Aliasing shared_ptrs: nav_algo_core takes shared_ptrs by value but the
  // move_base-owned pointers must NOT be deleted by us.
  costmap_ros_alias_ = std::shared_ptr<costmap_2d::Costmap2DROS>(
    costmap_ros, [](costmap_2d::Costmap2DROS *) {});
  tf_buffer_alias_ = std::shared_ptr<tf2_ros::Buffer>(
    tf, [](tf2_ros::Buffer *) {});
  global_frame_ = costmap_ros_->getGlobalFrameID();

  // The Nav2 Optimizer reads its top-level params (controller_frequency,
  // ControllerFrequency, the etcs) via getParamGetter(""), which under Nav2
  // resolves to the controller_server node's namespace, NOT FollowPath's.
  // For move_base equivalence: the parent NodeHandle is move_base's "~/"
  // (containing controller_frequency 20.0 set in move_base_params.yaml).
  // The plugin's own params live under "~/<plugin_name>/" and are reached
  // via getParamGetter("<plugin_name>") which concatenates with "/".
  ros::NodeHandle parent_nh("~");
  ros::NodeHandle pnh("~/" + name_);

  pnh.param<double>("xy_goal_tolerance",  xy_goal_tolerance_,  0.25);
  pnh.param<double>("yaw_goal_tolerance", yaw_goal_tolerance_, 0.25);
  goal_checker_ = std::make_unique<SimpleGoalChecker>(
    xy_goal_tolerance_, yaw_goal_tolerance_);

  // Pseudo-LifecycleNode the ported code uses for clock/logger/param reads.
  lc_node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>(name_, pnh);

  // Wire up ParametersHandler at the PARENT (move_base) level, so empty-ns
  // getters reach controller_frequency / TFBufferTimeout / etc., and
  // <name>-prefixed getters reach the plugin's own block.
  rclcpp_lifecycle::LifecycleNode::WeakPtr weak_lc = lc_node_;
  parameters_handler_ = std::make_unique<mppi::ParametersHandler>(weak_lc);
  parameters_handler_->setNodeHandle(parent_nh);

  // Initialise the algorithm core. Same call sequence as Nav2's
  // MPPIController::configure.
  optimizer_.initialize(weak_lc, name_, costmap_ros_alias_, parameters_handler_.get());
  path_handler_.initialize(
    weak_lc, name_, costmap_ros_alias_, tf_buffer_alias_,
    parameters_handler_.get());

  // Odometry feedback: Nav2 gets robot_speed via the Controller API; in ROS 1
  // move_base does not forward it, so subscribe directly.
  std::string odom_topic;
  pnh.param<std::string>("odom_topic", odom_topic, "/odom");
  odom_sub_ = pnh.subscribe(odom_topic, 1, &MPPIControllerROS::odomCallback, this);

  // ── Optional GPU acceleration via CudaBackend ───────────────────────────
  // `use_cuda` param (plugin-private NS, default false). Backend buffer
  // sizes pulled from the canonical Nav2 MPPI shape + costmap dimensions.
  bool use_cuda = false;
  pnh.param<bool>("use_cuda", use_cuda, false);

#ifdef NAV_ALGO_MPPI_HAS_CUDA
  if (use_cuda) {
    nav_algo_mppi_cuda::CudaBackendConfig bcfg{};
    bcfg.batch_size       = optimizer_.settings().batch_size;
    bcfg.time_steps       = optimizer_.settings().time_steps;
    bcfg.path_max_points  = 1024;
    // Costmap on Nav2 sim is 200×200 (world-fixed); allow generous max so
    // a re-rolled costmap fits without re-alloc.
    bcfg.costmap_max_cells = 4 * 1024 * 1024;  // 4 M cells (~4 MB uint8)
    bcfg.footprint_max_n  = 16;
    cuda_backend_ = std::make_unique<nav_algo_mppi_cuda::CudaBackend>(bcfg);

    // Upload the robot footprint from costmap_ros so ObstaclesCritic's
    // footprint kernel can run.
    std::vector<float> fp_x, fp_y;
    for (const auto & p : costmap_ros_->getRobotFootprint()) {
      fp_x.push_back(static_cast<float>(p.x));
      fp_y.push_back(static_cast<float>(p.y));
    }
    if (!fp_x.empty()) {
      cuda_backend_->setFootprint(fp_x, fp_y);
    }
    optimizer_.setCudaBackend(cuda_backend_.get());
    ROS_INFO("MPPIControllerROS [%s] CUDA backend ENABLED "
             "(B=%u T=%u footprint_n=%zu).",
             name_.c_str(), bcfg.batch_size, bcfg.time_steps, fp_x.size());
  }
#else
  if (use_cuda) {
    ROS_WARN("MPPIControllerROS [%s] use_cuda=true requested but plugin "
             "was built without NAV_ALGO_MPPI_HAS_CUDA. Falling back to CPU.",
             name_.c_str());
  }
#endif

  ROS_INFO(
    "MPPIControllerROS [%s] initialised. xy_tol=%.2f yaw_tol=%.2f odom=%s use_cuda=%s",
    name_.c_str(), xy_goal_tolerance_, yaw_goal_tolerance_, odom_topic.c_str(),
    use_cuda ? "true" : "false");

  initialized_ = true;
}

void MPPIControllerROS::odomCallback(const nav_msgs::Odometry::ConstPtr & msg)
{
  std::lock_guard<std::mutex> lock(odom_mutex_);
  latest_odom_ = *msg;
}

bool MPPIControllerROS::setPlan(const std::vector<geometry_msgs::PoseStamped> & plan)
{
  if (!initialized_ || plan.empty()) { return false; }
  nav_msgs::Path path_msg;
  path_msg.header = plan.front().header;
  path_msg.poses  = plan;
  path_handler_.setPath(path_msg);
  goal_pose_ = plan.back();
  have_plan_ = true;
  return true;
}

bool MPPIControllerROS::computeVelocityCommands(geometry_msgs::Twist & cmd_vel)
{
  if (!initialized_ || !have_plan_) { return false; }

  // Current robot pose: ask Costmap2DROS (it already does tf lookup with
  // proper transform_tolerance handling).
  geometry_msgs::PoseStamped robot_pose;
  if (!costmap_ros_->getRobotPose(robot_pose)) {
    ROS_WARN_THROTTLE(1.0, "MPPIControllerROS [%s]: failed to get robot pose", name_.c_str());
    return false;
  }

  // Current velocity from latest odom.
  geometry_msgs::Twist robot_speed;
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    robot_speed = latest_odom_.twist.twist;
  }

  // Lock costmap + run the Nav2 optimizer.
  std::lock_guard<std::mutex> param_lock(*parameters_handler_->getLock());
  nav_msgs::Path transformed_plan;
  try {
    transformed_plan = path_handler_.transformPath(robot_pose);
  } catch (const std::exception & e) {
    ROS_WARN_THROTTLE(1.0,
      "MPPIControllerROS [%s]: transformPath failed: %s", name_.c_str(), e.what());
    return false;
  }

  costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
  std::unique_lock<costmap_2d::Costmap2D::mutex_t> costmap_lock(*(costmap->getMutex()));

  geometry_msgs::TwistStamped twist;
  try {
    twist = optimizer_.evalControl(
      robot_pose, robot_speed, transformed_plan, goal_checker_.get());
  } catch (const std::exception & e) {
    ROS_WARN_THROTTLE(1.0,
      "MPPIControllerROS [%s]: evalControl failed: %s", name_.c_str(), e.what());
    return false;
  }

  cmd_vel = twist.twist;
  return true;
}

bool MPPIControllerROS::isGoalReached()
{
  if (!initialized_ || !have_plan_) { return false; }
  geometry_msgs::PoseStamped pose;
  if (!costmap_ros_->getRobotPose(pose)) { return false; }
  const double dx = pose.pose.position.x - goal_pose_.pose.position.x;
  const double dy = pose.pose.position.y - goal_pose_.pose.position.y;
  const double dist = std::hypot(dx, dy);
  if (dist > xy_goal_tolerance_) { return false; }
  const double yaw_now  = tf2::getYaw(pose.pose.orientation);
  const double yaw_goal = tf2::getYaw(goal_pose_.pose.orientation);
  double dyaw = std::fabs(yaw_now - yaw_goal);
  while (dyaw > M_PI) { dyaw -= 2.0 * M_PI; }
  return std::fabs(dyaw) <= yaw_goal_tolerance_;
}

}  // namespace nav_algo_mppi_ros1

PLUGINLIB_EXPORT_CLASS(nav_algo_mppi_ros1::MPPIControllerROS,
                      nav_core::BaseLocalPlanner)
