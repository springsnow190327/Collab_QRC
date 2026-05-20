// cfpa2_single_robot.hpp — single-robot CFPA2 node.
//
// Subclasses CFPA2Coordinator. Adds:
//   - peer_coordination/blocked_frontiers subscriber (PR-4) + filter hook
//   - exploration_complete subscriber (pause flag)
//   - ramp_ascent_goal subscriber (optional override)
//
// Decentralised production deployment runs one of these per robot. The
// peer_coordinator_node (Python, separate package) sits alongside and
// republishes claimed frontiers as blocked_frontiers PoseArrays.

#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "cfpa2_collaborative_autonomy/cfpa2_coordinator.hpp"
#ifdef CFPA2_ROS1
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "std_msgs/String.h"
#else
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "std_msgs/msg/string.hpp"
#endif

namespace cfpa2 {

class CFPA2SingleRobotNode : public CFPA2Coordinator
{
public:
#ifdef CFPA2_ROS1
  CFPA2SingleRobotNode();
  CFPA2SingleRobotNode(ros::NodeHandle & nh, ros::NodeHandle & pnh);
#else
  explicit CFPA2SingleRobotNode(
      const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());
#endif

protected:
  bool is_goal_peer_claimed(Goal goal) override;

private:
#ifdef CFPA2_ROS1
  void init_single_robot();  // shared ctor body
  void on_blocked_frontiers(const geometry_msgs::PoseArray::ConstPtr & msg);
  void on_exploration_complete(const std_msgs::String::ConstPtr & msg);
  void on_ramp_ascent_goal(
      const geometry_msgs::PointStamped::ConstPtr & msg,
      const std::string & ns);
#else
  void on_blocked_frontiers(const geometry_msgs::msg::PoseArray::SharedPtr msg);
  void on_exploration_complete(const std_msgs::msg::String::SharedPtr msg);
  void on_ramp_ascent_goal(
      const geometry_msgs::msg::PointStamped::SharedPtr msg,
      const std::string & ns);
#endif

  // Match-tolerance + TTL constants — must stay in sync with
  // peer_coordinator_node.py.
  static constexpr double kPeerBlockedMatchTolM = 0.5;
  static constexpr double kPeerBlockedTimeoutSec = 12.0;

  std::vector<Goal> peer_blocked_frontiers_;
  std::uint64_t peer_blocked_received_ns_ = 0;

  std::string last_status_;
  std::string robot_namespace_;
};

}  // namespace cfpa2
