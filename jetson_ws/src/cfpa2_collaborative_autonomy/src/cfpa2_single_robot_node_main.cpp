// cfpa2_single_robot_node_main.cpp — entry point for the single-robot
// CFPA2 binary (decentralised production deployment).
//
// The CFPA2_ROS1 guard selects the ROS 1 (Noetic) entry point; the
// default (no guard) is the production ROS 2 Humble path.

#include <memory>

#include "cfpa2_collaborative_autonomy/cfpa2_single_robot.hpp"

#ifdef CFPA2_ROS1
#include "ros/ros.h"

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "cfpa2_single_robot");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  cfpa2::CFPA2SingleRobotNode node(nh, pnh);
  ros::spin();
  return 0;
}
#else
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<cfpa2::CFPA2SingleRobotNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
#endif
