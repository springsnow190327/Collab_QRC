// cfpa2_coordinator_node_main.cpp — entry point for the dual-robot
// joint-allocator CFPA2 coordinator binary.
//
// The CFPA2_ROS1 guard selects the ROS 1 (Noetic) entry point; the
// default (no guard) is the production ROS 2 Humble path.

#include <memory>

#include "cfpa2_collaborative_autonomy/cfpa2_coordinator.hpp"

#ifdef CFPA2_ROS1
#include "ros/ros.h"

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "cfpa2_coordinator");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  cfpa2::CFPA2Coordinator node(nh, pnh, cfpa2::CFPA2Coordinator::Options{});
  ros::spin();
  return 0;
}
#else
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<cfpa2::CFPA2Coordinator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
#endif
