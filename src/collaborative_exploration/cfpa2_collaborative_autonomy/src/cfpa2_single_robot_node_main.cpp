// cfpa2_single_robot_node_main.cpp — entry point for the single-robot
// CFPA2 binary (decentralised production deployment).

#include <memory>

#include "cfpa2_collaborative_autonomy/cfpa2_single_robot.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<cfpa2::CFPA2SingleRobotNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
