// cfpa2_coordinator_node_main.cpp — entry point for the dual-robot
// joint-allocator CFPA2 coordinator binary.

#include <memory>

#include "cfpa2_collaborative_autonomy/cfpa2_coordinator.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<cfpa2::CFPA2Coordinator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
