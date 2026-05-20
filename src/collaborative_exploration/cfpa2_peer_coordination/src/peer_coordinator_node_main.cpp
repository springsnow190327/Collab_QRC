// peer_coordinator_node_main.cpp — entry point for the decentralised CFPA2 peer-coordination node.

#include <memory>

#include "cfpa2_peer_coordination/peer_coordinator.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<cfpa2_peer_coordination::PeerCoordinatorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}