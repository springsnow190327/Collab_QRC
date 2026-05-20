// ros2/rclcpp_visualizer.hpp — ROS 2 adapter implementing core::IVisualizer.
// Wraps rclcpp::Publisher<OccupancyGrid/MarkerArray> for coordinator map +
// robot pose / trajectory markers + frontier marker spheres.

#pragma once

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/qos.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "cfpa2_collaborative_autonomy/core/output.hpp"
#include "cfpa2_collaborative_autonomy/ros2/conversions.hpp"

namespace cfpa2 {
namespace ros2 {

class RclcppVisualizer : public core::IVisualizer
{
public:
  RclcppVisualizer(
      rclcpp::Node * node,
      const std::string & coordinator_map_topic,
      const std::string & robot_markers_topic,
      const std::string & frontier_markers_topic,
      double robot_marker_scale)
  : clock_(node->get_clock()),
    robot_marker_scale_(robot_marker_scale)
  {
    rclcpp::QoS coord_qos(rclcpp::KeepLast(1));
    coord_qos.reliable();
    coord_qos.transient_local();
    coord_map_pub_ = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
        coordinator_map_topic, coord_qos);
    robot_markers_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
        robot_markers_topic, 10);
    frontier_markers_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
        frontier_markers_topic, 10);
  }

  void publish_coordinator_map(const core::Grid & grid) override
  {
    coord_map_pub_->publish(to_msg_grid(grid));
  }

  void publish_robot_markers(
      const std::vector<core::RobotPoseView> & robot_poses,
      const std::vector<core::TrajectoryView> & trajectories) override
  {
    visualization_msgs::msg::MarkerArray arr;
    int id = 0;
    const auto stamp = clock_->now();
    for (const auto & p : robot_poses) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = p.frame_id;
      m.header.stamp = stamp;
      m.ns = "mtare_robot_pose";
      m.id = id++;
      m.type = visualization_msgs::msg::Marker::SPHERE;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x = p.x;
      m.pose.position.y = p.y;
      m.pose.position.z = p.z;
      const double half = p.yaw * 0.5;
      m.pose.orientation.w = std::cos(half);
      m.pose.orientation.z = std::sin(half);
      m.scale.x = m.scale.y = m.scale.z = p.scale;
      m.color.r = p.color[0];
      m.color.g = p.color[1];
      m.color.b = p.color[2];
      m.color.a = 1.0f;
      arr.markers.push_back(m);
    }
    for (const auto & t : trajectories) {
      if (t.points_xy.empty()) continue;
      visualization_msgs::msg::Marker m;
      m.header.frame_id = t.frame_id;
      m.header.stamp = stamp;
      m.ns = "mtare_robot_traj";
      m.id = id++;
      m.type = visualization_msgs::msg::Marker::LINE_STRIP;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.scale.x = 0.05;
      m.color.r = t.color[0];
      m.color.g = t.color[1];
      m.color.b = t.color[2];
      m.color.a = 0.6f;
      for (const auto & xy : t.points_xy) {
        geometry_msgs::msg::Point p;
        p.x = xy.first;
        p.y = xy.second;
        p.z = 0.05;
        m.points.push_back(p);
      }
      arr.markers.push_back(m);
    }
    robot_markers_pub_->publish(arr);
  }

  void publish_frontier_markers(
      const std::string & frame_id,
      const std::vector<core::Goal> & frontiers) override
  {
    visualization_msgs::msg::MarkerArray arr;
    visualization_msgs::msg::Marker m;
    m.header.frame_id = frame_id;
    m.header.stamp = clock_->now();
    m.ns = "mtare_goal_points";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = m.scale.y = m.scale.z = 0.15;
    m.color.r = 1.0f;
    m.color.g = 1.0f;
    m.color.b = 0.0f;
    m.color.a = 0.8f;
    for (const auto & g : frontiers) {
      geometry_msgs::msg::Point p;
      p.x = g.first;
      p.y = g.second;
      p.z = 0.05;
      m.points.push_back(p);
    }
    arr.markers.push_back(m);
    frontier_markers_pub_->publish(arr);
  }

private:
  rclcpp::Clock::SharedPtr clock_;
  double robot_marker_scale_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr coord_map_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr robot_markers_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr frontier_markers_pub_;
};

}  // namespace ros2
}  // namespace cfpa2
