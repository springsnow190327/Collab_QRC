// ros1/roscpp_visualizer.hpp — ROS 1 adapter implementing core::IVisualizer.
// Wraps ros::Publisher for coordinator map (OccupancyGrid) + robot pose /
// trajectory markers + frontier marker spheres (MarkerArray).
//
// Mirror of ros2/rclcpp_visualizer.hpp. The ROS 2 coordinator-map QoS was
// reliable + transient_local + KeepLast(1); the ROS 1 equivalent is a
// latched advertise (queue size 1, latch=true) so a late-joining RViz
// still receives the most recent map.

#pragma once

#include <cmath>
#include <string>
#include <vector>

#include "geometry_msgs/Point.h"
#include "nav_msgs/OccupancyGrid.h"
#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#include "cfpa2_collaborative_autonomy/core/output.hpp"
#include "cfpa2_collaborative_autonomy/ros1/conversions.hpp"

namespace cfpa2 {
namespace ros1 {

class RoscppVisualizer : public core::IVisualizer
{
public:
  RoscppVisualizer(
      ros::NodeHandle & nh,
      const std::string & coordinator_map_topic,
      const std::string & robot_markers_topic,
      const std::string & frontier_markers_topic,
      double robot_marker_scale)
  : robot_marker_scale_(robot_marker_scale)
  {
    coord_map_pub_ = nh.advertise<nav_msgs::OccupancyGrid>(
        coordinator_map_topic, 1, /*latch=*/true);
    robot_markers_pub_ = nh.advertise<visualization_msgs::MarkerArray>(
        robot_markers_topic, 10);
    frontier_markers_pub_ = nh.advertise<visualization_msgs::MarkerArray>(
        frontier_markers_topic, 10);
  }

  void publish_coordinator_map(const core::Grid & grid) override
  {
    coord_map_pub_.publish(to_msg_grid(grid));
  }

  void publish_robot_markers(
      const std::vector<core::RobotPoseView> & robot_poses,
      const std::vector<core::TrajectoryView> & trajectories) override
  {
    visualization_msgs::MarkerArray arr;
    int id = 0;
    const auto stamp = ros::Time::now();
    for (const auto & p : robot_poses) {
      visualization_msgs::Marker m;
      m.header.frame_id = p.frame_id;
      m.header.stamp = stamp;
      m.ns = "mtare_robot_pose";
      m.id = id++;
      m.type = visualization_msgs::Marker::SPHERE;
      m.action = visualization_msgs::Marker::ADD;
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
      visualization_msgs::Marker m;
      m.header.frame_id = t.frame_id;
      m.header.stamp = stamp;
      m.ns = "mtare_robot_traj";
      m.id = id++;
      m.type = visualization_msgs::Marker::LINE_STRIP;
      m.action = visualization_msgs::Marker::ADD;
      m.scale.x = 0.05;
      m.color.r = t.color[0];
      m.color.g = t.color[1];
      m.color.b = t.color[2];
      m.color.a = 0.6f;
      for (const auto & xy : t.points_xy) {
        geometry_msgs::Point p;
        p.x = xy.first;
        p.y = xy.second;
        p.z = 0.05;
        m.points.push_back(p);
      }
      arr.markers.push_back(m);
    }
    robot_markers_pub_.publish(arr);
  }

  void publish_frontier_markers(
      const std::string & frame_id,
      const std::vector<core::Goal> & frontiers) override
  {
    visualization_msgs::MarkerArray arr;
    visualization_msgs::Marker m;
    m.header.frame_id = frame_id;
    m.header.stamp = ros::Time::now();
    m.ns = "mtare_goal_points";
    m.id = 0;
    m.type = visualization_msgs::Marker::SPHERE_LIST;
    m.action = visualization_msgs::Marker::ADD;
    m.scale.x = m.scale.y = m.scale.z = 0.15;
    m.color.r = 1.0f;
    m.color.g = 1.0f;
    m.color.b = 0.0f;
    m.color.a = 0.8f;
    for (const auto & g : frontiers) {
      geometry_msgs::Point p;
      p.x = g.first;
      p.y = g.second;
      p.z = 0.05;
      m.points.push_back(p);
    }
    arr.markers.push_back(m);
    frontier_markers_pub_.publish(arr);
  }

private:
  double robot_marker_scale_;
  ros::Publisher coord_map_pub_;
  ros::Publisher robot_markers_pub_;
  ros::Publisher frontier_markers_pub_;
};

}  // namespace ros1
}  // namespace cfpa2
