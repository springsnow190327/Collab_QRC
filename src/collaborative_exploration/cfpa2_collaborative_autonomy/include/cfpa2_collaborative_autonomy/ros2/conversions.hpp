// ros2/conversions.hpp — POD ↔ ROS 2 message converters.
//
// The algorithm (cfpa2::core::*) only sees POD types from
// core/types.hpp. Adapter code calls these converters once at each
// subscription / publish boundary so message-type details stay
// confined to the ros2/ layer.
//
// A future ros1/conversions.hpp will have the same functions taking
// `nav_msgs::OccupancyGrid` etc. — algorithm headers compile unchanged.

#pragma once

#include <cmath>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "cfpa2_collaborative_autonomy/core/types.hpp"

namespace cfpa2 {
namespace ros2 {

inline core::Grid to_core_grid(const nav_msgs::msg::OccupancyGrid & msg)
{
  core::Grid g;
  g.info.width = static_cast<int>(msg.info.width);
  g.info.height = static_cast<int>(msg.info.height);
  g.info.resolution = msg.info.resolution;
  g.info.origin_x = msg.info.origin.position.x;
  g.info.origin_y = msg.info.origin.position.y;
  g.info.frame_id = msg.header.frame_id;
  g.data = msg.data;
  return g;
}

inline nav_msgs::msg::OccupancyGrid to_msg_grid(const core::Grid & g)
{
  nav_msgs::msg::OccupancyGrid msg;
  msg.info.width = static_cast<std::uint32_t>(g.info.width);
  msg.info.height = static_cast<std::uint32_t>(g.info.height);
  msg.info.resolution = static_cast<float>(g.info.resolution);
  msg.info.origin.position.x = g.info.origin_x;
  msg.info.origin.position.y = g.info.origin_y;
  msg.info.origin.orientation.w = 1.0;
  msg.header.frame_id = g.info.frame_id;
  msg.data = g.data;
  return msg;
}

/// Pull 2D pose + linear velocity out of an Odometry. yaw computed from
/// the orientation quaternion via the standard yaw extraction.
inline core::OdomXY to_core_odom(const nav_msgs::msg::Odometry & msg)
{
  const auto & q = msg.pose.pose.orientation;
  // yaw = atan2(2(w*z + x*y), 1 - 2(y² + z²)) — z-axis rotation.
  const double yaw = std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  core::OdomXY o;
  o.x = msg.pose.pose.position.x;
  o.y = msg.pose.pose.position.y;
  o.yaw = yaw;
  o.vx = msg.twist.twist.linear.x;
  o.vy = msg.twist.twist.linear.y;
  return o;
}

inline geometry_msgs::msg::PointStamped to_msg_point_stamped(
    const core::Goal & goal,
    const std::string & frame_id,
    const rclcpp::Time & stamp)
{
  geometry_msgs::msg::PointStamped p;
  p.header.frame_id = frame_id;
  p.header.stamp = stamp;
  p.point.x = goal.first;
  p.point.y = goal.second;
  p.point.z = 0.0;
  return p;
}

}  // namespace ros2
}  // namespace cfpa2
