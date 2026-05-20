// ros1/roscpp_goal_publisher.hpp — ROS 1 adapter implementing
// core::IGoalPublisher. Wraps ros::Publisher for goal (PointStamped) +
// per-namespace goal marker (Marker) + status (String) emission.
//
// Mirror of ros2/rclcpp_goal_publisher.hpp; the only differences are the
// publisher API (nh.advertise<T> instead of node->create_publisher<T>),
// the message namespaces (drop `::msg::`), and ros::Time::now() for stamps.

#pragma once

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "geometry_msgs/PointStamped.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "visualization_msgs/Marker.h"

#include "cfpa2_collaborative_autonomy/core/output.hpp"

namespace cfpa2 {
namespace ros1 {

class RoscppGoalPublisher : public core::IGoalPublisher
{
public:
  RoscppGoalPublisher(
      ros::NodeHandle & nh,
      const std::vector<std::string> & namespaces,
      const std::string & goal_topic_suffix,
      const std::string & marker_topic_template = "/{ns}/mtare_goal_marker",
      const std::string & status_topic_template = "/{ns}/exploration_status")
  {
    for (const auto & ns : namespaces) {
      goal_pubs_[ns] = nh.advertise<geometry_msgs::PointStamped>(
          "/" + ns + goal_topic_suffix, 10);
      goal_marker_pubs_[ns] = nh.advertise<visualization_msgs::Marker>(
          fill_topic(marker_topic_template, ns), 10);
      status_pubs_[ns] = nh.advertise<std_msgs::String>(
          fill_topic(status_topic_template, ns), 10);
    }
  }

  void publish_goal(
      const std::string & ns,
      core::Goal goal,
      const std::string & frame_id) override
  {
    const auto pit = goal_pubs_.find(ns);
    if (pit == goal_pubs_.end()) return;
    geometry_msgs::PointStamped msg;
    msg.header.frame_id = frame_id;
    msg.header.stamp = ros::Time::now();
    msg.point.x = goal.first;
    msg.point.y = goal.second;
    msg.point.z = 0.0;
    pit->second.publish(msg);
  }

  void publish_goal_marker(
      const std::string & ns,
      core::Goal goal,
      const std::string & frame_id,
      std::array<float, 3> color) override
  {
    const auto pit = goal_marker_pubs_.find(ns);
    if (pit == goal_marker_pubs_.end()) return;
    visualization_msgs::Marker m;
    m.header.frame_id = frame_id;
    m.header.stamp = ros::Time::now();
    m.ns = "mtare_goal";
    m.id = 0;
    m.type = visualization_msgs::Marker::SPHERE;
    m.action = visualization_msgs::Marker::ADD;
    m.pose.position.x = goal.first;
    m.pose.position.y = goal.second;
    m.pose.position.z = 0.1;
    m.pose.orientation.w = 1.0;
    m.scale.x = m.scale.y = m.scale.z = 0.3;
    m.color.r = color[0];
    m.color.g = color[1];
    m.color.b = color[2];
    m.color.a = 1.0f;
    pit->second.publish(m);
  }

  void publish_status(const std::string & ns, const std::string & status) override
  {
    const auto pit = status_pubs_.find(ns);
    if (pit == status_pubs_.end()) return;
    std_msgs::String s;
    s.data = status;
    pit->second.publish(s);
  }

private:
  static std::string fill_topic(const std::string & tmpl, const std::string & ns)
  {
    static const std::string kNeedle = "{ns}";
    std::string out = tmpl;
    auto p = out.find(kNeedle);
    if (p != std::string::npos) out.replace(p, kNeedle.size(), ns);
    return out;
  }

  std::unordered_map<std::string, ros::Publisher> goal_pubs_;
  std::unordered_map<std::string, ros::Publisher> goal_marker_pubs_;
  std::unordered_map<std::string, ros::Publisher> status_pubs_;
};

}  // namespace ros1
}  // namespace cfpa2
