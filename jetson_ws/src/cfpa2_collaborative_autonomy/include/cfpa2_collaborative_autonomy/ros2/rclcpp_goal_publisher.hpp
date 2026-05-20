// ros2/rclcpp_goal_publisher.hpp — ROS 2 adapter implementing
// core::IGoalPublisher. Wraps rclcpp::Publisher<PointStamped/Marker/String>
// for goal + per-namespace goal marker + status emission.

#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "cfpa2_collaborative_autonomy/core/output.hpp"

namespace cfpa2 {
namespace ros2 {

class RclcppGoalPublisher : public core::IGoalPublisher
{
public:
  RclcppGoalPublisher(
      rclcpp::Node * node,
      const std::vector<std::string> & namespaces,
      const std::string & goal_topic_suffix,
      const std::string & marker_topic_template = "/{ns}/mtare_goal_marker",
      const std::string & status_topic_template = "/{ns}/exploration_status")
  : clock_(node->get_clock())
  {
    for (const auto & ns : namespaces) {
      goal_pubs_[ns] = node->create_publisher<geometry_msgs::msg::PointStamped>(
          "/" + ns + goal_topic_suffix, 10);
      goal_marker_pubs_[ns] = node->create_publisher<visualization_msgs::msg::Marker>(
          fill_topic(marker_topic_template, ns), 10);
      status_pubs_[ns] = node->create_publisher<std_msgs::msg::String>(
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
    geometry_msgs::msg::PointStamped msg;
    msg.header.frame_id = frame_id;
    msg.header.stamp = clock_->now();
    msg.point.x = goal.first;
    msg.point.y = goal.second;
    msg.point.z = 0.0;
    pit->second->publish(msg);
  }

  void publish_goal_marker(
      const std::string & ns,
      core::Goal goal,
      const std::string & frame_id,
      std::array<float, 3> color) override
  {
    const auto pit = goal_marker_pubs_.find(ns);
    if (pit == goal_marker_pubs_.end()) return;
    visualization_msgs::msg::Marker m;
    m.header.frame_id = frame_id;
    m.header.stamp = clock_->now();
    m.ns = "mtare_goal";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x = goal.first;
    m.pose.position.y = goal.second;
    m.pose.position.z = 0.1;
    m.pose.orientation.w = 1.0;
    m.scale.x = m.scale.y = m.scale.z = 0.3;
    m.color.r = color[0];
    m.color.g = color[1];
    m.color.b = color[2];
    m.color.a = 1.0f;
    pit->second->publish(m);
  }

  void publish_status(const std::string & ns, const std::string & status) override
  {
    const auto pit = status_pubs_.find(ns);
    if (pit == status_pubs_.end()) return;
    std_msgs::msg::String s;
    s.data = status;
    pit->second->publish(s);
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

  rclcpp::Clock::SharedPtr clock_;
  std::unordered_map<std::string, rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr> goal_pubs_;
  std::unordered_map<std::string, rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr> goal_marker_pubs_;
  std::unordered_map<std::string, rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> status_pubs_;
};

}  // namespace ros2
}  // namespace cfpa2
