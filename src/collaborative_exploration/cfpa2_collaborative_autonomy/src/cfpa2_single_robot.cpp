// cfpa2_single_robot.cpp — single-robot CFPA2 subclass.

#include "cfpa2_collaborative_autonomy/cfpa2_single_robot.hpp"

#include <cmath>

#include "cfpa2_collaborative_autonomy/core/logging.hpp"

namespace cfpa2 {

namespace {

CFPA2Coordinator::Options single_robot_opts()
{
  CFPA2Coordinator::Options o;
  o.node_name = "cfpa2_single_robot";
  o.default_namespaces = {"robot"};
  o.startup_label = "cfpa2_single_robot";
  o.planner_desc = "Single-robot CFPA2";
  return o;
}

}  // namespace

CFPA2SingleRobotNode::CFPA2SingleRobotNode(
    const rclcpp::NodeOptions & node_options)
: CFPA2Coordinator(node_options, single_robot_opts())
{
  if (namespaces_.size() != 1) {
    CFPA2_LOG_ERROR(log_facade_,
        "cfpa2_single_robot expects exactly 1 namespace, got %zu",
        namespaces_.size());
  }

  const std::string default_ns = namespaces_.empty() ? "robot" : namespaces_.front();
  declare_parameter<std::string>("robot_namespace", default_ns);
  robot_namespace_ = get_parameter("robot_namespace").as_string();
  if (!robot_namespace_.empty() && robot_namespace_.front() == '/') {
    robot_namespace_.erase(0, 1);
  }
  if (robot_namespace_.empty()) robot_namespace_ = default_ns;

  // (Exploration-status publisher lives behind goal_pub_facade_ —
  // the base class constructs it with `/<ns>/exploration_status` per ns.)

  // Subscribe to peer-coordination blocked-frontiers feed.
  const std::string blocked_topic =
      "/" + robot_namespace_ + "/cfpa2_peer_coordination/blocked_frontiers";
  subs_.push_back(create_subscription<geometry_msgs::msg::PoseArray>(
      blocked_topic, 10,
      [this](const geometry_msgs::msg::PoseArray::SharedPtr msg) {
        on_blocked_frontiers(msg);
      }));
  CFPA2_LOG_INFO(log_facade_, "Subscribed to peer blocked frontiers on %s",
      blocked_topic.c_str());

  // Subscribe to exploration_complete (external pause signal).
  subs_.push_back(create_subscription<std_msgs::msg::String>(
      "/" + robot_namespace_ + "/exploration_complete", 10,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        on_exploration_complete(msg);
      }));

  // Ramp-ascent goal subscriber (optional overlay).
  declare_parameter<bool>("ramp_ascent_enabled", false);
  if (get_parameter("ramp_ascent_enabled").as_bool()) {
    declare_parameter<std::string>("ramp_ascent_goal_topic_suffix", "/ramp_ascent_goal");
    const std::string suffix = get_parameter("ramp_ascent_goal_topic_suffix").as_string();
    const std::string topic = "/" + robot_namespace_ + suffix;
    subs_.push_back(create_subscription<geometry_msgs::msg::PointStamped>(
        topic, 10,
        [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
          on_ramp_ascent_goal(msg, robot_namespace_);
        }));
    CFPA2_LOG_INFO(log_facade_, "[%s] ramp ascent goal <- %s",
        robot_namespace_.c_str(), topic.c_str());
  }
}

bool CFPA2SingleRobotNode::is_goal_peer_claimed(Goal goal)
{
  if (peer_blocked_received_ns_ == 0) return false;  // no peer data yet → fail-open
  const auto now_ns = clock_facade_->now_ns();
  const auto age_ns = now_ns > peer_blocked_received_ns_
      ? now_ns - peer_blocked_received_ns_ : 0;
  if (static_cast<double>(age_ns) > kPeerBlockedTimeoutSec * 1e9) {
    return false;  // stale → fail-open
  }
  const double tol2 = kPeerBlockedMatchTolM * kPeerBlockedMatchTolM;
  for (const auto & b : peer_blocked_frontiers_) {
    const double dx = goal.first - b.first;
    const double dy = goal.second - b.second;
    if (dx * dx + dy * dy <= tol2) return true;
  }
  return false;
}

void CFPA2SingleRobotNode::on_blocked_frontiers(
    const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
  peer_blocked_frontiers_.clear();
  peer_blocked_frontiers_.reserve(msg->poses.size());
  for (const auto & p : msg->poses) {
    peer_blocked_frontiers_.emplace_back(p.position.x, p.position.y);
  }
  peer_blocked_received_ns_ = clock_facade_->now_ns();
}

void CFPA2SingleRobotNode::on_exploration_complete(
    const std_msgs::msg::String::SharedPtr msg)
{
  if (paused_) return;
  paused_ = true;
  const std::string reason = msg->data.empty() ? "unspecified" : msg->data;
  CFPA2_LOG_WARN(log_facade_,
      "[exploration_complete] reason=%s -- pausing CFPA2 goal publication.",
      reason.c_str());
  if (goal_pub_facade_) {
    goal_pub_facade_->publish_status(robot_namespace_, "paused");
  }
  last_status_ = "paused";
}

void CFPA2SingleRobotNode::on_ramp_ascent_goal(
    const geometry_msgs::msg::PointStamped::SharedPtr /*msg*/,
    const std::string & /*ns*/)
{
  // TODO: ramp-ascent override wiring. For now we just receive the topic
  // without taking action; the base ramp_ascent_goal_if_valid() hook
  // returns nullopt so the override path stays disabled.
}

}  // namespace cfpa2
