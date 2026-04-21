#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <cmath>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

class TarePlannerNode : public rclcpp::Node
{
public:
  TarePlannerNode()
  : Node("tare_planner_node")
  {
    this->declare_parameter<std::vector<std::string>>("namespaces", {"robot_a", "robot_b"});
    this->declare_parameter<std::string>("input_topic_suffix", "/way_point_seed");
    this->declare_parameter<std::string>("output_topic_suffix", "/way_point_tare");
    this->declare_parameter<std::string>("planner_mode_suffix", "/planner_mode");
    this->declare_parameter<double>("output_rate_hz", 5.0);
    this->declare_parameter<bool>("hold_last_output", true);
    this->declare_parameter<bool>("stamp_now", true);

    namespaces_ = this->get_parameter("namespaces").as_string_array();
    input_topic_suffix_ = this->get_parameter("input_topic_suffix").as_string();
    output_topic_suffix_ = this->get_parameter("output_topic_suffix").as_string();
    planner_mode_suffix_ = this->get_parameter("planner_mode_suffix").as_string();
    output_rate_hz_ = std::max(1.0, this->get_parameter("output_rate_hz").as_double());
    hold_last_output_ = this->get_parameter("hold_last_output").as_bool();
    stamp_now_ = this->get_parameter("stamp_now").as_bool();

    for (const auto & ns : namespaces_) {
      auto & state = robots_[ns];
      state.sub = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/" + ns + input_topic_suffix_,
        10,
        [this, ns](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
          if (!std::isfinite(msg->point.x) || !std::isfinite(msg->point.y) || !std::isfinite(msg->point.z)) {
            return;
          }
          auto it = robots_.find(ns);
          if (it == robots_.end()) {
            return;
          }
          it->second.latest = *msg;
        });
      state.pub = this->create_publisher<geometry_msgs::msg::PointStamped>(
        "/" + ns + output_topic_suffix_, 10);
      state.mode_pub = this->create_publisher<std_msgs::msg::String>(
        "/" + ns + planner_mode_suffix_, 10);
    }

    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / output_rate_hz_),
      [this]() { this->on_tick(); });

    RCLCPP_INFO(
      this->get_logger(),
      "tare_planner_node started | provider=%s namespaces=%zu input_suffix=%s output_suffix=%s",
      TARE_ORTOOLS_PROVIDER_STR,
      namespaces_.size(),
      input_topic_suffix_.c_str(),
      output_topic_suffix_.c_str());
  }

private:
  struct RobotState
  {
    std::optional<geometry_msgs::msg::PointStamped> latest{};
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub{};
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub{};
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub{};
  };

  void on_tick()
  {
    const auto now = this->now();
    for (auto & [ns, state] : robots_) {
      if (!state.latest.has_value() && !hold_last_output_) {
        continue;
      }
      if (!state.latest.has_value()) {
        continue;
      }
      auto msg = state.latest.value();
      if (stamp_now_) {
        msg.header.stamp = now;
      }
      state.pub->publish(msg);

      std_msgs::msg::String mode{};
      mode.data = "LOCAL_TARE";
      state.mode_pub->publish(mode);
    }
  }

  std::vector<std::string> namespaces_{};
  std::string input_topic_suffix_{};
  std::string output_topic_suffix_{};
  std::string planner_mode_suffix_{};
  double output_rate_hz_{5.0};
  bool hold_last_output_{true};
  bool stamp_now_{true};
  std::unordered_map<std::string, RobotState> robots_{};
  rclcpp::TimerBase::SharedPtr timer_{};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TarePlannerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
