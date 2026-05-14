// filter_chain_runner — apply a configurable grid_map filter chain to an
// incoming GridMap topic, publish the filtered result.
//
// Phase 4 of plans/2026-05-14-trav-grid-rewrite.md.
//
// Inputs:
//   - parameter `input_topic`  (default "elevation_map_raw")
//   - parameter `output_topic` (default "elevation_map_filtered")
//   - parameter `filter_chain_param` (default "filter_chain") — name of the
//     parameter list this node loads its chain from. The chain YAML must be
//     loaded under that name (e.g. via --params-file).
//
// The filter chain runs synchronously inside the input callback. With our
// 30x30 m @ 0.10 m grid (~90k cells) and a 5-stage chain, expect ~10 ms
// per pass on a 5090 — well inside the 5 Hz emap publish budget.

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <filters/filter_chain.hpp>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_ros/grid_map_ros.hpp>

namespace trav_cost_filters
{

class FilterChainRunner : public rclcpp::Node
{
public:
  FilterChainRunner()
  : rclcpp::Node("filter_chain_runner"),
    filter_chain_("grid_map::GridMap")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "elevation_map_raw");
    output_topic_ = declare_parameter<std::string>("output_topic", "elevation_map_filtered");
    // Default "filters" matches grid_map_demos convention; FilterChain reads
    // the parameter list under <node_name>.<filter_chain_param>.*
    filter_chain_param_ = declare_parameter<std::string>("filter_chain_param", "filters");

    if (!filter_chain_.configure(
        filter_chain_param_,
        this->get_node_logging_interface(),
        this->get_node_parameters_interface()))
    {
      RCLCPP_ERROR(
        get_logger(),
        "Could not configure filter chain from parameter '%s'. "
        "Aborting.", filter_chain_param_.c_str());
      throw std::runtime_error("filter_chain configure failed");
    }

    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.reliable();

    pub_ = create_publisher<grid_map_msgs::msg::GridMap>(output_topic_, qos);
    sub_ = create_subscription<grid_map_msgs::msg::GridMap>(
      input_topic_, qos,
      std::bind(&FilterChainRunner::onMap, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "filter_chain_runner started. in=%s out=%s chain_param=%s",
      input_topic_.c_str(), output_topic_.c_str(), filter_chain_param_.c_str());
  }

private:
  void onMap(const grid_map_msgs::msg::GridMap::ConstSharedPtr msg)
  {
    grid_map::GridMap in_map;
    grid_map::GridMapRosConverter::fromMessage(*msg, in_map);

    grid_map::GridMap out_map;
    if (!filter_chain_.update(in_map, out_map)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "filter_chain.update failed; skipping frame.");
      return;
    }

    auto out_msg = grid_map::GridMapRosConverter::toMessage(out_map);
    out_msg->header.stamp = msg->header.stamp;
    out_msg->header.frame_id = msg->header.frame_id;
    pub_->publish(std::move(out_msg));

    if (++count_ % 20 == 1) {
      std::string names;
      for (const auto & l : out_map.getLayers()) {
        if (!names.empty()) names += ",";
        names += l;
      }
      RCLCPP_INFO(
        get_logger(),
        "filtered frame #%u — in layers=%zu out layers=%zu [%s]",
        count_, in_map.getLayers().size(), out_map.getLayers().size(),
        names.c_str());
    }
  }

  filters::FilterChain<grid_map::GridMap> filter_chain_;
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr sub_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr pub_;
  std::string input_topic_;
  std::string output_topic_;
  std::string filter_chain_param_;
  unsigned int count_{0};
};

}  // namespace trav_cost_filters

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<trav_cost_filters::FilterChainRunner>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("filter_chain_runner"), "fatal: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
