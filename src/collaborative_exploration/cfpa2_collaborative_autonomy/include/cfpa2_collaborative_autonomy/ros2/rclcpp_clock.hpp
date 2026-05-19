// ros2/rclcpp_clock.hpp — ROS 2 adapter that implements core::IClock
// via rclcpp::Clock.
//
// Usage:
//   auto clock = std::make_shared<cfpa2::ros2::RclcppClock>(this->get_clock());
//   // then pass `*clock` to anything taking `core::IClock &`.

#pragma once

#include <memory>

#include "rclcpp/clock.hpp"

#include "cfpa2_collaborative_autonomy/core/clock.hpp"

namespace cfpa2 {
namespace ros2 {

class RclcppClock : public core::IClock
{
public:
  explicit RclcppClock(rclcpp::Clock::SharedPtr clock)
  : clock_(std::move(clock)) {}

  std::uint64_t now_ns() const override
  {
    return static_cast<std::uint64_t>(clock_->now().nanoseconds());
  }

private:
  rclcpp::Clock::SharedPtr clock_;
};

}  // namespace ros2
}  // namespace cfpa2
