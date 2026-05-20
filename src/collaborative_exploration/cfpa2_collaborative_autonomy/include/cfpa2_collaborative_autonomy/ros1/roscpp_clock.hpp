// ros1/roscpp_clock.hpp — ROS 1 adapter that implements core::IClock
// via ros::Time.
//
// Usage:
//   auto clock = std::make_shared<cfpa2::ros1::RoscppClock>();
//   // then pass `*clock` to anything taking `core::IClock &`.
//
// ros::Time::now() respects sim time automatically when /use_sim_time is
// true and a /clock publisher is present, so no extra wiring is needed to
// match the ROS 2 RclcppClock behaviour.

#pragma once

#include "ros/time.h"

#include "cfpa2_collaborative_autonomy/core/clock.hpp"

namespace cfpa2 {
namespace ros1 {

class RoscppClock : public core::IClock
{
public:
  RoscppClock() = default;

  std::uint64_t now_ns() const override
  {
    return static_cast<std::uint64_t>(ros::Time::now().toNSec());
  }
};

}  // namespace ros1
}  // namespace cfpa2
