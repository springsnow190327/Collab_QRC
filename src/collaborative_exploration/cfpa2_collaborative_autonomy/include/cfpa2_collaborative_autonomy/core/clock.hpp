// core/clock.hpp — abstract clock interface so the algorithm doesn't
// depend on rclcpp::Clock / ros::Time directly.
//
// The ros2 adapter wraps rclcpp::Clock; a future ros1 adapter would
// wrap ros::Time::now() (or sim time when use_sim_time is true).
// Unit tests can also plug in a manual clock that advances when the
// test pushes it.

#pragma once

#include <cstdint>

namespace cfpa2 {
namespace core {

class IClock
{
public:
  virtual ~IClock() = default;
  /// Monotonically non-decreasing nanoseconds since some epoch. Must
  /// reflect sim time when sim time is in use.
  virtual std::uint64_t now_ns() const = 0;
};

}  // namespace core
}  // namespace cfpa2
