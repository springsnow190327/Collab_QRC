// core/logger.hpp — abstract logger interface so the algorithm doesn't
// depend on RCLCPP_INFO / ROS_INFO directly.
//
// ros2 adapter wraps RCLCPP_INFO(get_logger(), ...).
// A future ros1 adapter wraps ROS_INFO_NAMED(...).
//
// Algorithm code calls one of the three severity methods with a
// pre-formatted std::string. printf-style formatting happens at the
// call site (use `std::ostringstream` or `fmt::format` if available).

#pragma once

#include <string>

namespace cfpa2 {
namespace core {

class ILogger
{
public:
  virtual ~ILogger() = default;
  virtual void info(const std::string & msg) = 0;
  virtual void warn(const std::string & msg) = 0;
  virtual void error(const std::string & msg) = 0;
};

}  // namespace core
}  // namespace cfpa2
