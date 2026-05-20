// ros2/rclcpp_logger.hpp — ROS 2 adapter that implements core::ILogger
// via the RCLCPP_* macros on a stored rclcpp::Logger.
//
// Usage:
//   auto logger = std::make_shared<cfpa2::ros2::RclcppLogger>(this->get_logger());
//   // pass `*logger` to anything taking `core::ILogger &`.

#pragma once

#include <string>

#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"

#include "cfpa2_collaborative_autonomy/core/logger.hpp"

namespace cfpa2 {
namespace ros2 {

class RclcppLogger : public core::ILogger
{
public:
  explicit RclcppLogger(rclcpp::Logger logger) : logger_(std::move(logger)) {}

  void info(const std::string & msg) override
  {
    RCLCPP_INFO(logger_, "%s", msg.c_str());
  }

  void warn(const std::string & msg) override
  {
    RCLCPP_WARN(logger_, "%s", msg.c_str());
  }

  void error(const std::string & msg) override
  {
    RCLCPP_ERROR(logger_, "%s", msg.c_str());
  }

private:
  rclcpp::Logger logger_;
};

}  // namespace ros2
}  // namespace cfpa2
