// ros1/roscpp_logger.hpp — ROS 1 adapter that implements core::ILogger
// via the ROS_* macros.
//
// Usage:
//   auto logger = std::make_shared<cfpa2::ros1::RoscppLogger>();
//   // pass `*logger` to anything taking `core::ILogger &`.

#pragma once

#include <string>

#include "ros/console.h"

#include "cfpa2_collaborative_autonomy/core/logger.hpp"

namespace cfpa2 {
namespace ros1 {

class RoscppLogger : public core::ILogger
{
public:
  RoscppLogger() = default;

  void info(const std::string & msg) override
  {
    ROS_INFO("%s", msg.c_str());
  }

  void warn(const std::string & msg) override
  {
    ROS_WARN("%s", msg.c_str());
  }

  void error(const std::string & msg) override
  {
    ROS_ERROR("%s", msg.c_str());
  }
};

}  // namespace ros1
}  // namespace cfpa2
