// core/logging.hpp — printf-style logging helper macros that route
// through the abstract ILogger.
//
// Used by the algorithm body in place of RCLCPP_INFO(get_logger(), ...).
// The Noetic adapter just provides a different ILogger impl; macros
// stay identical (no per-platform sed needed).

#pragma once

#include <cstdarg>
#include <cstdio>
#include <string>

#include "cfpa2_collaborative_autonomy/core/logger.hpp"

namespace cfpa2 {
namespace core {

enum class LogLevel { kInfo, kWarn, kError };

inline void log_format(ILogger & logger, LogLevel level, const char * fmt, ...)
    __attribute__((format(printf, 3, 4)));

inline void log_format(ILogger & logger, LogLevel level, const char * fmt, ...)
{
  char stack_buf[512];
  va_list args;
  va_start(args, fmt);
  va_list args2;
  va_copy(args2, args);
  const int n = std::vsnprintf(stack_buf, sizeof(stack_buf), fmt, args);
  va_end(args);

  std::string msg;
  if (n < 0) {
    msg = "[cfpa2::log_format error]";
  } else if (static_cast<std::size_t>(n) < sizeof(stack_buf)) {
    msg = stack_buf;
  } else {
    msg.resize(static_cast<std::size_t>(n) + 1);
    std::vsnprintf(msg.data(), msg.size(), fmt, args2);
    msg.pop_back();
  }
  va_end(args2);

  switch (level) {
    case LogLevel::kInfo:  logger.info(msg);  break;
    case LogLevel::kWarn:  logger.warn(msg);  break;
    case LogLevel::kError: logger.error(msg); break;
  }
}

}  // namespace core
}  // namespace cfpa2

// printf-style routing macros. Same signature as RCLCPP_INFO except
// the first arg is a `std::shared_ptr<core::ILogger>` (or any pointer
// that dereferences to an ILogger&) instead of an rclcpp::Logger.
#define CFPA2_LOG_INFO(facade, fmt, ...) \
  ::cfpa2::core::log_format(*(facade), ::cfpa2::core::LogLevel::kInfo,  fmt, ##__VA_ARGS__)
#define CFPA2_LOG_WARN(facade, fmt, ...) \
  ::cfpa2::core::log_format(*(facade), ::cfpa2::core::LogLevel::kWarn,  fmt, ##__VA_ARGS__)
#define CFPA2_LOG_ERROR(facade, fmt, ...) \
  ::cfpa2::core::log_format(*(facade), ::cfpa2::core::LogLevel::kError, fmt, ##__VA_ARGS__)
