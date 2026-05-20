// ros1/param_facade.hpp — rclcpp-style parameter facade for ROS 1.
//
// The CFPA2 ctor reads ~100 parameters with the rclcpp idiom:
//
//     declare_parameter<double>("publish_rate", 1.0);
//     publish_rate_ = get_parameter("publish_rate").as_double();
//
// Under ROS 2 these are `rclcpp::Node` methods. Under ROS 1 there is no
// node base, so this header provides a mixin (RosParamFacade) that exposes
// the SAME `declare_parameter<T>(name, default)` + `get_parameter(name)`
// surface backed by a private NodeHandle's rosparam store. That lets
// `declare_all_parameters()` / `read_all_parameters()` compile UNCHANGED
// against both ROS distributions — only the mixin differs.
//
// `get_parameter(name)` returns a ParamValue with `.as_double()`,
// `.as_int()`, `.as_bool()`, `.as_string()`, `.as_string_array()`
// accessors mirroring rclcpp::Parameter's typed getters.

#pragma once

#include <string>
#include <vector>

#include "ros/ros.h"

namespace cfpa2 {
namespace ros1 {

/// Typed view of a single declared parameter. Lazily reads from rosparam
/// (falling back to the stored default) on each typed accessor call.
class ParamValue
{
public:
  ParamValue(ros::NodeHandle * pnh, std::string name)
  : pnh_(pnh), name_(std::move(name)) {}

  double as_double() const
  {
    double v = 0.0;
    pnh_->getParam(name_, v);
    return v;
  }

  int64_t as_int() const
  {
    int v = 0;
    pnh_->getParam(name_, v);
    return static_cast<int64_t>(v);
  }

  bool as_bool() const
  {
    bool v = false;
    pnh_->getParam(name_, v);
    return v;
  }

  std::string as_string() const
  {
    std::string v;
    pnh_->getParam(name_, v);
    return v;
  }

  std::vector<std::string> as_string_array() const
  {
    std::vector<std::string> v;
    pnh_->getParam(name_, v);
    return v;
  }

private:
  ros::NodeHandle * pnh_;
  std::string name_;
};

/// Mixin that backs rclcpp-style declare/get parameter calls with a
/// private NodeHandle. The CFPA2Coordinator stores its own `pnh_` and
/// points this facade at it during construction (set_param_handle()).
class RosParamFacade
{
protected:
  /// Declare a parameter: if the operator hasn't set it on the rosparam
  /// server, write the default so subsequent get_parameter() reads it back.
  template <typename T>
  void declare_parameter(const std::string & name, const T & default_value)
  {
    if (param_handle_ != nullptr && !param_handle_->hasParam(name)) {
      param_handle_->setParam(name, default_value);
    }
  }

  ParamValue get_parameter(const std::string & name)
  {
    return ParamValue(param_handle_, name);
  }

  void set_param_handle(ros::NodeHandle * pnh) { param_handle_ = pnh; }

private:
  ros::NodeHandle * param_handle_ = nullptr;
};

}  // namespace ros1
}  // namespace cfpa2
