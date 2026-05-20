// Copyright 2026 Collab_QRC
// SPDX-License-Identifier: Apache-2.0
//
// ROS 1 reimplementation of nav2_mppi_controller::ParametersHandler.
//
// Public API surface kept identical to the Humble original so all ported
// critics, models, and the optimizer can include this header unchanged:
//
//   auto getter = params_handler.getParamGetter("PathAlignCritic");
//   getter(weight_, "cost_weight", 14.0f);
//   getter(consider_footprint_, "consider_footprint", true);
//
// Body differences vs Nav2:
//   - Reads rosparam server through a ros::NodeHandle scoped to the
//     controller's private namespace (e.g. "/move_base/FollowPath").
//   - No dynamic reconfigure: all params resolved at load time. The
//     setDynamicParamCallback / addPostCallback / addPreCallback hooks are
//     accepted but no-op'd, since the critics' "dynamic" updates only fire
//     in response to ROS 2 SetParameters service calls that don't exist on
//     Noetic. Equivalence test compares the static-load case only.
//   - The "." separator used by Nav2 to nest YAML keys becomes "/" so that
//     rosparam YAML loaded under <ns>/PathAlignCritic/cost_weight resolves
//     identically.

#ifndef NAV_ALGO_CORE__MPPI__TOOLS__PARAMETERS_HANDLER_HPP_
#define NAV_ALGO_CORE__MPPI__TOOLS__PARAMETERS_HANDLER_HPP_

#include <algorithm>
#include <functional>
#include <mutex>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

#include "nav_algo_core/compat.hpp"

namespace mppi
{

enum class ParameterType { Dynamic, Static };

class ParametersHandler
{
public:
  using get_param_func_t = void (const rclcpp::Parameter & param);
  using post_callback_t = void ();
  using pre_callback_t  = void ();

  ParametersHandler() = default;
  explicit ParametersHandler(const ros::NodeHandle & parent_nh)
  : nh_(parent_nh) {}

  // Legacy LifecycleNode-based constructor kept so that existing call sites
  // that construct ParametersHandler(weak_ptr) compile. The weak_ptr is
  // ignored; the rosparam read goes through nh_ which the controller plugin
  // explicitly sets before any critic calls getParamGetter().
  explicit ParametersHandler(const rclcpp_lifecycle::LifecycleNode::WeakPtr &)
  {}

  void setNodeHandle(const ros::NodeHandle & nh) { nh_ = nh; }

  void start() {}  // no dynamic param infrastructure under ROS 1

  rcl_interfaces::msg::SetParametersResult dynamicParamsCallback(
    std::vector<rclcpp::Parameter>)
  {
    return {};
  }

  inline auto getParamGetter(const std::string & ns);

  template<typename T> void addPostCallback(T &&) {}
  template<typename T> void addPreCallback(T &&) {}
  template<typename T> void setDynamicParamCallback(T &, const std::string &) {}
  template<typename T>
  void addDynamicParamCallback(const std::string &, T &&) {}

  std::mutex * getLock() { return &parameters_change_mutex_; }

protected:
  template<typename SettingT, typename ParamT>
  void getParam(
    SettingT & setting, const std::string & name, ParamT default_value,
    ParameterType /*param_type*/ = ParameterType::Dynamic);

  std::mutex parameters_change_mutex_;
  ros::NodeHandle nh_;
};

inline auto ParametersHandler::getParamGetter(const std::string & ns)
{
  return [this, ns](
    auto & setting, const std::string & name, auto default_value,
    ParameterType param_type = ParameterType::Dynamic) {
      getParam(
        setting,
        ns.empty() ? name : ns + "." + name,
        std::move(default_value),
        param_type);
    };
}

namespace detail {

// Translate Nav2's "ns.subns.key" → "ns/subns/key" so YAML loaded into
// rosparam under a single sub-namespace resolves correctly.
inline std::string rosify(std::string s)
{
  std::replace(s.begin(), s.end(), '.', '/');
  return s;
}

// ros::NodeHandle::param<T> covers bool/int/double/string. For other types
// (vector<int>/<double>/<string>/<bool>) NodeHandle::getParam is used.
template<typename T>
inline bool getRosParam(
  const ros::NodeHandle & nh, const std::string & name, T & out, T def)
{
  return nh.param<T>(name, out, def);
}
template<>
inline bool getRosParam<std::vector<double>>(
  const ros::NodeHandle & nh, const std::string & name,
  std::vector<double> & out, std::vector<double> def)
{
  if (!nh.getParam(name, out)) { out = def; return false; }
  return true;
}
template<>
inline bool getRosParam<std::vector<int>>(
  const ros::NodeHandle & nh, const std::string & name,
  std::vector<int> & out, std::vector<int> def)
{
  if (!nh.getParam(name, out)) { out = def; return false; }
  return true;
}
template<>
inline bool getRosParam<std::vector<std::string>>(
  const ros::NodeHandle & nh, const std::string & name,
  std::vector<std::string> & out, std::vector<std::string> def)
{
  if (!nh.getParam(name, out)) { out = def; return false; }
  return true;
}
template<>
inline bool getRosParam<std::vector<bool>>(
  const ros::NodeHandle & nh, const std::string & name,
  std::vector<bool> & out, std::vector<bool> def)
{
  // ros::NodeHandle::getParam doesn't accept vector<bool>; promote to int.
  std::vector<int> tmp;
  if (!nh.getParam(name, tmp)) { out = def; return false; }
  out.clear();
  for (auto v : tmp) { out.push_back(v != 0); }
  return true;
}

}  // namespace detail

template<typename SettingT, typename ParamT>
void ParametersHandler::getParam(
  SettingT & setting, const std::string & name, ParamT default_value,
  ParameterType /*param_type*/)
{
  ParamT got;
  detail::getRosParam<ParamT>(
    nh_, detail::rosify(name), got, std::move(default_value));
  setting = static_cast<SettingT>(got);
}

}  // namespace mppi

#endif  // NAV_ALGO_CORE__MPPI__TOOLS__PARAMETERS_HANDLER_HPP_
