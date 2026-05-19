// Copyright 2026 Collab_QRC
// SPDX-License-Identifier: Apache-2.0
//
// nav_algo_core/compat.hpp
//
// ROS 1 Noetic compatibility shim for ported Nav2 Humble algorithm code.
// Goal: let the bulk of Nav2 SmacPlannerLattice + MPPIController source files
// compile under Noetic with minimal edits. This header maps:
//   - rclcpp/RCLCPP_* logging macros → ROS_* macros
//   - rclcpp_lifecycle::LifecycleNode → ros::NodeHandle (typedef only; param
//     access goes through ParametersHandler facade, not the node directly)
//   - nav2_costmap_2d::* → costmap_2d::*       (namespace alias)
//   - geometry_msgs::msg::X → geometry_msgs::X (type aliases inside ::msg ns)
//   - nav_msgs::msg::X       → nav_msgs::X
//   - visualization_msgs::msg::X → visualization_msgs::X
//   - rclcpp::Parameter / ParameterValue / ParameterType / Logger → stubs
//
// The intent is mechanical: ported Nav2 sources include this header (auto-
// inserted at the top by the porting script) and otherwise compile unchanged.
// Anything semantically different from Nav2 (dynamic-reconfigure, lifecycle
// transitions, message-stamped time) is either no-op'd or re-routed.

#ifndef NAV_ALGO_CORE__COMPAT_HPP_
#define NAV_ALGO_CORE__COMPAT_HPP_

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

// ───────────────────────────────────────────────────────────────────────────
// 1. ROS 1 base + logging
// ───────────────────────────────────────────────────────────────────────────
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/time.h>
#include <ros/node_handle.h>

// rclcpp logging macros take a logger handle as their first argument;
// ROS_INFO et al. do not. The handle is dropped, format is passed through.
#define RCLCPP_INFO(logger,  ...) ROS_INFO(__VA_ARGS__)
#define RCLCPP_WARN(logger,  ...) ROS_WARN(__VA_ARGS__)
#define RCLCPP_ERROR(logger, ...) ROS_ERROR(__VA_ARGS__)
#define RCLCPP_DEBUG(logger, ...) ROS_DEBUG(__VA_ARGS__)
#define RCLCPP_FATAL(logger, ...) ROS_FATAL(__VA_ARGS__)
#define RCLCPP_INFO_STREAM(logger,  s) ROS_INFO_STREAM(s)
#define RCLCPP_WARN_STREAM(logger,  s) ROS_WARN_STREAM(s)
#define RCLCPP_ERROR_STREAM(logger, s) ROS_ERROR_STREAM(s)
#define RCLCPP_DEBUG_STREAM(logger, s) ROS_DEBUG_STREAM(s)

// Throttled variants: ROS 2 takes (logger, clock, period_ms, fmt). ROS 1's
// ROS_*_THROTTLE takes (period_seconds, fmt). Drop logger+clock, divide ms→s.
#define RCLCPP_INFO_THROTTLE(logger,  clock, period_ms, ...) \
  ROS_INFO_THROTTLE((period_ms) / 1000.0,  __VA_ARGS__)
#define RCLCPP_WARN_THROTTLE(logger,  clock, period_ms, ...) \
  ROS_WARN_THROTTLE((period_ms) / 1000.0,  __VA_ARGS__)
#define RCLCPP_ERROR_THROTTLE(logger, clock, period_ms, ...) \
  ROS_ERROR_THROTTLE((period_ms) / 1000.0, __VA_ARGS__)
#define RCLCPP_DEBUG_THROTTLE(logger, clock, period_ms, ...) \
  ROS_DEBUG_THROTTLE((period_ms) / 1000.0, __VA_ARGS__)

// ───────────────────────────────────────────────────────────────────────────
// 2. Message type & header path aliases (ROS 2 *.hpp under ::msg → ROS 1 *.h)
// ───────────────────────────────────────────────────────────────────────────
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Header.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
// (builtin_interfaces is a ROS 2-only pkg; deliberately omitted under ROS 1.)

namespace geometry_msgs { namespace msg {
  using Pose          = ::geometry_msgs::Pose;
  using PoseStamped   = ::geometry_msgs::PoseStamped;
  using PoseArray     = ::geometry_msgs::PoseArray;
  using Point         = ::geometry_msgs::Point;
  using Point32       = ::geometry_msgs::Point32;
  using Polygon       = ::geometry_msgs::Polygon;
  using Quaternion    = ::geometry_msgs::Quaternion;
  using Twist         = ::geometry_msgs::Twist;
  using TwistStamped  = ::geometry_msgs::TwistStamped;
  using Vector3       = ::geometry_msgs::Vector3;
}}
namespace nav_msgs { namespace msg {
  using Path           = ::nav_msgs::Path;
  using OccupancyGrid  = ::nav_msgs::OccupancyGrid;
}}
namespace std_msgs { namespace msg {
  using Header    = ::std_msgs::Header;
  using ColorRGBA = ::std_msgs::ColorRGBA;
}}

// builtin_interfaces::msg::Time is ROS 2's standard time message. ROS 1 has
// no such package; map onto ros::Time for any use sites that just store /
// compare stamped time (Nav2's optimizer header has a single such field).
namespace builtin_interfaces { namespace msg {
  using Time = ros::Time;
}}
namespace visualization_msgs { namespace msg {
  using Marker      = ::visualization_msgs::Marker;
  using MarkerArray = ::visualization_msgs::MarkerArray;
}}

// ───────────────────────────────────────────────────────────────────────────
// 3. nav2_costmap_2d → costmap_2d (Noetic) namespace alias
//    Cost-value constants and the Costmap2D / Costmap2DROS classes are
//    semantically equivalent between Humble nav2_costmap_2d and Noetic
//    costmap_2d. Footprint collision checker (Nav2-only) is re-implemented
//    inline below; same for INSCRIBED_INFLATED_OBSTACLE et al. which are
//    defined identically in Noetic <costmap_2d/cost_values.h>.
// ───────────────────────────────────────────────────────────────────────────
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d_publisher.h>
#include <costmap_2d/cost_values.h>
#include <costmap_2d/inflation_layer.h>
#include <costmap_2d/footprint.h>

#include <costmap_2d/layer.h>

namespace nav2_costmap_2d {
  using ::costmap_2d::Costmap2D;
  using ::costmap_2d::Costmap2DROS;
  using ::costmap_2d::Costmap2DPublisher;
  using ::costmap_2d::InflationLayer;
  using ::costmap_2d::Layer;
  using ::costmap_2d::FREE_SPACE;
  using ::costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
  using ::costmap_2d::LETHAL_OBSTACLE;
  using ::costmap_2d::NO_INFORMATION;

  // Nav2 typedef for a footprint polygon. ROS 1 just uses raw vector.
  using Footprint = std::vector<geometry_msgs::Point>;

  static constexpr uint8_t MAX_NON_OBSTACLE = 252;

  // Nav2 sentinel for "no speed limit set" — propagated through controllers.
  static constexpr double NO_SPEED_LIMIT = 0.0;

  // Nav2-only template class. Re-implemented inline below — it does
  // footprint-in-costmap collision checking via supercover line rasterisation;
  // ~150 LOC lifted from nav2_costmap_2d/footprint_collision_checker.hpp.
  template <typename CostmapT>
  class FootprintCollisionChecker
  {
  public:
    FootprintCollisionChecker() : costmap_(nullptr) {}
    explicit FootprintCollisionChecker(CostmapT costmap) : costmap_(costmap) {}

    void setCostmap(CostmapT costmap) { costmap_ = costmap; }
    CostmapT getCostmap() { return costmap_; }

    // Returns max cell cost intersected by the footprint polygon edges.
    double footprintCost(const Footprint & footprint)
    {
      if (!costmap_) {return static_cast<double>(LETHAL_OBSTACLE);}
      unsigned int mx, my;
      double footprint_cost = 0.0;
      for (size_t i = 0; i < footprint.size() - 1; ++i) {
        if (!costmap_->worldToMap(footprint[i].x, footprint[i].y, mx, my)) {
          return static_cast<double>(LETHAL_OBSTACLE);
        }
        unsigned int x0 = mx, y0 = my;
        if (!costmap_->worldToMap(footprint[i+1].x, footprint[i+1].y, mx, my)) {
          return static_cast<double>(LETHAL_OBSTACLE);
        }
        footprint_cost = std::max(footprint_cost, lineCost(x0, mx, y0, my));
        if (footprint_cost == static_cast<double>(LETHAL_OBSTACLE)) {return footprint_cost;}
      }
      // Closing segment.
      if (!costmap_->worldToMap(footprint.back().x, footprint.back().y, mx, my)) {
        return static_cast<double>(LETHAL_OBSTACLE);
      }
      unsigned int x0 = mx, y0 = my;
      if (!costmap_->worldToMap(footprint.front().x, footprint.front().y, mx, my)) {
        return static_cast<double>(LETHAL_OBSTACLE);
      }
      return std::max(footprint_cost, lineCost(x0, mx, y0, my));
    }

    double lineCost(int x0, int x1, int y0, int y1) const
    {
      double line_cost = 0.0;
      int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
      int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
      int err = dx + dy;
      while (true) {
        double pt_cost = pointCost(x0, y0);
        if (pt_cost == static_cast<double>(LETHAL_OBSTACLE)) {return pt_cost;}
        line_cost = std::max(line_cost, pt_cost);
        if (x0 == x1 && y0 == y1) {break;}
        int e2 = 2 * err;
        if (e2 >= dy) {err += dy; x0 += sx;}
        if (e2 <= dx) {err += dx; y0 += sy;}
      }
      return line_cost;
    }

    double pointCost(int x, int y) const
    {
      if (!costmap_) {return static_cast<double>(LETHAL_OBSTACLE);}
      if (x < 0 || y < 0 ||
          x >= static_cast<int>(costmap_->getSizeInCellsX()) ||
          y >= static_cast<int>(costmap_->getSizeInCellsY()))
      {
        return static_cast<double>(LETHAL_OBSTACLE);
      }
      return static_cast<double>(costmap_->getCost(x, y));
    }

    // Passthrough to underlying costmap so Nav2 callers that go through the
    // checker (e.g. obstacles_critic at line 222) keep compiling.
    bool worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my)
    {
      return costmap_ ? costmap_->worldToMap(wx, wy, mx, my) : false;
    }

    // Footprint-at-pose convenience used by GridCollisionChecker.
    double footprintCostAtPose(
      double x, double y, double theta, const Footprint & footprint)
    {
      double cos_th = std::cos(theta), sin_th = std::sin(theta);
      Footprint oriented = footprint;
      for (auto & p : oriented) {
        double nx = x + p.x * cos_th - p.y * sin_th;
        double ny = y + p.x * sin_th + p.y * cos_th;
        p.x = nx; p.y = ny;
      }
      return footprintCost(oriented);
    }

  protected:
    CostmapT costmap_;
  };
}  // namespace nav2_costmap_2d

// ───────────────────────────────────────────────────────────────────────────
// 4. rclcpp / rclcpp_lifecycle stubs sufficient for header includes
// ───────────────────────────────────────────────────────────────────────────
namespace rclcpp {

class Logger {
 public:
  explicit Logger(const std::string & name = "") : name_(name) {}
  const std::string & get_name() const { return name_; }
 private:
  std::string name_;
};

inline Logger get_logger(const std::string & name) { return Logger(name); }

// Stand-in for rclcpp::Time / Duration used in optimizer header only.
class Time {
 public:
  Time() : t_(ros::Time(0)) {}
  Time(int32_t sec, uint32_t nsec) : t_(ros::Time(sec, nsec)) {}
  explicit Time(double seconds) : t_(ros::Time(seconds)) {}
  // Intentionally non-explicit so rclcpp::Time arguments can be assigned
  // into ros::Time fields (e.g. msg.header.stamp = rclcpp::Time(0, 0)).
  Time(const ros::Time & t) : t_(t) {}
  double seconds() const { return t_.toSec(); }
  uint64_t nanoseconds() const { return t_.toNSec(); }
  operator ros::Time() const { return t_; }
  ros::Time as_ros1() const { return t_; }
 private:
  ros::Time t_;
};

class Duration {
 public:
  Duration() : d_(ros::Duration(0)) {}
  Duration(int32_t sec, uint32_t nsec) : d_(ros::Duration(sec, nsec)) {}
  explicit Duration(double seconds) : d_(ros::Duration(seconds)) {}
  // Nav2's smoother does `Duration dt = std::chrono::duration<double>(...);`
  template <class Rep, class Period>
  Duration(const std::chrono::duration<Rep, Period> & d)
  : d_(ros::Duration(std::chrono::duration<double>(d).count())) {}
  static Duration from_seconds(double s) { return Duration(s); }
  double seconds() const { return d_.toSec(); }
  bool operator>(const Duration & rhs) const { return d_ > rhs.d_; }
  bool operator<(const Duration & rhs) const { return d_ < rhs.d_; }
 private:
  ros::Duration d_;
};

inline Duration operator-(const Time & a, const Time & b)
{
  return Duration(a.seconds() - b.seconds());
}

// rclcpp::Clock used by Nav2's collision_checker.hpp to debounce throttled
// log lines. Wraps ros::Time::now(); throttle is by wall clock.
class Clock {
 public:
  using SharedPtr = std::shared_ptr<Clock>;
  Time now() const { return Time(ros::Time::now()); }
};

// Stand-in for rclcpp::Parameter — only used by ParametersHandler internals,
// which are fully re-implemented; this class is kept as a defined-but-unused
// type so that any vestigial template instantiations still compile.
class Parameter {
 public:
  Parameter() = default;
  Parameter(const std::string & name, bool        v) : name_(name), bv_(v) {}
  Parameter(const std::string & name, int64_t     v) : name_(name), iv_(v) {}
  Parameter(const std::string & name, double      v) : name_(name), dv_(v) {}
  Parameter(const std::string & name, std::string v) : name_(name), sv_(std::move(v)) {}
  bool        as_bool()   const { return bv_; }
  int64_t     as_int()    const { return iv_; }
  double      as_double() const { return dv_; }
  std::string as_string() const { return sv_; }
  std::vector<bool>        as_bool_array()    const { return {}; }
  std::vector<int64_t>     as_integer_array() const { return {}; }
  std::vector<double>      as_double_array()  const { return {}; }
  std::vector<std::string> as_string_array()  const { return {}; }
  const std::string & get_name() const { return name_; }
 private:
  std::string name_;
  bool        bv_{};
  int64_t     iv_{};
  double      dv_{};
  std::string sv_;
};
inline std::string to_string(const Parameter & p) { return p.get_name(); }

class ParameterValue {
 public:
  template <class T>
  explicit ParameterValue(T) {}
};

}  // namespace rclcpp

// In rclcpp the param handle type lives under node_interfaces. Stub.
namespace rclcpp { namespace node_interfaces {
  struct OnSetParametersCallbackHandle {
    using SharedPtr = std::shared_ptr<OnSetParametersCallbackHandle>;
  };
}}

// rcl_interfaces::msg::SetParametersResult is referenced by Nav2's dynamic
// parameter callback signature. Stubbed to a struct with a single field.
namespace rcl_interfaces { namespace msg {
  struct SetParametersResult {
    bool   successful{true};
    std::string reason;
  };
}}

namespace rclcpp_lifecycle {
// LifecycleNode under ROS 1 wraps a ros::NodeHandle scoped to the plugin's
// private namespace. The wrapper exposes the small slice of the rclcpp::Node
// API that the ported Nav2 sources actually call: get_logger, now,
// get_name, get_parameter (read from rosparam), get_clock.
class LifecycleNode {
 public:
  using SharedPtr = std::shared_ptr<LifecycleNode>;
  using WeakPtr   = std::weak_ptr<LifecycleNode>;

  explicit LifecycleNode(const std::string & name = "")
  : name_(name), nh_(std::string("~/") + name) {}
  LifecycleNode(const std::string & name, const ros::NodeHandle & nh)
  : name_(name), nh_(nh) {}

  rclcpp::Logger get_logger() const { return rclcpp::Logger(name_); }
  rclcpp::Time   now()        const { return rclcpp::Time(ros::Time::now()); }
  const std::string & get_name() const { return name_; }

  std::shared_ptr<rclcpp::Clock> get_clock() const {
    return std::make_shared<rclcpp::Clock>();
  }

  // rclcpp::Node::get_parameter(name, T& val) returns bool. We map this onto
  // ros::NodeHandle::getParam. Nav2 uses "." separators inside parameter
  // names ("PathAlignCritic.cost_weight") — translate to "/" for rosparam.
  template <typename T>
  bool get_parameter(const std::string & name, T & val) const {
    std::string n = name;
    std::replace(n.begin(), n.end(), '.', '/');
    return nh_.getParam(n, val);
  }

  // declare_parameter under ROS 1 is a no-op; rosparam is implicitly typed.
  template <typename T>
  T declare_parameter(const std::string & name, const T & default_value) {
    T val;
    std::string n = name;
    std::replace(n.begin(), n.end(), '.', '/');
    if (!nh_.getParam(n, val)) {val = default_value;}
    return val;
  }
  template <typename T>
  void declare_parameter(const std::string & /*name*/) {}

  const ros::NodeHandle & nh() const { return nh_; }

 private:
  std::string name_;
  ros::NodeHandle nh_;
};
}  // namespace rclcpp_lifecycle

// nav2_util::LifecycleNode is Nav2's own thin wrapper around
// rclcpp_lifecycle::LifecycleNode; under ROS 1 they collapse to the same type.
namespace nav2_util {
  using LifecycleNode = rclcpp_lifecycle::LifecycleNode;
}

// rclcpp_lifecycle::LifecyclePublisher<T> is Nav2's managed publisher. Under
// ROS 1, the publisher lifecycle is implicit (latched on advertise, no
// activate/deactivate). The shim exposes publish(T) and on_activate/deactivate
// as no-ops so trajectory_visualizer.cpp et al. don't have to be rewritten
// beyond replacing the underlying ros::Publisher creation.
namespace rclcpp_lifecycle {
template <typename T>
class LifecyclePublisher {
 public:
  using SharedPtr = std::shared_ptr<LifecyclePublisher<T>>;
  LifecyclePublisher() = default;
  explicit LifecyclePublisher(const ros::Publisher & pub) : pub_(pub) {}
  void publish(const T & msg) { pub_.publish(msg); }
  void on_activate() {}
  void on_deactivate() {}
 private:
  ros::Publisher pub_;
};
}  // namespace rclcpp_lifecycle

// ───────────────────────────────────────────────────────────────────────────
// 5. nav2_util re-export shims
// ───────────────────────────────────────────────────────────────────────────
namespace nav2_util {

// Nav2's declare_parameter_if_not_declared(node, name, ParameterValue(default))
// in Humble lazily registers a param on the node before reading. Under ROS 1
// every rosparam is implicitly available (no declare step), so this is a
// no-op stub kept only so call sites referencing it parse cleanly. Callers
// must follow up with a real rosparam read (e.g. node_handle.param<T>).
template <class NodeT, class ValueT>
inline void declare_parameter_if_not_declared(
  NodeT &, const std::string &, ValueT) {}

namespace geometry_utils {
  inline double euclidean_distance(
    const geometry_msgs::Pose & a, const geometry_msgs::Pose & b)
  {
    const double dx = a.position.x - b.position.x;
    const double dy = a.position.y - b.position.y;
    const double dz = a.position.z - b.position.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
  }
  inline double euclidean_distance(
    const geometry_msgs::PoseStamped & a, const geometry_msgs::PoseStamped & b)
  {
    return euclidean_distance(a.pose, b.pose);
  }

  // Build a quaternion representing a rotation `angle` rad around the Z axis.
  inline geometry_msgs::Quaternion orientationAroundZAxis(double angle)
  {
    geometry_msgs::Quaternion q;
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(angle / 2.0);
    q.w = std::cos(angle / 2.0);
    return q;
  }
}  // namespace geometry_utils
}  // namespace nav2_util

// ───────────────────────────────────────────────────────────────────────────
// 6. nav2_core::exceptions re-export
// ───────────────────────────────────────────────────────────────────────────
namespace nav2_core {
class PlannerException : public std::runtime_error {
 public:
  explicit PlannerException(const std::string & what) : std::runtime_error(what) {}
};
class NoValidPathCouldBeFound : public PlannerException {
 public:
  explicit NoValidPathCouldBeFound(const std::string & what) : PlannerException(what) {}
};
class PlannerTimedOut : public PlannerException {
 public:
  explicit PlannerTimedOut(const std::string & what) : PlannerException(what) {}
};
class StartOccupied : public PlannerException {
 public:
  explicit StartOccupied(const std::string & what) : PlannerException(what) {}
};
class GoalOccupied : public PlannerException {
 public:
  explicit GoalOccupied(const std::string & what) : PlannerException(what) {}
};
class StartOutsideMapBounds : public PlannerException {
 public:
  explicit StartOutsideMapBounds(const std::string & what) : PlannerException(what) {}
};
class GoalOutsideMapBounds : public PlannerException {
 public:
  explicit GoalOutsideMapBounds(const std::string & what) : PlannerException(what) {}
};
class ControllerException : public std::runtime_error {
 public:
  explicit ControllerException(const std::string & what) : std::runtime_error(what) {}
};

// GoalChecker minimal interface (Nav2's nav2_core::GoalChecker plugin).
// Only the call surface optimizer/critics reference is preserved.
class GoalChecker {
 public:
  virtual ~GoalChecker() = default;
  virtual void getTolerances(
    geometry_msgs::Pose & pose_tolerance, geometry_msgs::Twist & vel_tolerance) = 0;
};
}  // namespace nav2_core

#endif  // NAV_ALGO_CORE__COMPAT_HPP_
