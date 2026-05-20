# CFPA2 C++ → ROS 1 Noetic port checklist

Concrete file-by-file diff guide for porting `cfpa2_collaborative_autonomy`
from ROS 2 Humble (current Jetson Orin Nano + desktop target) to ROS 1
Noetic on the real Go2's Orin NX 16 GB.

The Phase-D + Phase-E hexagonal refactor (2026-05-19) isolated **every**
ROS-specific call inside `ros2/` adapters. The 1,400 LOC algorithm body
in `src/cfpa2_coordinator.cpp` + `src/cfpa2_single_robot.cpp` does
**not** need a rewrite — it uses `core::Grid` / `core::OdomXY` POD
types, `core::IClock` / `core::ILogger` / `core::IGoalPublisher` /
`core::IVisualizer` interfaces, and `CFPA2_LOG_INFO` / `CFPA2_LOG_WARN`
/ `CFPA2_LOG_ERROR` printf-style macros that route through ILogger.

After Phase E the only ROS-2-specific code left in the
non-`ros2/`-namespaced files is:
- `rclcpp::Node` inheritance on `CFPA2Coordinator`
- `declare_parameter` / `get_parameter` in the constructor
- `create_subscription<T>` / `create_wall_timer` in the constructor
- `nav_msgs::msg::OccupancyGrid::SharedPtr` etc. callback signatures
- `rclcpp::SubscriptionBase::SharedPtr` / `rclcpp::TimerBase::SharedPtr`
  member types

Everything else (publish_*, get_clock()->now(), RCLCPP_INFO, OccupancyGrid
field access) is already routed through interfaces that have ROS-1-side
counterparts you'll write once.

---

## 0. Architecture recap

```
include/cfpa2_collaborative_autonomy/
├── core/                          ← ROS-independent. Don't touch.
│   ├── types.hpp                  ← POD: Grid, OdomXY, Goal, ScoredGoal, BlacklistDisk, ...
│   ├── clock.hpp                  ← abstract IClock (now_ns())
│   ├── logger.hpp                 ← abstract ILogger (info/warn/error)
│   ├── logging.hpp                ← CFPA2_LOG_INFO/WARN/ERROR printf-style macros
│   └── output.hpp                 ← abstract IGoalPublisher / IVisualizer (+ RobotPoseView, TrajectoryView)
├── ops/                           ← ROS-independent kernel library. Don't touch.
│   ├── frontier_extract.hpp
│   ├── distance_transform.hpp
│   ├── info_gain.hpp
│   ├── dead_frontier_filter.hpp
│   ├── cluster.hpp
│   ├── grid_offsets.hpp
│   └── ops.hpp
├── ros2/                          ← ROS 2 adapters. COPY → ros1/ for Noetic.
│   ├── rclcpp_clock.hpp           → ros1/ros_clock.hpp
│   ├── rclcpp_logger.hpp          → ros1/ros_logger.hpp
│   ├── conversions.hpp            → ros1/conversions.hpp
│   ├── rclcpp_goal_publisher.hpp  → ros1/ros_goal_publisher.hpp
│   └── rclcpp_visualizer.hpp      → ros1/ros_visualizer.hpp
├── cfpa2_coordinator.hpp          ← class header. Strip `::msg`, swap sub/timer types.
└── cfpa2_single_robot.hpp         ← same.

src/
├── cfpa2_coordinator.cpp          ← algorithm body. ZERO hand edits.
├── cfpa2_single_robot.cpp         ← subclass. ZERO hand edits.
├── cfpa2_coordinator_node_main.cpp ← rclcpp::init/spin → ros::init/spin.
├── cfpa2_single_robot_node_main.cpp ← same.
└── ops/*.cpp                       ← kernel impls. Don't touch.
```

---

## 1. CMakeLists.txt

ROS 1 uses **catkin**, not ament_cmake. Rewrite top-level
`CMakeLists.txt` + add a `catkin_package(...)` call.

```cmake
# ROS 1 (Noetic) variant — keep alongside ROS 2 CMakeLists or split into
# noetic/CMakeLists.txt + a top-level dispatch.

cmake_minimum_required(VERSION 3.10)
project(cfpa2_collaborative_autonomy)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -O2)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  std_msgs
  visualization_msgs
  cfpa2_peer_coordination_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES cfpa2_ops cfpa2_node_lib
  CATKIN_DEPENDS roscpp geometry_msgs nav_msgs std_msgs visualization_msgs cfpa2_peer_coordination_msgs)

include_directories(include ${catkin_INCLUDE_DIRS})

# ops/ library — unchanged
add_library(cfpa2_ops STATIC
  src/ops/frontier_extract.cpp src/ops/distance_transform.cpp
  src/ops/info_gain.cpp src/ops/dead_frontier_filter.cpp src/ops/cluster.cpp)

# node lib — unchanged source list; CMakeLists just links catkin deps now
add_library(cfpa2_node_lib STATIC
  src/cfpa2_coordinator.cpp src/cfpa2_single_robot.cpp)
target_link_libraries(cfpa2_node_lib cfpa2_ops ${catkin_LIBRARIES})

# Executables
add_executable(cfpa2_coordinator_node src/cfpa2_coordinator_node_main.cpp)
target_link_libraries(cfpa2_coordinator_node cfpa2_node_lib)
add_executable(cfpa2_single_robot_node src/cfpa2_single_robot_node_main.cpp)
target_link_libraries(cfpa2_single_robot_node cfpa2_node_lib)
```

---

## 2. package.xml

```xml
<package format="2">
  <name>cfpa2_collaborative_autonomy</name>
  <version>0.2.0</version>
  <buildtool_depend>catkin</buildtool_depend>

  <depend>roscpp</depend>
  <depend>geometry_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>std_msgs</depend>
  <depend>visualization_msgs</depend>
  <depend>cfpa2_peer_coordination_msgs</depend>
</package>
```

`cfpa2_peer_coordination_msgs` will also need a ROS 1 build of the 4 .msg
files (PeerState, ClaimedFrontier, NegotiationRequest, NegotiationResponse).
Same `.msg` source files — just a catkin `add_message_files` declaration.

---

## 3. New file: `include/cfpa2_collaborative_autonomy/ros1/ros_clock.hpp`

```cpp
#pragma once
#include <ros/time.h>
#include "cfpa2_collaborative_autonomy/core/clock.hpp"

namespace cfpa2 {
namespace ros1 {

class RosClock : public core::IClock {
public:
  std::uint64_t now_ns() const override
  {
    // ros::Time::now() respects sim_time when /use_sim_time is true.
    return static_cast<std::uint64_t>(ros::Time::now().toNSec());
  }
};

}}  // namespace cfpa2::ros1
```

## 4. New file: `include/cfpa2_collaborative_autonomy/ros1/ros_logger.hpp`

```cpp
#pragma once
#include <ros/console.h>
#include "cfpa2_collaborative_autonomy/core/logger.hpp"

namespace cfpa2 {
namespace ros1 {

class RosLogger : public core::ILogger {
public:
  void info(const std::string & m) override  { ROS_INFO_STREAM(m); }
  void warn(const std::string & m) override  { ROS_WARN_STREAM(m); }
  void error(const std::string & m) override { ROS_ERROR_STREAM(m); }
};

}}  // namespace cfpa2::ros1
```

## 5. New file: `include/cfpa2_collaborative_autonomy/ros1/conversions.hpp`

```cpp
#pragma once
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <ros/time.h>
#include "cfpa2_collaborative_autonomy/core/types.hpp"

namespace cfpa2 {
namespace ros1 {

inline core::Grid to_core_grid(const nav_msgs::OccupancyGrid & msg)
{
  core::Grid g;
  g.info.width = msg.info.width;
  g.info.height = msg.info.height;
  g.info.resolution = msg.info.resolution;
  g.info.origin_x = msg.info.origin.position.x;
  g.info.origin_y = msg.info.origin.position.y;
  g.info.frame_id = msg.header.frame_id;
  g.data = msg.data;
  return g;
}

inline nav_msgs::OccupancyGrid to_msg_grid(const core::Grid & g)
{
  nav_msgs::OccupancyGrid msg;
  msg.info.width = g.info.width;
  msg.info.height = g.info.height;
  msg.info.resolution = g.info.resolution;
  msg.info.origin.position.x = g.info.origin_x;
  msg.info.origin.position.y = g.info.origin_y;
  msg.info.origin.orientation.w = 1.0;
  msg.header.frame_id = g.info.frame_id;
  msg.data = g.data;
  return msg;
}

inline core::OdomXY to_core_odom(const nav_msgs::Odometry & msg)
{
  const auto & q = msg.pose.pose.orientation;
  const double yaw = std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  core::OdomXY o;
  o.x = msg.pose.pose.position.x;
  o.y = msg.pose.pose.position.y;
  o.yaw = yaw;
  o.vx = msg.twist.twist.linear.x;
  o.vy = msg.twist.twist.linear.y;
  return o;
}

inline geometry_msgs::PointStamped to_msg_point_stamped(
    const core::Goal & goal, const std::string & frame_id, const ros::Time & stamp)
{
  geometry_msgs::PointStamped p;
  p.header.frame_id = frame_id;
  p.header.stamp = stamp;
  p.point.x = goal.first;
  p.point.y = goal.second;
  p.point.z = 0.0;
  return p;
}

}}  // namespace cfpa2::ros1
```

---

## 6. `cfpa2_coordinator.hpp` — header diff

Mechanical replacements (sed-style):

```diff
-#include "geometry_msgs/msg/point.hpp"
-#include "geometry_msgs/msg/point_stamped.hpp"
-#include "nav_msgs/msg/occupancy_grid.hpp"
-#include "nav_msgs/msg/odometry.hpp"
-#include "rclcpp/rclcpp.hpp"
-#include "std_msgs/msg/empty.hpp"
-#include "std_msgs/msg/string.hpp"
-#include "visualization_msgs/msg/marker.hpp"
-#include "visualization_msgs/msg/marker_array.hpp"
+#include <ros/ros.h>
+#include <geometry_msgs/Point.h>
+#include <geometry_msgs/PointStamped.h>
+#include <nav_msgs/OccupancyGrid.h>
+#include <nav_msgs/Odometry.h>
+#include <std_msgs/Empty.h>
+#include <std_msgs/String.h>
+#include <visualization_msgs/Marker.h>
+#include <visualization_msgs/MarkerArray.h>
```

Class declaration:

```diff
-class CFPA2Coordinator : public rclcpp::Node
+class CFPA2Coordinator
```

Member types:

```diff
-  std::unordered_map<std::string, nav_msgs::msg::OccupancyGrid> maps_;
-  std::unordered_map<std::string, nav_msgs::msg::Odometry> odoms_;
+  std::unordered_map<std::string, nav_msgs::OccupancyGrid> maps_;
+  std::unordered_map<std::string, nav_msgs::Odometry> odoms_;
...
-  std::unordered_map<std::string, rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr> goal_pubs_;
-  std::unordered_map<std::string, rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr> goal_marker_pubs_;
-  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr coordinator_map_pub_;
-  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr robot_markers_pub_;
-  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr frontier_markers_pub_;
-  std::vector<rclcpp::SubscriptionBase::SharedPtr> subs_;
-  rclcpp::TimerBase::SharedPtr timer_;
+  ros::NodeHandle nh_;
+  ros::NodeHandle pnh_;  // private NH for params
+  std::unordered_map<std::string, ros::Publisher> goal_pubs_;
+  std::unordered_map<std::string, ros::Publisher> goal_marker_pubs_;
+  ros::Publisher coordinator_map_pub_;
+  ros::Publisher robot_markers_pub_;
+  ros::Publisher frontier_markers_pub_;
+  std::vector<ros::Subscriber> subs_;
+  ros::Timer timer_;
```

Callback signatures (the SharedPtr → ConstPtr shift is the main change):

```diff
-  void map_cb(const nav_msgs::msg::OccupancyGrid::SharedPtr msg, const std::string & ns);
-  void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg, const std::string & ns);
-  void nav_status_cb(const std_msgs::msg::String::SharedPtr msg, const std::string & ns);
+  void map_cb(const nav_msgs::OccupancyGrid::ConstPtr & msg, const std::string & ns);
+  void odom_cb(const nav_msgs::Odometry::ConstPtr & msg, const std::string & ns);
+  void nav_status_cb(const std_msgs::String::ConstPtr & msg, const std::string & ns);
```

Ctor:

```diff
-  CFPA2Coordinator(
-      const rclcpp::NodeOptions & node_options,
-      const Options & opts);
+  CFPA2Coordinator(ros::NodeHandle & nh, ros::NodeHandle & pnh, const Options & opts);
```

Method signatures that take ROS messages — same field-by-field, just
strip `::msg`:

```diff
-  std::vector<Goal> extract_frontiers(const nav_msgs::msg::OccupancyGrid & msg);
+  std::vector<Goal> extract_frontiers(const nav_msgs::OccupancyGrid & msg);
```

There are ~25 such method signatures. A sed pass handles them all:

```bash
sed -i 's/nav_msgs::msg::/nav_msgs::/g;
        s/geometry_msgs::msg::/geometry_msgs::/g;
        s/std_msgs::msg::/std_msgs::/g;
        s/visualization_msgs::msg::/visualization_msgs::/g' \
    include/cfpa2_collaborative_autonomy/cfpa2_coordinator.hpp
```

---

## 7. `cfpa2_coordinator.cpp` — algorithm body

The algorithm body is **almost** unchanged. Things to touch:

### 7a. Header include + namespace alias

```diff
-#include "cfpa2_collaborative_autonomy/ros2/rclcpp_clock.hpp"
-#include "cfpa2_collaborative_autonomy/ros2/rclcpp_logger.hpp"
-#include "rclcpp/qos.hpp"
+#include "cfpa2_collaborative_autonomy/ros1/ros_clock.hpp"
+#include "cfpa2_collaborative_autonomy/ros1/ros_logger.hpp"
```

### 7b. Ctor body

```diff
-  clock_facade_ = std::make_shared<ros2::RclcppClock>(this->get_clock());
-  log_facade_ = std::make_shared<ros2::RclcppLogger>(this->get_logger());
+  clock_facade_ = std::make_shared<ros1::RosClock>();
+  log_facade_ = std::make_shared<ros1::RosLogger>();
```

Parameter declarations / reads:

```diff
-  declare_parameter<double>("publish_rate", 1.0);
-  publish_rate_ = get_parameter("publish_rate").as_double();
+  pnh_.param("publish_rate", publish_rate_, 1.0);
```

(or use `pnh_.getParam("publish_rate", publish_rate_)` if you want to
keep declared defaults.) Do this mechanically for the ~100 params.

Publishers + subscriptions:

```diff
-  coordinator_map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
-      coordinator_map_topic_, coordinator_map_qos);
+  coordinator_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(
+      coordinator_map_topic_, 1, /*latch=*/true);
...
-  subs_.push_back(create_subscription<nav_msgs::msg::OccupancyGrid>(
-      map_topic, 1,
-      [this, ns](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) { map_cb(msg, ns); }));
+  subs_.push_back(nh_.subscribe<nav_msgs::OccupancyGrid>(
+      map_topic, 1,
+      boost::bind(&CFPA2Coordinator::map_cb, this, _1, ns)));
```

Timer:

```diff
-  timer_ = this->create_wall_timer(period, std::bind(&CFPA2Coordinator::tick, this));
+  timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_),
+                            std::bind(&CFPA2Coordinator::tick, this));
```

### 7c. RCLCPP_* → ROS_* (regex)

```bash
sed -i 's/RCLCPP_INFO(get_logger(), /ROS_INFO(/g;
        s/RCLCPP_WARN(get_logger(), /ROS_WARN(/g;
        s/RCLCPP_ERROR(get_logger(), /ROS_ERROR(/g;
        s/RCLCPP_INFO(this->get_logger(), /ROS_INFO(/g;
        s/RCLCPP_WARN(this->get_logger(), /ROS_WARN(/g;
        s/RCLCPP_ERROR(this->get_logger(), /ROS_ERROR(/g' \
    src/cfpa2_coordinator.cpp src/cfpa2_single_robot.cpp
```

The format specifiers (`%s`, `%d`, `%.2f`) are identical in both. No
behavioural change.

### 7d. ROS 2 → ROS 1 namespace strip (already covered in §6)

Same sed pass on the .cpp.

### 7e. Message stamps

The four remaining `get_clock()->now()` calls populate `header.stamp`
(ROS 2 `rclcpp::Time`). Replace:

```diff
-  msg.header.stamp = get_clock()->now();
+  msg.header.stamp = ros::Time::now();
```

(4 sites; left in the algorithm body until the publisher abstraction
ships in Phase E.)

---

## 8. Main entry point

```diff
- src/cfpa2_coordinator_node_main.cpp
- int main(int argc, char ** argv) {
-   rclcpp::init(argc, argv);
-   auto node = std::make_shared<cfpa2::CFPA2Coordinator>();
-   rclcpp::spin(node);
-   rclcpp::shutdown();
-   return 0;
- }

+ src/cfpa2_coordinator_node_main.cpp (ROS 1 variant)
+ int main(int argc, char ** argv) {
+   ros::init(argc, argv, "cfpa2_coordinator");
+   ros::NodeHandle nh, pnh("~");
+   cfpa2::CFPA2Coordinator node(nh, pnh, /*opts*/{});
+   ros::spin();
+   return 0;
+ }
```

Same shape for `cfpa2_single_robot_node_main.cpp`.

---

## 9. cfpa2_grid_ops.so / ctypes shim

The `src/cfpa2_grid_ops_c_api.cpp` extern "C" wrappers + ops/ kernel
library are pure C++ (no ROS). They build unchanged on Noetic — but if
the Python coordinator is no longer used, you can drop the entire
`cfpa2_grid_ops` shared library + its install rules.

---

## 10. Topic / TF conventions

Topic names + msg semantics are identical across ROS 1 / ROS 2 for the
existing CFPA2 contract:

- `/robot/<planning_map_topic_suffix>`  (OccupancyGrid)
- `/robot/odom/nav` (Odometry)
- `/robot/nav_status` (String, JSON payload — the manual parser in
  `apply_fast_blacklist` is portable)
- `/robot/frontier_replan` (Empty)
- `/robot/way_point_coord` (PointStamped)
- `/robot/exploration_status` (String)
- `/robot/cfpa2_peer_coordination/blocked_frontiers` (PoseArray) ← PR-4

The peer coordinator (Python, separate package) runs on either ROS 2
or ROS 1 — only its `package.xml` + `setup.py` need updating; the
protocol message types are auto-generated from `.msg` files for both.

TF is the same conceptual tree (`map → odom → base_link`) but the C++
APIs differ:
- ROS 2: `tf2_ros::Buffer / TransformListener` with `rclcpp` interfaces.
- ROS 1: `tf2_ros::Buffer / TransformListener` with `roscpp` interfaces.

CFPA2 doesn't currently read TF directly (it gets the robot pose from
`/odom/nav`), so no TF code to port.

---

## 11. Estimated work (updated after Phase E)

| Section | LOC | Estimated time |
|---|---|---|
| 1. CMakeLists.txt + package.xml rewrite | ~80 | 30 min |
| 3–5. ros1/ adapter headers (clock + logger + conversions + goal_publisher + visualizer) | **~400** (copy ros2/ headers, swap `nav_msgs::msg::` → `nav_msgs::`, `rclcpp::*` → `ros::*`) | 1.5 h |
| 6. `cfpa2_coordinator.hpp` ROS 2 → ROS 1: strip `::msg`, swap subscription/timer member types | ~20 lines edited | 20 min |
| 7. `cfpa2_coordinator.cpp` ctor: `declare_parameter` → `pnh.param`, `create_publisher` already removed (lives in `ros2::RclcppGoalPublisher` / `ros2::RclcppVisualizer` — copy these to `ros1/` with ROS 1 publisher API), `create_subscription` → `nh.subscribe`, `create_wall_timer` → `nh.createTimer` | ~50 lines edited | 1 h |
| 7b. Drop `rclcpp::Node` inheritance, accept `ros::NodeHandle &` in ctor | ~10 lines edited | 15 min |
| 7c. `cfpa2_single_robot.cpp` same set of ctor edits | ~30 lines edited | 30 min |
| 8. Main entry point rewrite (rclcpp::init/spin → ros::init/spin) | ~20 | 15 min |
| Build + smoke test on Orin NX | — | 1.5 h |
| **Total** | **~610 LOC adapter + ~100 LOC edited** | **~5 h** |

Algorithm body (1,400 LOC across `cfpa2_coordinator.cpp` + `cfpa2_single_robot.cpp`):
**zero hand edits.** Phase E pulled all of these:
- `RCLCPP_INFO/WARN/ERROR` → `CFPA2_LOG_INFO/WARN/ERROR` (already through ILogger)
- `get_clock()->now()` → `clock_facade_->now_ns()` (already through IClock)
- `nav_msgs::msg::OccupancyGrid` → `core::Grid` (already POD)
- `nav_msgs::msg::Odometry` → `core::OdomXY` (already POD)
- All `publish_*` impls → call `goal_pub_facade_->publish_goal(...)` / `viz_facade_->publish_*` (already through interfaces)

The Noetic port is now genuinely "write 2 adapter files + edit ctor + write main + build". ~5 h total, basically zero re-implementation of business logic.

---

## 12. Things the port does NOT need to touch

- `include/cfpa2_collaborative_autonomy/core/` — POD types, IClock,
  ILogger, IGoalPublisher, IVisualizer
- `include/cfpa2_collaborative_autonomy/ops/` — frontier extract,
  distance transform, info gain, dead-frontier filter, cluster
  representatives (all 7 hot kernels)
- `src/ops/*.cpp` — kernel implementations
- Algorithm logic in `cfpa2_coordinator.cpp` (apply_goal_policy,
  apply_fast_blacklist, joint allocator, stuck recovery, blacklist
  bookkeeping, utility scoring, IG, etc.)
- The 4 `.msg` files in `cfpa2_peer_coordination_msgs/msg/` (auto-gen
  C++ bindings work for both ROS versions)
- The 7 yaml configs in `config/`
- The 36 unit tests (they exercise pure-Python or pure-C++ logic; the
  C++ ones need a different test runner — gtest instead of pytest)

---

## 13. Sanity checks after port

1. Workspace builds: `catkin build cfpa2_collaborative_autonomy`
2. Binary launches: `rosrun cfpa2_collaborative_autonomy cfpa2_single_robot_node`
3. Topic IO: `rostopic info /robot/way_point_coord` shows the C++ node as publisher
4. Re-run the perf bench (port `/tmp/cfpa2_perf_bench.py` to rclpy →
   rospy — trivial) and confirm tick p95 stays ≤ 2 ms on Orin NX
   (Orin NX is ~1.33× the Orin Nano per-core; expected p95 ≤ 0.75 ms)
5. Smoke against an existing real-robot bag with the Noetic FAST-LIO2
   pipeline (`onboard_noetic_*` series) — see
   `docs/claude/noetic_fastlio_onboard.md` for the bag-replay setup

---

## 14. Reference: the abstraction contract

Anything in `cfpa2::core::*` must remain ROS-agnostic. If a future
refactor wants to add ROS-touching code, it goes in `ros2/` (ROS 2) or
`ros1/` (ROS 1), and exposes its functionality via the existing
abstract interfaces (or a new one in `core/`). The algorithm side
takes a reference / shared_ptr to the interface; the adapter ctor
constructs the implementation and passes it in.

This is the **ports and adapters** pattern. The Noetic port is exactly
the "add a second adapter" exercise.
