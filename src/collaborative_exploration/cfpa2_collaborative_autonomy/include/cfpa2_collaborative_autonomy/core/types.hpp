// core/types.hpp — ROS-independent POD types used by the CFPA2 algorithm.
//
// Everything in cfpa2::core is portable C++17 with no ROS dependency,
// so the same translation units compile against ROS 2 Humble (current
// production) and ROS 1 Noetic (the real-Go2 Orin NX target).
//
// The corresponding ROS 1 / ROS 2 message types live in the adapter
// layers (ros2/, future ros1/). Adapter code marshals between message
// fields and these POD structs at the subscription boundary.

#pragma once

#include <array>
#include <cstdint>
#include <deque>
#include <functional>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace cfpa2 {
namespace core {

/// World-frame goal point. Double precision throughout the algorithm;
/// the float-based ops kernels convert at the call boundary.
using Goal = std::pair<double, double>;

/// Quantised goal-cell key for blacklist lookup tables.
using GoalKey = std::pair<int, int>;

struct GoalKeyHash {
  std::size_t operator()(const GoalKey & k) const noexcept
  {
    const std::uint64_t a = static_cast<std::uint64_t>(static_cast<std::uint32_t>(k.first));
    const std::uint64_t b = static_cast<std::uint64_t>(static_cast<std::uint32_t>(k.second));
    return std::hash<std::uint64_t>{}((a << 32) ^ b);
  }
};

template <typename V>
using GoalKeyMap = std::unordered_map<GoalKey, V, GoalKeyHash>;

/// Snapshot of an OccupancyGrid's metadata + cell data, decoupled from
/// any ROS message type. Adapter code populates `info` + `data` directly
/// from `nav_msgs::msg::OccupancyGrid` (ROS 2) or `nav_msgs::OccupancyGrid`
/// (ROS 1). The struct layout intentionally mirrors both so a future
/// adapter can be a memcpy of the data buffer + scalar field copies.
struct GridInfo {
  int width = 0;
  int height = 0;
  double resolution = 0.0;
  double origin_x = 0.0;
  double origin_y = 0.0;
  std::string frame_id;
};

struct Grid {
  GridInfo info;
  std::vector<std::int8_t> data;
};

/// Minimal odometry snapshot the algorithm cares about: 2D pose + xy
/// linear velocity. Yaw is currently not used but kept for forward
/// compatibility with momentum-bonus refinements.
struct OdomXY {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  double vx = 0.0;
  double vy = 0.0;
};

// Blacklist disk: (x, y, radius_m, until_ns).
struct BlacklistDisk {
  double x;
  double y;
  double radius_m;
  std::uint64_t until_ns;
};

// (timestamp_ns, x, y) — pose-history entry for stuck-recovery checks.
struct PoseSample {
  std::uint64_t ns;
  double x;
  double y;
};

// (timestamp_ns, distance_m) — goal-progress sample.
struct ProgressSample {
  std::uint64_t ns;
  double distance_m;
};

// Per-tick utility table entry: goal + score.
struct ScoredGoal {
  Goal goal;
  double utility;
};
using UtilityList = std::vector<ScoredGoal>;

}  // namespace core
}  // namespace cfpa2
