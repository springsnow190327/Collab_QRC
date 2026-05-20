// core/output.hpp — abstract publisher interfaces so the algorithm
// doesn't depend on rclcpp::Publisher<T> / ros::Publisher directly.
//
// All goal / marker / status emission goes through these interfaces.
// The ros2 adapter wraps create_publisher<T>(...) calls and converts
// core POD output → ROS 2 messages. The future ros1 adapter does the
// same with nh.advertise<T>(...).

#pragma once

#include <array>
#include <string>
#include <vector>

#include "cfpa2_collaborative_autonomy/core/types.hpp"

namespace cfpa2 {
namespace core {

/// Lightweight trajectory polyline shape: (frame_id, color, points).
struct TrajectoryView {
  std::string frame_id;
  std::array<float, 3> color{0.0f, 0.0f, 0.0f};
  std::vector<std::pair<double, double>> points_xy;
};

/// Lightweight robot-pose marker shape.
struct RobotPoseView {
  std::string frame_id;
  double x = 0.0;
  double y = 0.0;
  double z = 0.15;
  double yaw = 0.0;
  std::array<float, 3> color{0.0f, 0.0f, 0.0f};
  double scale = 0.35;
};

/// Goal output to a single robot's executor. `frame_id` is whichever
/// frame the controller expects (typically marker_frame_override, else
/// the planning map's frame_id).
class IGoalPublisher
{
public:
  virtual ~IGoalPublisher() = default;
  virtual void publish_goal(
      const std::string & ns,
      Goal goal,
      const std::string & frame_id) = 0;

  /// Per-namespace goal marker (sphere). Color is the namespace's hash
  /// colour; scale + z handled by the adapter to keep this interface
  /// purely about what to publish.
  virtual void publish_goal_marker(
      const std::string & ns,
      Goal goal,
      const std::string & frame_id,
      std::array<float, 3> color) = 0;

  /// Pause / status string emitted on `<ns>/exploration_status`.
  virtual void publish_status(const std::string & ns, const std::string & status) = 0;
};

/// Visualization aggregate: coordinator map + robot markers + frontier
/// markers. Separated from IGoalPublisher because some deployments may
/// want to disable viz entirely (e.g. headless Jetson HIL).
class IVisualizer
{
public:
  virtual ~IVisualizer() = default;
  /// Republish the coordinator's view of the planning map (debug topic).
  virtual void publish_coordinator_map(const Grid & grid) = 0;

  /// Per-robot pose spheres + trajectory polylines.
  virtual void publish_robot_markers(
      const std::vector<RobotPoseView> & robot_poses,
      const std::vector<TrajectoryView> & trajectories) = 0;

  /// Frontier point cloud (sphere list) in the given frame.
  virtual void publish_frontier_markers(
      const std::string & frame_id,
      const std::vector<Goal> & frontiers) = 0;
};

}  // namespace core
}  // namespace cfpa2
