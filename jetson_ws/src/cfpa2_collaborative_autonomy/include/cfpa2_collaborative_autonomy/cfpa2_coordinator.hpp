// cfpa2_coordinator.hpp — C++ port of cfpa2_coordinator_node.py's
// CFPA2Coordinator. Multi-robot frontier allocator with utility scoring,
// goal-policy state machine, joint allocator for the dual-robot path, and
// space-time A* deconfliction. CFPA2SingleRobotNode inherits this and
// overrides tick_impl() for the single-robot decentralised path.

#pragma once

#include <array>
#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

// ─── ROS message + node API includes ─────────────────────────────────
// The hexagonal refactor confines ROS-specific code to the adapter
// layers, but the node class itself still inherits the ROS node base and
// declares ROS-message callbacks. The CFPA2_ROS1 guard selects the ROS 1
// (Noetic) headers + type spellings; the default (no guard) is the
// production ROS 2 Humble path and is byte-for-byte unchanged.
#ifdef CFPA2_ROS1
#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PointStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Empty.h"
#include "std_msgs/String.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#else
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#endif

#include "cfpa2_collaborative_autonomy/core/clock.hpp"
#include "cfpa2_collaborative_autonomy/core/logger.hpp"
#include "cfpa2_collaborative_autonomy/core/output.hpp"
#include "cfpa2_collaborative_autonomy/core/types.hpp"

#ifdef CFPA2_ROS1
#include "cfpa2_collaborative_autonomy/ros1/param_facade.hpp"
#endif

namespace cfpa2 {

// ── Re-export core POD types into the cfpa2:: namespace so callers
// using the legacy `cfpa2::Goal` / `cfpa2::GoalKey` continue to compile
// unchanged across the Phase-D refactor. Algorithm code newly written
// should reach for `cfpa2::core::*` directly.
using core::Goal;
using core::GoalKey;
using core::GoalKeyHash;
using core::BlacklistDisk;
using core::PoseSample;
using core::ProgressSample;
using core::ScoredGoal;
using core::UtilityList;

template <typename V>
using GoalKeyMap = core::GoalKeyMap<V>;

class CFPA2Coordinator
#ifdef CFPA2_ROS1
  : public ros1::RosParamFacade
#else
  : public rclcpp::Node
#endif
{
public:
  struct Options {
    std::string node_name = "cfpa2_coordinator";
    std::vector<std::string> default_namespaces = {"robot_a", "robot_b"};
    std::string startup_label = "cfpa2_coordinator";
    std::string planner_desc = "Coordinator";
  };

#ifdef CFPA2_ROS1
  // ROS 1 ctor: the algorithm body owns no node base. The public
  // NodeHandle (topic IO) + private NodeHandle (params, "~") are passed
  // in by main() and copied into nh_ / pnh_ members.
  CFPA2Coordinator(
      ros::NodeHandle & nh,
      ros::NodeHandle & pnh,
      const Options & opts);

  // Default convenience ctor (matches Python `CFPA2Coordinator()`): builds
  // a default public NodeHandle + a private ("~") NodeHandle internally.
  CFPA2Coordinator()
  : CFPA2Coordinator(Options{}) {}

  explicit CFPA2Coordinator(const Options & opts);

  virtual ~CFPA2Coordinator() = default;
#else
  CFPA2Coordinator(
      const rclcpp::NodeOptions & node_options,
      const Options & opts);

  // Default convenience ctor (matches Python `CFPA2Coordinator()`).
  explicit CFPA2Coordinator(
      const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
  : CFPA2Coordinator(node_options, Options{}) {}

  ~CFPA2Coordinator() override = default;
#endif

protected:
  // ─── declared parameter values, cached at startup ────────────────
  std::vector<std::string> namespaces_;
  double publish_rate_;
  double beta_;
  double sensor_range_;
  int frontier_stride_;
  int max_targets_;
  int frontier_raw_overfetch_factor_;
  int frontier_raw_overfetch_min_;
  std::string goal_topic_suffix_;
  std::string nav_status_topic_suffix_;
  std::string planning_map_topic_suffix_;
  std::int8_t free_value_;
  std::int8_t unknown_value_;
  std::int8_t occ_thresh_;
  std::int8_t cfpa2_reachability_occ_threshold_;
  bool cfpa2_reachability_allow_unknown_;
  std::string cfpa2_ig_mode_;
  int cfpa2_floodfill_budget_;
  int cfpa2_floodfill_max_radius_cells_;
  double switch_hysteresis_;
  double switch_min_dist_;
  double min_assign_distance_;
  double goal_lock_sec_;
  double progress_window_sec_;
  double progress_min_delta_m_;
  int blacklist_fail_count_;
  double blacklist_ttl_sec_;
  double blacklist_key_resolution_;
  double reached_blacklist_dist_;
  double goal_satisfied_dist_;
  bool goal_satisfied_requires_los_;
  double goal_satisfied_direct_dist_;
  double cfpa2_max_goal_distance_m_;
  int reached_blacklist_repeat_count_;
  double reached_blacklist_ttl_sec_;
  double overlap_weight_;
  double cfpa2_w_ig_;
  double cfpa2_w_c_;
  double cfpa2_w_sw_;
  double cfpa2_lambda_overlap_;
  double cfpa2_w_momentum_;
  double cfpa2_momentum_alpha_;
  double cfpa2_momentum_beta_;
  bool cfpa2_explore_unreachable_override_{false};
  double cfpa2_override_min_ig_{350.0};
  double cfpa2_override_dominance_{1.8};
  bool cfpa2_extent_seek_enabled_{false};
  double cfpa2_extent_target_x_{32.0};
  std::unordered_map<std::string, double> reached_x_min_;
  std::unordered_map<std::string, double> reached_x_max_;
  double cfpa2_frontier_cluster_radius_m_;
  double cfpa2_frontier_unknown_check_radius_m_;
  int cfpa2_frontier_min_unknown_cells_;
  double cfpa2_goal_min_hold_sec_;
  int cfpa2_challenger_streak_required_;
  double cfpa2_challenger_improvement_factor_;
  double cfpa2_challenger_min_lock_age_sec_;
  int cfpa2_tsp_k_;
  double cfpa2_stale_frontier_radius_m_;
  double cfpa2_min_utility_;
  double cfpa2_sigma_overlap_m_;
  double cfpa2_stuck_lock_sec_;
  double cfpa2_stuck_min_motion_m_;
  double cfpa2_stuck_blacklist_sec_;
  double cfpa2_stuck_window_sec_;
  double blacklist_cluster_radius_m_;
  double switch_hysteresis_max_lock_sec_;
  double local_nav_status_stale_sec_;
  double local_nav_stall_blacklist_sec_;
  bool fast_unreachable_enabled_;
  double fast_unreachable_blacklist_sec_;
  double fast_unreachable_startup_grace_sec_;
  int fast_unreachable_consecutive_threshold_;
  double cfpa2_close_stop_radius_m_;
  double cfpa2_close_stop_speed_epsilon_;
  bool cfpa2_space_time_enabled_;
  double cfpa2_space_time_horizon_sec_;
  double cfpa2_space_time_dt_sec_;
  double cfpa2_space_time_safety_radius_m_;
  double cfpa2_space_time_waypoint_lookahead_m_;
  double cfpa2_space_time_window_margin_m_;
  int cfpa2_space_time_max_expansions_;
  double cfpa2_space_time_assumed_speed_mps_;
  double cfpa2_space_time_max_speed_mps_;
  double cfpa2_frontier_min_cluster_area_m2_;
  double cfpa2_frontier_obstacle_clearance_m_;
  double cfpa2_goal_obstacle_clearance_m_;
  int exploration_gain_radius_cells_;
  std::string marker_frame_override_;
  std::string coordinator_map_topic_;
  std::string robot_markers_topic_;
  std::string frontier_markers_topic_;
  int trajectory_max_points_;
  double trajectory_min_point_distance_;
  double robot_marker_scale_;
  bool perf_enable_;
  int perf_tick_window_size_;
  int perf_min_samples_;
  double perf_tick_warn_p95_ms_;
  double perf_cpu_warn_pct_;
  bool adaptive_load_shedding_enabled_;
  double adaptive_budget_utilization_;
  double adaptive_restore_utilization_;
  int adaptive_max_frontier_stride_;
  int adaptive_min_max_targets_;
  int adaptive_min_exploration_gain_radius_cells_;
  int adaptive_max_skip_ticks_;
  bool debug_no_goal_logging_;
  double debug_no_goal_log_interval_sec_;
  double pivot_lock_radius_m_;
  double pivot_lock_max_hold_sec_;
  double pivot_lock_regress_release_m_;

  // ─── startup labels passed from ctor opts ────────────────────────
  std::string startup_label_;
  std::string planner_desc_;

  // ─── ROS-agnostic facade ─────────────────────────────────────────
  // Algorithm body should reach for these in preference to direct
  // calls to `get_clock()->now()` and the RCLCPP_* macros. The Phase D
  // hexagonal refactor (2026-05-19) means a Noetic port only needs to
  // swap rclcpp-based adapters for ros1-based ones; the algorithm
  // headers stay unchanged.
  std::shared_ptr<core::IClock> clock_facade_;
  std::shared_ptr<core::ILogger> log_facade_;
  std::shared_ptr<core::IGoalPublisher> goal_pub_facade_;
  std::shared_ptr<core::IVisualizer> viz_facade_;

  // ─── per-namespace state ─────────────────────────────────────────
  std::unordered_map<std::string, core::Grid> maps_;
  std::unordered_map<std::string, core::OdomXY> odoms_;
  // nav_status payload kept as parsed key→string-value map; numeric fields
  // are converted on the spot when consumed.
  std::unordered_map<std::string, std::map<std::string, std::string>> nav_status_;
  std::unordered_map<std::string, std::uint64_t> nav_status_rx_time_ns_;
  std::unordered_map<std::string, Goal> last_goal_;
  std::unordered_map<std::string, std::uint64_t> pivot_lock_held_since_ns_;
  std::unordered_map<std::string, double> pivot_lock_start_dist_m_;
  std::unordered_map<std::string, GoalKey> challenger_id_;
  std::unordered_map<std::string, int> challenger_streak_;
  std::unordered_map<std::string, std::uint64_t> last_goal_set_time_ns_;
  std::unordered_map<std::string, std::deque<ProgressSample>> goal_progress_samples_;
  std::unordered_map<std::string, GoalKeyMap<int>> goal_fail_counts_;
  std::unordered_map<std::string, GoalKeyMap<std::uint64_t>> goal_blacklist_until_ns_;
  std::unordered_map<std::string, std::vector<BlacklistDisk>> goal_blacklist_disks_;
  std::unordered_map<std::string, int> reached_goal_repeat_count_;
  std::unordered_map<std::string, std::optional<GoalKey>> reached_goal_last_key_;
  std::unordered_map<std::string, std::string> last_policy_reason_;
  std::unordered_map<std::string, std::uint64_t> odom_rx_time_ns_;
  std::unordered_map<std::string, std::pair<double, double>> odom_velocity_xy_;
  std::unordered_map<std::string, std::deque<std::pair<double, double>>> trajectory_history_;
  std::unordered_map<std::string, std::optional<std::pair<double, double>>> goal_lock_start_xy_;
  std::unordered_map<std::string, std::deque<PoseSample>> goal_lock_pose_history_;
  std::unordered_map<std::string, std::uint64_t> cfpa2_last_stuck_event_ns_;
  std::unordered_map<std::string, int> local_nav_last_stall_event_count_;
  std::unordered_map<std::string, std::uint64_t> frontier_replan_last_bl_ns_;
  std::unordered_map<std::string, int> last_unreachable_goal_seq_;
  std::unordered_map<std::string, GoalKeyMap<int>> unreachable_consec_;

  // Cached planning map for per-tick reads (set at top of tick_impl).
  std::optional<core::Grid> cur_planning_map_;

  // ─── global state ────────────────────────────────────────────────
  std::uint64_t cfpa2_last_close_stop_log_ns_ = 0;
  std::uint64_t start_ns_ = 0;
  std::uint64_t node_start_ns_ = 0;
  double summary_interval_sec_ = 10.0;
  std::uint64_t last_summary_ns_ = 0;
  std::uint64_t last_prereq_warn_ns_ = 0;
  std::uint64_t last_no_goal_debug_ns_ = 0;
  double tick_period_ms_ = 0.0;
  std::deque<double> perf_tick_durations_ms_;
  std::uint64_t last_perf_summary_ns_ = 0;
  double perf_last_cpu_process_sec_ = 0.0;
  std::uint64_t perf_last_cpu_wall_ns_ = 0;
  int adaptive_frontier_stride_ = 0;
  int adaptive_max_targets_ = 0;
  int adaptive_exploration_gain_radius_cells_ = 0;
  int adaptive_skip_ticks_ = 0;
  int adaptive_tick_skip_counter_ = 0;

  // Scratch buffer for batch_info_gain_floodfill — sized lazily to W*H.
  std::vector<std::int32_t> floodfill_scratch_;

  // ─── subscriptions + timer ───────────────────────────────────────
  // Publishers live behind goal_pub_facade_ and viz_facade_ (ROS-agnostic
  // interfaces). Subscriptions are kept Node-owned here for now because
  // the boundary conversion happens inside map_cb / odom_cb. A future
  // refactor can lift these into ros2::CoordinatorNode too.
#ifdef CFPA2_ROS1
  ros::NodeHandle nh_;    // public NodeHandle: topic IO
  ros::NodeHandle pnh_;   // private NodeHandle ("~"): params
  std::vector<ros::Subscriber> subs_;
  ros::Timer timer_;
#else
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subs_;
  rclcpp::TimerBase::SharedPtr timer_;
#endif

  // ─── callbacks ───────────────────────────────────────────────────
  // ROS-message callbacks. The bodies do `to_core_grid` / `to_core_odom`
  // at the boundary and then operate on POD types from there on.
#ifdef CFPA2_ROS1
  void map_cb(const nav_msgs::OccupancyGrid::ConstPtr & msg, const std::string & ns);
  void odom_cb(const nav_msgs::Odometry::ConstPtr & msg, const std::string & ns);
#else
  void map_cb(const nav_msgs::msg::OccupancyGrid::SharedPtr msg, const std::string & ns);
  void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg, const std::string & ns);
#endif

  // Internal helper that takes the already-converted POD odom + reception
  // timestamp. Algorithm-side callers go through this so the same logic
  // works when the Noetic adapter drives it directly.
  void on_odom(const std::string & ns, const core::OdomXY & odom, std::uint64_t now_ns);
#ifdef CFPA2_ROS1
  void nav_status_cb(const std_msgs::String::ConstPtr & msg, const std::string & ns);
#else
  void nav_status_cb(const std_msgs::msg::String::SharedPtr msg, const std::string & ns);
#endif
  void frontier_replan_cb(const std::string & ns);
  void apply_fast_blacklist(
      const std::string & ns,
      const std::map<std::string, std::string> & payload);

  // ─── pure helpers ────────────────────────────────────────────────
  static int grid_index(int x, int y, int w) noexcept { return y * w + x; }
  std::optional<std::pair<int, int>> world_to_grid(
      const core::Grid & msg, double wx, double wy) const;
  std::pair<double, double> grid_to_world(
      const core::Grid & msg, int gx, int gy) const;
  static std::vector<std::pair<int, int>> grid_line_cells(int x0, int y0, int x1, int y1);
  bool is_free_idx(const std::vector<std::int8_t> & data, int idx) const;
  bool is_unknown_idx(const std::vector<std::int8_t> & data, int idx) const;
  bool has_frontier_obstacle_clearance(
      const std::vector<std::int8_t> & data,
      int gx, int gy, int w, int h, int clearance_cells) const;
  bool goal_has_obstacle_clearance(
      const core::Grid & msg, double wx, double wy,
      double clearance_m) const;

  // ─── map ops (delegate to cfpa2::ops::*) ────────────────────────
  std::vector<Goal> extract_frontiers(const core::Grid & msg);
  std::vector<Goal> filter_dead_frontiers(
      const std::vector<Goal> & points, const core::Grid & msg);
  std::vector<Goal> cluster_representatives(
      const std::vector<Goal> & points, double cluster_radius_m);
  std::vector<Goal> merge_targets(
      const std::vector<std::vector<Goal>> & target_lists, double merge_resolution);
  int frontier_raw_capacity(int width, int height, int max_targets) const;

  // dist_map: linear cell index → BFS-distance in cells.
  std::unordered_map<int, int> distance_transform(
      const core::Grid & msg, std::pair<double, double> start_w);

  // ─── blacklist ───────────────────────────────────────────────────
  GoalKey goal_key(Goal goal) const;
  void prune_blacklist(const std::string & ns, std::uint64_t now_ns);
  bool is_blacklisted(const std::string & ns, Goal goal, std::uint64_t now_ns) const;
  void add_blacklist_disk(
      const std::string & ns, Goal goal,
      std::uint64_t until_ns, double radius_m);
  void register_goal_failure(
      const std::string & ns, Goal goal,
      std::uint64_t now_ns, const std::string & reason);
  std::set<std::string> consume_local_nav_stall_blacklists(std::uint64_t now_ns);
  void update_reached_goal_blacklist(
      const std::string & ns, std::uint64_t now_ns);
  void blacklist_active_goal(
      const std::string & ns, std::uint64_t now_ns, const std::string & reason);

  // ─── goal predicates ─────────────────────────────────────────────
  double distance_robot_to_goal(const std::string & ns, Goal goal) const;
  bool goal_too_close(const std::string & ns, Goal goal) const;
  bool goal_too_far(const std::string & ns, Goal goal) const;
  bool goal_satisfied(
      const std::string & ns, Goal goal,
      const core::Grid * map_msg = nullptr) const;
  bool goals_equivalent(Goal a, Goal b) const;
  bool goal_reachable(
      const core::Grid & msg,
      const std::unordered_map<int, int> & dist_map, Goal goal) const;
  bool goal_line_of_sight_clear(
      const core::Grid & map_msg, Goal robot_w, Goal goal_w) const;
  const core::Grid * goal_satisfaction_map(const std::string & ns) const;
  std::optional<std::string> held_goal_safety_failure(
      const core::Grid & msg,
      const std::unordered_map<int, int> & dist_map, Goal goal) const;

  // ─── progress + stuck ───────────────────────────────────────────
  std::optional<double> progress_delta(const std::string & ns) const;
  bool pivot_clearance_blocked(const std::string & ns) const;
  std::optional<double> max_displacement_in_window(
      const std::string & ns, std::uint64_t window_ns) const;
  Goal set_active_goal(
      const std::string & ns, Goal goal, std::uint64_t now_ns);
  std::optional<Goal> maybe_force_cfpa2_stuck_recovery(
      const std::string & ns, std::uint64_t now_ns,
      const UtilityList & utilities,
      const std::vector<Goal> & fallback_targets,
      const core::Grid * map_msg);
  std::optional<Goal> apply_cfpa2_proximity_stop(
      const std::string & ns, Goal goal,
      const core::Grid & map_msg);

  // ─── utility scoring ─────────────────────────────────────────────
  double cfpa2_single_utility(
      const std::string & ns, Goal goal,
      const core::Grid & map_msg,
      const std::unordered_map<int, int> & dist_map);
  double cfpa2_overlap_penalty(Goal goal_i, Goal goal_j) const;
  double cfpa2_switch_penalty(const std::string & ns, Goal goal) const;
  double cfpa2_momentum(const std::string & ns) const;
  double cfpa2_momentum_bonus(const std::string & ns, Goal goal) const;

  // ─── info gain ───────────────────────────────────────────────────
  double frontier_information_gain(
      const core::Grid & msg, Goal goal) const;
  double frontier_information_gain_floodfill(
      const core::Grid & msg, Goal goal) const;
  std::vector<double> batch_frontier_information_gain(
      const core::Grid & msg, const std::vector<Goal> & goals);
  std::optional<double> grid_path_cost_m(
      const core::Grid & msg,
      const std::unordered_map<int, int> & dist_map, Goal goal) const;

  // ─── space-time A* ───────────────────────────────────────────────
  std::optional<std::vector<std::pair<int, int>>> space_time_astar_cells(
      const std::string & ns,
      const core::Grid & map_msg,
      Goal final_goal,
      const std::unordered_map<std::string, Goal> & planned_goals);
  std::vector<std::set<std::pair<int, int>>> predict_other_robot_blocks(
      const core::Grid & map_msg,
      const std::string & ns,
      const std::unordered_map<std::string, Goal> & planned_goals,
      int steps, double dt_sec, int safety_radius_cells);
  std::optional<std::pair<int, int>> find_nearest_free_cell(
      const core::Grid & msg,
      std::pair<int, int> start, int search_radius);
  std::optional<Goal> cfpa2_space_time_waypoint(
      const std::string & ns,
      const std::vector<std::pair<int, int>> & path,
      const core::Grid & msg);

  // ─── goal selection ──────────────────────────────────────────────
  void set_policy_reason(const std::string & ns, const std::string & reason);
  Goal apply_switch_hysteresis(
      const std::string & ns, Goal goal, double assignment_score);
  Goal apply_goal_policy(
      const std::string & ns, Goal candidate_goal, double assignment_score,
      const core::Grid & map_msg,
      const std::unordered_map<int, int> & dist_map,
      std::uint64_t now_ns,
      const std::vector<Goal> * current_targets = nullptr);
  std::optional<Goal> tsp_top_k_head(
      const std::string & ns, const UtilityList & utilities, int k);
  std::optional<Goal> cfpa2_best_available_goal(
      const std::string & ns, std::uint64_t now_ns,
      const UtilityList & utilities,
      const std::optional<Goal> & exclude_goal,
      const std::vector<Goal> * fallback_targets,
      const core::Grid * map_msg,
      const std::unordered_map<int, int> * dist_map);

  // ─── tick ────────────────────────────────────────────────────────
  void tick();                  // timer callback wrapper
  virtual void tick_impl();     // overridable by single-robot subclass
  void record_tick_perf(std::uint64_t tick_start_ns);
  void update_adaptive_load_shedding(std::uint64_t now_ns);
  static bool goal_is_finite(Goal goal_w);

  // ─── publishers ──────────────────────────────────────────────────
  void publish_goal(const std::string & ns,
      const core::Grid & map_msg, Goal goal_w);
  void publish_goal_marker(const std::string & ns,
      const std::string & frame_id, Goal goal_w);
  void publish_coordinator_map(const core::Grid & target_map);
  void publish_robot_markers(const core::Grid & target_map);
  void publish_frontier_markers(
      const core::Grid & target_map,
      const std::vector<Goal> & targets);
  std::array<float, 3> ns_color(const std::string & ns) const;
  void append_trajectory(const std::string & ns, double x, double y);

  // ─── logging ─────────────────────────────────────────────────────
  void maybe_log_summary(std::uint64_t now_ns,
      const std::unordered_map<std::string, std::size_t> & per_ns_frontiers,
      const std::unordered_map<std::string, std::size_t> & per_ns_reachable,
      const std::unordered_map<std::string, std::optional<Goal>> & per_ns_assigned,
      const std::unordered_map<std::string, double> & per_ns_dist,
      const std::unordered_map<std::string, double> & per_ns_util,
      std::size_t targets_total);
  std::tuple<int, int, int> map_cell_stats(const core::Grid & msg) const;
  bool should_log_no_goal_debug(std::uint64_t now_ns);
  void log_no_goal_debug(
      std::uint64_t now_ns, const std::string & reason,
      const core::Grid & planning_map,
      const std::unordered_map<std::string, std::vector<Goal>> & per_ns_targets);
  static double percentile(const std::vector<double> & sorted_values, double quantile);
  void maybe_log_perf_summary();

  // ─── robot xy convenience ────────────────────────────────────────
  Goal robot_xy(const std::string & ns) const;

  // ─── hooks for subclasses ────────────────────────────────────────
  // Single-robot subclass overrides this to drop peer-claimed goals
  // before they enter the candidate pool. Non-const because the override
  // reads the current time to check blocked_frontiers TTL.
  virtual bool is_goal_peer_claimed(Goal /*goal*/) { return false; }

  // Single-robot subclass overrides this to inject ramp-ascent override.
  virtual std::optional<Goal> ramp_ascent_goal_if_valid(
      const std::string & /*ns*/,
      const core::Grid & /*map_msg*/,
      const std::unordered_map<int, int> & /*dist_map*/,
      std::uint64_t /*now_ns*/)
  { return std::nullopt; }

  // Single-robot subclass sets this true when it receives
  // exploration_complete and wants to stop publishing goals.
  bool paused_ = false;

private:
  // ─── ctor helpers ────────────────────────────────────────────────
  void declare_all_parameters(const std::vector<std::string> & default_namespaces);
  void read_all_parameters();
  void setup_publishers();
  void setup_subscriptions();
  void seed_per_ns_state();
};

}  // namespace cfpa2
