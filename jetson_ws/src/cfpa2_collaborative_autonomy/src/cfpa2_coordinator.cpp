// cfpa2_coordinator.cpp — C++ port of cfpa2_coordinator_node.py.
// Implementations live in this single translation unit for now; future
// refactors may split it by concern (callbacks / policy / publishers).

#include "cfpa2_collaborative_autonomy/cfpa2_coordinator.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <functional>
#include <limits>
#include <queue>
#include <sstream>

#include "cfpa2_collaborative_autonomy/core/logging.hpp"
#include "cfpa2_collaborative_autonomy/ops/ops.hpp"
#ifdef CFPA2_ROS1
#include "cfpa2_collaborative_autonomy/ros1/conversions.hpp"
#include "cfpa2_collaborative_autonomy/ros1/roscpp_clock.hpp"
#include "cfpa2_collaborative_autonomy/ros1/roscpp_logger.hpp"
#include "cfpa2_collaborative_autonomy/ros1/roscpp_goal_publisher.hpp"
#include "cfpa2_collaborative_autonomy/ros1/roscpp_visualizer.hpp"
#include "boost/bind.hpp"
#else
#include "cfpa2_collaborative_autonomy/ros2/conversions.hpp"
#include "cfpa2_collaborative_autonomy/ros2/rclcpp_clock.hpp"
#include "cfpa2_collaborative_autonomy/ros2/rclcpp_logger.hpp"
#include "cfpa2_collaborative_autonomy/ros2/rclcpp_goal_publisher.hpp"
#include "cfpa2_collaborative_autonomy/ros2/rclcpp_visualizer.hpp"
#include "rclcpp/qos.hpp"
#endif

namespace cfpa2 {

namespace {

constexpr double kPi = 3.14159265358979323846;

#ifndef CFPA2_ROS1
// Convert rclcpp time to ns since epoch (steady) — matches Python's
// `self.get_clock().now().nanoseconds`. (ROS 1 has no rclcpp::Time;
// this helper is unused dead code retained for the ROS 2 build.)
inline std::uint64_t to_ns(const rclcpp::Time & t)
{
  return static_cast<std::uint64_t>(t.nanoseconds());
}
#endif

// Lightweight JSON-ish payload parser. nav_status_v1 messages have the
// shape `{"state": "unreachable", "goal_seq": 123, ...}`. We only need
// string/number fields; values are stored as raw strings.
std::map<std::string, std::string> parse_simple_json_object(const std::string & s)
{
  std::map<std::string, std::string> out;
  // Strip outer braces + whitespace.
  std::string body;
  body.reserve(s.size());
  bool seen_open = false;
  for (char c : s) {
    if (!seen_open) {
      if (c == '{') seen_open = true;
      continue;
    }
    if (c == '}') break;
    body.push_back(c);
  }
  if (body.empty()) return out;
  // Split top-level commas (we don't expect nested objects/arrays in our
  // nav_status messages, so a naive split is fine).
  std::size_t i = 0;
  while (i < body.size()) {
    // skip leading whitespace/commas.
    while (i < body.size() && (body[i] == ',' || body[i] == ' ' || body[i] == '\t')) ++i;
    if (i >= body.size()) break;
    // Key must be "quoted".
    if (body[i] != '"') break;
    ++i;
    std::size_t k0 = i;
    while (i < body.size() && body[i] != '"') ++i;
    if (i >= body.size()) break;
    std::string key = body.substr(k0, i - k0);
    ++i;  // skip closing quote
    // Skip past ':' and whitespace.
    while (i < body.size() && (body[i] == ':' || body[i] == ' ' || body[i] == '\t')) ++i;
    // Value: either "string" or a bareword/number, terminated by ',' or end.
    std::string val;
    if (i < body.size() && body[i] == '"') {
      ++i;
      std::size_t v0 = i;
      while (i < body.size() && body[i] != '"') ++i;
      val = body.substr(v0, i - v0);
      if (i < body.size()) ++i;  // skip closing quote
    } else {
      std::size_t v0 = i;
      while (i < body.size() && body[i] != ',') ++i;
      val = body.substr(v0, i - v0);
      // trim trailing whitespace
      while (!val.empty() && (val.back() == ' ' || val.back() == '\t')) val.pop_back();
    }
    out[key] = val;
  }
  return out;
}

}  // namespace

#ifdef CFPA2_ROS1
CFPA2Coordinator::CFPA2Coordinator(const Options & opts)
: CFPA2Coordinator(*(new ros::NodeHandle()), *(new ros::NodeHandle("~")), opts)
{
  // Delegating to the (nh, pnh, opts) ctor with default-constructed
  // NodeHandles. Heap allocation is intentional: NodeHandle is
  // refcounted by roscpp and outlives the temporaries; the (nh, pnh)
  // ctor copies them into nh_ / pnh_ members which keep them alive.
}

CFPA2Coordinator::CFPA2Coordinator(
    ros::NodeHandle & nh,
    ros::NodeHandle & pnh,
    const Options & opts)
: nh_(nh),
  pnh_(pnh)
{
  startup_label_ = opts.startup_label;
  planner_desc_ = opts.planner_desc;

  // Point the rclcpp-style param facade at the private NodeHandle so
  // declare_all_parameters() / read_all_parameters() compile unchanged.
  set_param_handle(&pnh_);

  // ROS-agnostic facade for time + logging.
  clock_facade_ = std::make_shared<ros1::RoscppClock>();
  log_facade_ = std::make_shared<ros1::RoscppLogger>();

  declare_all_parameters(opts.default_namespaces);
  read_all_parameters();
  seed_per_ns_state();
  setup_publishers();
  setup_subscriptions();

  // Tick timer.
  tick_period_ms_ = 1000.0 / std::max(0.2, publish_rate_);
  timer_ = nh_.createTimer(
      ros::Duration(1.0 / std::max(0.2, publish_rate_)),
      [this](const ros::TimerEvent &) { tick(); });

  start_ns_ = clock_facade_->now_ns();
#else
CFPA2Coordinator::CFPA2Coordinator(
    const rclcpp::NodeOptions & node_options,
    const Options & opts)
: rclcpp::Node(opts.node_name, node_options),
  startup_label_(opts.startup_label),
  planner_desc_(opts.planner_desc)
{
  // ROS-agnostic facade for time + logging. The algorithm body uses
  // these so a Noetic port only has to swap the adapter implementation.
  clock_facade_ = std::make_shared<ros2::RclcppClock>(this->get_clock());
  log_facade_ = std::make_shared<ros2::RclcppLogger>(this->get_logger());

  declare_all_parameters(opts.default_namespaces);
  read_all_parameters();
  seed_per_ns_state();
  setup_publishers();
  setup_subscriptions();

  // Tick timer.
  const auto period = std::chrono::nanoseconds(
      static_cast<std::int64_t>(1.0e9 / std::max(0.2, publish_rate_)));
  tick_period_ms_ = 1000.0 / std::max(0.2, publish_rate_);
  timer_ = this->create_wall_timer(period, std::bind(&CFPA2Coordinator::tick, this));

  start_ns_ = clock_facade_->now_ns();
#endif
  node_start_ns_ = start_ns_;

  CFPA2_LOG_INFO(log_facade_,"[planner_startup] %s initialized.", startup_label_.c_str());

  std::ostringstream nss;
  for (std::size_t i = 0; i < namespaces_.size(); ++i) {
    if (i) nss << ", ";
    nss << namespaces_[i];
  }
  CFPA2_LOG_INFO(log_facade_,
      "%s started for [%s]\n"
      "  -- Utility weights --\n"
      "    w_ig=%.2f w_c=%.2f w_sw=%.2f w_momentum=%.2f min_utility=%.2f\n"
      "  -- Frontier --\n"
      "    sensor_range=%.1fm gain_radius=%dcells min_cluster=%.2fm^2 beta=%.2f\n"
      "  -- Assignment --\n"
      "    min_dist=%.2fm switch_hysteresis=%.3f goal_lock=%.0fs",
      planner_desc_.c_str(), nss.str().c_str(),
      cfpa2_w_ig_, cfpa2_w_c_, cfpa2_w_sw_, cfpa2_w_momentum_, cfpa2_min_utility_,
      sensor_range_, exploration_gain_radius_cells_,
      cfpa2_frontier_min_cluster_area_m2_, beta_,
      min_assign_distance_, switch_hysteresis_, goal_lock_sec_);
}

void CFPA2Coordinator::declare_all_parameters(
    const std::vector<std::string> & default_namespaces)
{
  declare_parameter<std::vector<std::string>>("namespaces", default_namespaces);
  declare_parameter<double>("publish_rate", 1.0);
  declare_parameter<double>("beta", 0.18);
  declare_parameter<double>("sensor_range", 3.5);
  declare_parameter<int>("frontier_stride", 2);
  declare_parameter<int>("max_targets", 800);
  declare_parameter<int>("frontier_raw_overfetch_factor", 20);
  declare_parameter<int>("frontier_raw_overfetch_min", 4096);
  declare_parameter<std::string>("goal_topic_suffix", "/way_point_coord");
  declare_parameter<std::string>("nav_status_topic_suffix", "/nav_status");
  declare_parameter<std::string>("planning_map_topic_suffix", "/map");
  declare_parameter<int>("free_value", 0);
  declare_parameter<int>("unknown_value", -1);
  declare_parameter<int>("occupancy_block_threshold", 50);
  declare_parameter<int>("cfpa2_reachability_occ_threshold", 0);
  declare_parameter<bool>("cfpa2_reachability_allow_unknown", false);
  declare_parameter<std::string>("cfpa2_ig_mode", "local");
  declare_parameter<int>("cfpa2_floodfill_budget", 2000);
  declare_parameter<int>("cfpa2_floodfill_max_radius_cells", 100);
  declare_parameter<double>("switch_hysteresis", 0.02);
  declare_parameter<double>("switch_min_dist", 0.35);
  declare_parameter<double>("min_assign_distance", 0.30);
  declare_parameter<double>("goal_lock_sec", 5.0);
  declare_parameter<double>("progress_window_sec", 3.0);
  declare_parameter<double>("progress_min_delta_m", 0.15);
  declare_parameter<int>("blacklist_fail_count", 2);
  declare_parameter<double>("blacklist_ttl_sec", 30.0);
  declare_parameter<double>("blacklist_key_resolution", 0.5);
  declare_parameter<double>("reached_blacklist_dist", 0.30);
  declare_parameter<double>("goal_satisfied_dist", 0.0);
  declare_parameter<bool>("goal_satisfied_requires_los", true);
  declare_parameter<double>("goal_satisfied_direct_dist", 0.30);
  declare_parameter<double>("cfpa2_max_goal_distance_m", 0.0);
  declare_parameter<int>("reached_blacklist_repeat_count", 3);
  declare_parameter<double>("reached_blacklist_ttl_sec", 12.0);
  declare_parameter<double>("overlap_weight", 1.0);
  declare_parameter<double>("cfpa2_w_ig", 1.0);
  declare_parameter<double>("cfpa2_w_c", 0.6);
  declare_parameter<double>("cfpa2_w_sw", 0.2);
  declare_parameter<double>("cfpa2_lambda_overlap", 0.5);
  declare_parameter<double>("cfpa2_w_momentum", 2.0);
  declare_parameter<double>("cfpa2_momentum_alpha", 1.5);
  declare_parameter<double>("cfpa2_momentum_beta", 2.0);
  // High-IG unreachable-region override (extent-completeness). When the robot
  // has reduced its reachable region to small interior pockets (best reachable
  // IG < override_min_ig) but a much larger unexplored region exists across an
  // unreachable gap (e.g. the −x corridor in the V-shaped ops2 building), snap
  // the largest such unreachable frontier to its nearest reachable cell and
  // commit there — driving the robot toward the big unexplored region instead
  // of grinding explored nooks forever. Disabled by default; ops2 overlay
  // enables it. See the Pass-1 block + 2026-05-20 notes.
  declare_parameter<bool>("cfpa2_explore_unreachable_override", false);
  declare_parameter<double>("cfpa2_override_min_ig", 350.0);
  declare_parameter<double>("cfpa2_override_dominance", 1.8);
  // Extent-seek (bidirectional-coverage completeness). The ops2 building is
  // V-shaped from spawn; greedy exploration exhausts ONE arm (reaches x≈+34)
  // and either grinds its explored pockets or wedges in the far corner, never
  // returning for the other arm. Extent-seek tracks the robot's PHYSICALLY
  // traversed x-extent and, once one ±x extreme is reached (|x| ≥
  // extent_target_x), redirects toward the unreached extreme by committing to
  // the frontier furthest in that direction (snapped to a reachable cell if
  // needed). It fires at extent_target_x (< the corner-wedge point) so the
  // robot turns around while still mobile. Disabled by default; ops2 enables.
  declare_parameter<bool>("cfpa2_extent_seek_enabled", false);
  declare_parameter<double>("cfpa2_extent_target_x", 32.0);
  declare_parameter<double>("cfpa2_frontier_cluster_radius_m", 1.0);
  declare_parameter<double>("cfpa2_frontier_unknown_check_radius_m", 0.40);
  declare_parameter<int>("cfpa2_frontier_min_unknown_cells", 20);
  declare_parameter<double>("cfpa2_goal_min_hold_sec", 5.0);
  declare_parameter<int>("cfpa2_challenger_streak_required", 3);
  declare_parameter<double>("cfpa2_challenger_improvement_factor", 1.20);
  declare_parameter<double>("cfpa2_challenger_min_lock_age_sec", 2.0);
  declare_parameter<int>("cfpa2_tsp_k", 5);
  declare_parameter<double>("cfpa2_stale_frontier_radius_m", 0.40);
  declare_parameter<double>("cfpa2_min_utility", -0.5);
  declare_parameter<double>("cfpa2_sigma_overlap_m", 0.0);
  declare_parameter<double>("cfpa2_stuck_lock_sec", 45.0);
  declare_parameter<double>("cfpa2_stuck_min_motion_m", 0.20);
  declare_parameter<double>("cfpa2_stuck_blacklist_sec", 60.0);
  declare_parameter<double>("cfpa2_stuck_window_sec", 45.0);
  declare_parameter<double>("blacklist_cluster_radius_m", 1.0);
  declare_parameter<double>("switch_hysteresis_max_lock_sec", 20.0);
  declare_parameter<double>("local_nav_status_stale_sec", 3.0);
  declare_parameter<double>("local_nav_stall_blacklist_sec", 45.0);
  declare_parameter<bool>("fast_unreachable_enabled", true);
  declare_parameter<double>("fast_unreachable_blacklist_sec", 60.0);
  declare_parameter<double>("fast_unreachable_startup_grace_sec", 15.0);
  declare_parameter<int>("fast_unreachable_consecutive_threshold", 3);
  declare_parameter<double>("cfpa2_close_stop_radius_m", 0.35);
  declare_parameter<double>("cfpa2_close_stop_speed_epsilon", 0.02);
  declare_parameter<bool>("cfpa2_space_time_enabled", true);
  declare_parameter<double>("cfpa2_space_time_horizon_sec", 5.0);
  declare_parameter<double>("cfpa2_space_time_dt_sec", 0.40);
  declare_parameter<double>("cfpa2_space_time_safety_radius_m", 0.45);
  declare_parameter<double>("cfpa2_space_time_waypoint_lookahead_m", 0.9);
  declare_parameter<double>("cfpa2_space_time_window_margin_m", 3.0);
  declare_parameter<int>("cfpa2_space_time_max_expansions", 12000);
  declare_parameter<double>("cfpa2_space_time_assumed_speed_mps", 0.25);
  declare_parameter<double>("cfpa2_space_time_max_speed_mps", 0.60);
  declare_parameter<double>("cfpa2_frontier_min_cluster_area_m2", 0.20);
  declare_parameter<double>("cfpa2_frontier_obstacle_clearance_m", 0.40);
  declare_parameter<double>("cfpa2_goal_obstacle_clearance_m", 0.0);
  declare_parameter<int>("exploration_gain_radius_cells", 4);
  declare_parameter<std::string>("marker_frame_override", "world");
  declare_parameter<std::string>("coordinator_map_topic", "/mtare/coordinator_map");
  declare_parameter<std::string>("robot_markers_topic", "/mtare/robot_markers");
  declare_parameter<std::string>("frontier_markers_topic", "/mtare/frontier_markers");
  declare_parameter<int>("trajectory_max_points", 600);
  declare_parameter<double>("trajectory_min_point_distance", 0.08);
  declare_parameter<double>("robot_marker_scale", 0.35);
  declare_parameter<bool>("perf_enable", true);
  declare_parameter<int>("perf_tick_window_size", 240);
  declare_parameter<int>("perf_min_samples", 20);
  declare_parameter<double>("perf_tick_warn_p95_ms", 150.0);
  declare_parameter<double>("perf_cpu_warn_pct", 15.0);
  declare_parameter<bool>("adaptive_load_shedding_enabled", false);
  declare_parameter<double>("adaptive_budget_utilization", 0.85);
  declare_parameter<double>("adaptive_restore_utilization", 0.55);
  declare_parameter<int>("adaptive_max_frontier_stride", 8);
  declare_parameter<int>("adaptive_min_max_targets", 120);
  declare_parameter<int>("adaptive_min_exploration_gain_radius_cells", 2);
  declare_parameter<int>("adaptive_max_skip_ticks", 2);
  declare_parameter<bool>("debug_no_goal_logging", true);
  declare_parameter<double>("debug_no_goal_log_interval_sec", 2.0);
  declare_parameter<double>("pivot_lock_radius_m", 0.45);
  declare_parameter<double>("pivot_lock_max_hold_sec", 15.0);
  declare_parameter<double>("pivot_lock_regress_release_m", 0.50);
}

void CFPA2Coordinator::read_all_parameters()
{
  namespaces_ = get_parameter("namespaces").as_string_array();
  publish_rate_ = std::max(0.2, get_parameter("publish_rate").as_double());
  beta_ = get_parameter("beta").as_double();
  sensor_range_ = std::max(0.1, get_parameter("sensor_range").as_double());
  frontier_stride_ = std::max(1, static_cast<int>(get_parameter("frontier_stride").as_int()));
  max_targets_ = std::max(50, static_cast<int>(get_parameter("max_targets").as_int()));
  frontier_raw_overfetch_factor_ = std::max(
      1, static_cast<int>(get_parameter("frontier_raw_overfetch_factor").as_int()));
  frontier_raw_overfetch_min_ = std::max(
      max_targets_,
      static_cast<int>(get_parameter("frontier_raw_overfetch_min").as_int()));
  goal_topic_suffix_ = get_parameter("goal_topic_suffix").as_string();

  std::string pm = get_parameter("planning_map_topic_suffix").as_string();
  if (!pm.empty() && pm[0] != '/') pm.insert(pm.begin(), '/');
  if (pm.empty()) pm = "/map";
  planning_map_topic_suffix_ = pm;

  nav_status_topic_suffix_ = get_parameter("nav_status_topic_suffix").as_string();
  free_value_ = static_cast<std::int8_t>(get_parameter("free_value").as_int());
  unknown_value_ = static_cast<std::int8_t>(get_parameter("unknown_value").as_int());
  occ_thresh_ = static_cast<std::int8_t>(get_parameter("occupancy_block_threshold").as_int());

  const int reach_occ_thr =
      static_cast<int>(get_parameter("cfpa2_reachability_occ_threshold").as_int());
  cfpa2_reachability_occ_threshold_ = reach_occ_thr > 0
      ? static_cast<std::int8_t>(std::max(1, std::min(100, reach_occ_thr)))
      : occ_thresh_;
  cfpa2_reachability_allow_unknown_ =
      get_parameter("cfpa2_reachability_allow_unknown").as_bool();

  std::string ig_mode = get_parameter("cfpa2_ig_mode").as_string();
  if (ig_mode != "local" && ig_mode != "floodfill") {
    CFPA2_LOG_WARN(log_facade_,"cfpa2_ig_mode='%s' invalid, falling back to 'local'",
        ig_mode.c_str());
    ig_mode = "local";
  }
  cfpa2_ig_mode_ = ig_mode;
  cfpa2_floodfill_budget_ = std::max(
      1, static_cast<int>(get_parameter("cfpa2_floodfill_budget").as_int()));
  cfpa2_floodfill_max_radius_cells_ = std::max(
      1, static_cast<int>(get_parameter("cfpa2_floodfill_max_radius_cells").as_int()));

  switch_hysteresis_ = std::max(0.0, get_parameter("switch_hysteresis").as_double());
  switch_min_dist_ = std::max(0.1, get_parameter("switch_min_dist").as_double());
  min_assign_distance_ = std::max(0.0, get_parameter("min_assign_distance").as_double());
  goal_lock_sec_ = std::max(0.0, get_parameter("goal_lock_sec").as_double());
  progress_window_sec_ = std::max(0.5, get_parameter("progress_window_sec").as_double());
  progress_min_delta_m_ = std::max(0.0, get_parameter("progress_min_delta_m").as_double());
  blacklist_fail_count_ = std::max(1, static_cast<int>(get_parameter("blacklist_fail_count").as_int()));
  blacklist_ttl_sec_ = std::max(0.0, get_parameter("blacklist_ttl_sec").as_double());
  blacklist_key_resolution_ = std::max(0.05, get_parameter("blacklist_key_resolution").as_double());
  reached_blacklist_dist_ = std::max(0.0, get_parameter("reached_blacklist_dist").as_double());

  const double cfg_gs = get_parameter("goal_satisfied_dist").as_double();
  goal_satisfied_dist_ = cfg_gs > 0.0
      ? std::max(0.1, cfg_gs)
      : std::max(switch_min_dist_, reached_blacklist_dist_);
  goal_satisfied_requires_los_ = get_parameter("goal_satisfied_requires_los").as_bool();
  goal_satisfied_direct_dist_ = std::min(
      goal_satisfied_dist_,
      std::max(0.0, get_parameter("goal_satisfied_direct_dist").as_double()));
  cfpa2_max_goal_distance_m_ = std::max(0.0, get_parameter("cfpa2_max_goal_distance_m").as_double());
  reached_blacklist_dist_ = std::max(reached_blacklist_dist_, goal_satisfied_dist_);
  reached_blacklist_repeat_count_ = std::max(
      1, static_cast<int>(get_parameter("reached_blacklist_repeat_count").as_int()));
  reached_blacklist_ttl_sec_ = std::max(0.0, get_parameter("reached_blacklist_ttl_sec").as_double());

  overlap_weight_ = std::max(0.0, get_parameter("overlap_weight").as_double());
  cfpa2_w_ig_ = get_parameter("cfpa2_w_ig").as_double();
  cfpa2_w_c_ = std::max(0.0, get_parameter("cfpa2_w_c").as_double());
  cfpa2_w_sw_ = std::max(0.0, get_parameter("cfpa2_w_sw").as_double());
  cfpa2_lambda_overlap_ = std::max(0.0, get_parameter("cfpa2_lambda_overlap").as_double());
  cfpa2_w_momentum_ = std::max(0.0, get_parameter("cfpa2_w_momentum").as_double());
  cfpa2_momentum_alpha_ = std::max(0.0, get_parameter("cfpa2_momentum_alpha").as_double());
  cfpa2_momentum_beta_ = std::max(0.0, get_parameter("cfpa2_momentum_beta").as_double());
  cfpa2_explore_unreachable_override_ =
      get_parameter("cfpa2_explore_unreachable_override").as_bool();
  cfpa2_override_min_ig_ =
      std::max(0.0, get_parameter("cfpa2_override_min_ig").as_double());
  cfpa2_override_dominance_ =
      std::max(1.0, get_parameter("cfpa2_override_dominance").as_double());
  cfpa2_extent_seek_enabled_ =
      get_parameter("cfpa2_extent_seek_enabled").as_bool();
  cfpa2_extent_target_x_ =
      std::max(0.0, get_parameter("cfpa2_extent_target_x").as_double());
  cfpa2_frontier_cluster_radius_m_ =
      std::max(0.0, get_parameter("cfpa2_frontier_cluster_radius_m").as_double());
  cfpa2_frontier_unknown_check_radius_m_ =
      std::max(0.0, get_parameter("cfpa2_frontier_unknown_check_radius_m").as_double());
  cfpa2_frontier_min_unknown_cells_ = std::max(
      0, static_cast<int>(get_parameter("cfpa2_frontier_min_unknown_cells").as_int()));
  cfpa2_goal_min_hold_sec_ = std::max(0.0, get_parameter("cfpa2_goal_min_hold_sec").as_double());
  cfpa2_challenger_streak_required_ = std::max(
      0, static_cast<int>(get_parameter("cfpa2_challenger_streak_required").as_int()));
  cfpa2_challenger_improvement_factor_ = std::max(
      1.0, get_parameter("cfpa2_challenger_improvement_factor").as_double());
  cfpa2_challenger_min_lock_age_sec_ =
      std::max(0.0, get_parameter("cfpa2_challenger_min_lock_age_sec").as_double());
  cfpa2_tsp_k_ = std::max(1, static_cast<int>(get_parameter("cfpa2_tsp_k").as_int()));
  cfpa2_stale_frontier_radius_m_ =
      std::max(0.05, get_parameter("cfpa2_stale_frontier_radius_m").as_double());
  cfpa2_min_utility_ = get_parameter("cfpa2_min_utility").as_double();
  cfpa2_sigma_overlap_m_ = std::max(0.0, get_parameter("cfpa2_sigma_overlap_m").as_double());
  cfpa2_stuck_lock_sec_ = std::max(0.0, get_parameter("cfpa2_stuck_lock_sec").as_double());
  cfpa2_stuck_min_motion_m_ = std::max(0.0, get_parameter("cfpa2_stuck_min_motion_m").as_double());
  cfpa2_stuck_blacklist_sec_ = std::max(0.0, get_parameter("cfpa2_stuck_blacklist_sec").as_double());
  cfpa2_stuck_window_sec_ = std::max(1.0, get_parameter("cfpa2_stuck_window_sec").as_double());
  blacklist_cluster_radius_m_ = std::max(0.0, get_parameter("blacklist_cluster_radius_m").as_double());
  switch_hysteresis_max_lock_sec_ =
      std::max(0.0, get_parameter("switch_hysteresis_max_lock_sec").as_double());
  local_nav_status_stale_sec_ =
      std::max(0.0, get_parameter("local_nav_status_stale_sec").as_double());
  local_nav_stall_blacklist_sec_ =
      std::max(0.0, get_parameter("local_nav_stall_blacklist_sec").as_double());
  fast_unreachable_enabled_ = get_parameter("fast_unreachable_enabled").as_bool();
  fast_unreachable_blacklist_sec_ =
      std::max(5.0, get_parameter("fast_unreachable_blacklist_sec").as_double());
  fast_unreachable_startup_grace_sec_ =
      std::max(0.0, get_parameter("fast_unreachable_startup_grace_sec").as_double());
  fast_unreachable_consecutive_threshold_ = std::max(
      1, static_cast<int>(get_parameter("fast_unreachable_consecutive_threshold").as_int()));
  cfpa2_close_stop_radius_m_ = std::max(0.0, get_parameter("cfpa2_close_stop_radius_m").as_double());
  cfpa2_close_stop_speed_epsilon_ =
      std::max(0.0, get_parameter("cfpa2_close_stop_speed_epsilon").as_double());

  cfpa2_space_time_enabled_ = get_parameter("cfpa2_space_time_enabled").as_bool();
  cfpa2_space_time_horizon_sec_ =
      std::max(0.5, get_parameter("cfpa2_space_time_horizon_sec").as_double());
  cfpa2_space_time_dt_sec_ =
      std::max(0.05, get_parameter("cfpa2_space_time_dt_sec").as_double());
  cfpa2_space_time_safety_radius_m_ =
      std::max(0.0, get_parameter("cfpa2_space_time_safety_radius_m").as_double());
  cfpa2_space_time_waypoint_lookahead_m_ =
      std::max(0.1, get_parameter("cfpa2_space_time_waypoint_lookahead_m").as_double());
  cfpa2_space_time_window_margin_m_ =
      std::max(0.0, get_parameter("cfpa2_space_time_window_margin_m").as_double());
  cfpa2_space_time_max_expansions_ = std::max(
      1000, static_cast<int>(get_parameter("cfpa2_space_time_max_expansions").as_int()));
  cfpa2_space_time_assumed_speed_mps_ =
      std::max(0.01, get_parameter("cfpa2_space_time_assumed_speed_mps").as_double());
  cfpa2_space_time_max_speed_mps_ = std::max(
      cfpa2_space_time_assumed_speed_mps_,
      get_parameter("cfpa2_space_time_max_speed_mps").as_double());

  cfpa2_frontier_min_cluster_area_m2_ =
      std::max(0.0, get_parameter("cfpa2_frontier_min_cluster_area_m2").as_double());
  cfpa2_frontier_obstacle_clearance_m_ =
      std::max(0.0, get_parameter("cfpa2_frontier_obstacle_clearance_m").as_double());
  const double cfg_goal_clr = get_parameter("cfpa2_goal_obstacle_clearance_m").as_double();
  cfpa2_goal_obstacle_clearance_m_ = cfg_goal_clr > 0.0
      ? cfg_goal_clr
      : cfpa2_frontier_obstacle_clearance_m_;
  exploration_gain_radius_cells_ = std::max(
      1, static_cast<int>(get_parameter("exploration_gain_radius_cells").as_int()));

  marker_frame_override_ = get_parameter("marker_frame_override").as_string();
  coordinator_map_topic_ = get_parameter("coordinator_map_topic").as_string();
  robot_markers_topic_ = get_parameter("robot_markers_topic").as_string();
  frontier_markers_topic_ = get_parameter("frontier_markers_topic").as_string();
  trajectory_max_points_ = std::max(
      10, static_cast<int>(get_parameter("trajectory_max_points").as_int()));
  trajectory_min_point_distance_ =
      std::max(0.0, get_parameter("trajectory_min_point_distance").as_double());
  robot_marker_scale_ = std::max(0.05, get_parameter("robot_marker_scale").as_double());

  perf_enable_ = get_parameter("perf_enable").as_bool();
  perf_tick_window_size_ = std::max(
      20, static_cast<int>(get_parameter("perf_tick_window_size").as_int()));
  perf_min_samples_ = std::max(5, static_cast<int>(get_parameter("perf_min_samples").as_int()));
  perf_tick_warn_p95_ms_ = std::max(0.0, get_parameter("perf_tick_warn_p95_ms").as_double());
  perf_cpu_warn_pct_ = std::max(0.0, get_parameter("perf_cpu_warn_pct").as_double());

  adaptive_load_shedding_enabled_ = get_parameter("adaptive_load_shedding_enabled").as_bool();
  adaptive_budget_utilization_ =
      std::max(0.1, std::min(0.99, get_parameter("adaptive_budget_utilization").as_double()));
  adaptive_restore_utilization_ =
      std::max(0.1, std::min(adaptive_budget_utilization_,
          get_parameter("adaptive_restore_utilization").as_double()));
  adaptive_max_frontier_stride_ = std::max(
      frontier_stride_,
      static_cast<int>(get_parameter("adaptive_max_frontier_stride").as_int()));
  adaptive_min_max_targets_ = std::max(
      50, static_cast<int>(get_parameter("adaptive_min_max_targets").as_int()));
  adaptive_min_exploration_gain_radius_cells_ = std::max(
      1, static_cast<int>(get_parameter("adaptive_min_exploration_gain_radius_cells").as_int()));
  adaptive_max_skip_ticks_ = std::max(
      0, static_cast<int>(get_parameter("adaptive_max_skip_ticks").as_int()));

  debug_no_goal_logging_ = get_parameter("debug_no_goal_logging").as_bool();
  debug_no_goal_log_interval_sec_ =
      std::max(0.2, get_parameter("debug_no_goal_log_interval_sec").as_double());
  pivot_lock_radius_m_ = std::max(0.0, get_parameter("pivot_lock_radius_m").as_double());
  pivot_lock_max_hold_sec_ = std::max(0.0, get_parameter("pivot_lock_max_hold_sec").as_double());
  pivot_lock_regress_release_m_ =
      std::max(0.0, get_parameter("pivot_lock_regress_release_m").as_double());

  // Initialise adaptive trackers to base values.
  adaptive_frontier_stride_ = frontier_stride_;
  adaptive_max_targets_ = max_targets_;
  adaptive_exploration_gain_radius_cells_ = exploration_gain_radius_cells_;
  adaptive_skip_ticks_ = 0;
  adaptive_tick_skip_counter_ = 0;
}

void CFPA2Coordinator::seed_per_ns_state()
{
  for (const auto & ns : namespaces_) {
    goal_progress_samples_[ns];
    goal_fail_counts_[ns];
    goal_blacklist_until_ns_[ns];
    goal_blacklist_disks_[ns];
    reached_goal_repeat_count_[ns] = 0;
    reached_goal_last_key_[ns] = std::nullopt;
    last_policy_reason_[ns] = "init";
    odom_velocity_xy_[ns] = {0.0, 0.0};
    trajectory_history_[ns];
    goal_lock_start_xy_[ns] = std::nullopt;
    goal_lock_pose_history_[ns];
    cfpa2_last_stuck_event_ns_[ns] = 0;
    local_nav_last_stall_event_count_[ns] = 0;
    frontier_replan_last_bl_ns_[ns] = 0;
    unreachable_consec_[ns];
  }
}

void CFPA2Coordinator::setup_publishers()
{
  // All publishers live behind the goal_pub_facade_ + viz_facade_ adapters.
  // The Noetic port swaps the ros2:: impls (constructed from the
  // rclcpp::Node*) for ros1:: impls (constructed from a ros::NodeHandle&).
#ifdef CFPA2_ROS1
  goal_pub_facade_ = std::make_shared<ros1::RoscppGoalPublisher>(
      nh_, namespaces_, goal_topic_suffix_);
  viz_facade_ = std::make_shared<ros1::RoscppVisualizer>(
      nh_,
      coordinator_map_topic_, robot_markers_topic_, frontier_markers_topic_,
      robot_marker_scale_);
#else
  goal_pub_facade_ = std::make_shared<ros2::RclcppGoalPublisher>(
      this, namespaces_, goal_topic_suffix_);
  viz_facade_ = std::make_shared<ros2::RclcppVisualizer>(
      this,
      coordinator_map_topic_, robot_markers_topic_, frontier_markers_topic_,
      robot_marker_scale_);
#endif
}

void CFPA2Coordinator::setup_subscriptions()
{
  for (const auto & ns : namespaces_) {
    const std::string map_topic = "/" + ns + planning_map_topic_suffix_;
#ifdef CFPA2_ROS1
    subs_.push_back(nh_.subscribe<nav_msgs::OccupancyGrid>(
        map_topic, 1,
        boost::bind(&CFPA2Coordinator::map_cb, this, _1, ns)));
#else
    subs_.push_back(create_subscription<nav_msgs::msg::OccupancyGrid>(
        map_topic, 1,
        [this, ns](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
          map_cb(msg, ns);
        }));
#endif
    CFPA2_LOG_INFO(log_facade_,"[%s] planning map <- %s (occ_thresh=%d, unknown=%d)",
        ns.c_str(), map_topic.c_str(),
        static_cast<int>(occ_thresh_), static_cast<int>(unknown_value_));

#ifdef CFPA2_ROS1
    subs_.push_back(nh_.subscribe<nav_msgs::Odometry>(
        "/" + ns + "/odom/nav", 10,
        boost::bind(&CFPA2Coordinator::odom_cb, this, _1, ns)));

    subs_.push_back(nh_.subscribe<std_msgs::String>(
        "/" + ns + nav_status_topic_suffix_, 10,
        boost::bind(&CFPA2Coordinator::nav_status_cb, this, _1, ns)));

    subs_.push_back(nh_.subscribe<std_msgs::Empty>(
        "/" + ns + "/frontier_replan", 10,
        boost::function<void(const std_msgs::Empty::ConstPtr &)>(
            [this, ns](const std_msgs::Empty::ConstPtr &) {
              frontier_replan_cb(ns);
            })));
#else
    subs_.push_back(create_subscription<nav_msgs::msg::Odometry>(
        "/" + ns + "/odom/nav", 10,
        [this, ns](const nav_msgs::msg::Odometry::SharedPtr msg) {
          odom_cb(msg, ns);
        }));

    subs_.push_back(create_subscription<std_msgs::msg::String>(
        "/" + ns + nav_status_topic_suffix_, 10,
        [this, ns](const std_msgs::msg::String::SharedPtr msg) {
          nav_status_cb(msg, ns);
        }));

    subs_.push_back(create_subscription<std_msgs::msg::Empty>(
        "/" + ns + "/frontier_replan", 10,
        [this, ns](const std_msgs::msg::Empty::SharedPtr) {
          frontier_replan_cb(ns);
        }));
#endif
  }
}

// ───────────────────────────────────────────────────────────────────
//  Callback implementations
// ───────────────────────────────────────────────────────────────────

#ifdef CFPA2_ROS1
void CFPA2Coordinator::map_cb(
    const nav_msgs::OccupancyGrid::ConstPtr & msg, const std::string & ns)
{
  // Convert ROS msg → POD at the boundary. The algorithm body only
  // ever sees core::Grid.
  maps_[ns] = ros1::to_core_grid(*msg);
}

void CFPA2Coordinator::odom_cb(
    const nav_msgs::Odometry::ConstPtr & msg, const std::string & ns)
{
  on_odom(ns, ros1::to_core_odom(*msg), clock_facade_->now_ns());
}
#else
void CFPA2Coordinator::map_cb(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg, const std::string & ns)
{
  // Convert ROS msg → POD at the boundary. The algorithm body only
  // ever sees core::Grid; the Noetic adapter does the same with
  // ros1::to_core_grid().
  maps_[ns] = ros2::to_core_grid(*msg);
}

void CFPA2Coordinator::odom_cb(
    const nav_msgs::msg::Odometry::SharedPtr msg, const std::string & ns)
{
  on_odom(ns, ros2::to_core_odom(*msg), clock_facade_->now_ns());
}
#endif

void CFPA2Coordinator::on_odom(
    const std::string & ns, const core::OdomXY & odom, std::uint64_t now_ns)
{
  odoms_[ns] = odom;
  odom_rx_time_ns_[ns] = now_ns;
  odom_velocity_xy_[ns] = {odom.vx, odom.vy};
  append_trajectory(ns, odom.x, odom.y);

  auto & history = goal_lock_pose_history_[ns];
  history.push_back({now_ns, odom.x, odom.y});
  // Trim by time + cap to 4096 samples.
  const std::uint64_t cutoff_ns = now_ns >
      static_cast<std::uint64_t>(cfpa2_stuck_window_sec_ * 1e9)
      ? now_ns - static_cast<std::uint64_t>(cfpa2_stuck_window_sec_ * 1e9)
      : 0;
  while (!history.empty() && history.front().ns < cutoff_ns) history.pop_front();
  while (history.size() > 4096) history.pop_front();
}

#ifdef CFPA2_ROS1
void CFPA2Coordinator::nav_status_cb(
    const std_msgs::String::ConstPtr & msg, const std::string & ns)
#else
void CFPA2Coordinator::nav_status_cb(
    const std_msgs::msg::String::SharedPtr msg, const std::string & ns)
#endif
{
  const auto payload = parse_simple_json_object(msg->data);
  if (payload.empty()) return;
  nav_status_[ns] = payload;
  nav_status_rx_time_ns_[ns] = clock_facade_->now_ns();
  if (fast_unreachable_enabled_) {
    apply_fast_blacklist(ns, payload);
  }
}

void CFPA2Coordinator::frontier_replan_cb(const std::string & ns)
{
  const auto now_ns = clock_facade_->now_ns();
  if (now_ns - frontier_replan_last_bl_ns_[ns] < static_cast<std::uint64_t>(1.0 * 1e9)) {
    return;  // 1 s debounce
  }
  frontier_replan_last_bl_ns_[ns] = now_ns;
  blacklist_active_goal(ns, now_ns, "frontier_replan");
}

namespace {

// Parse "[1.5, 2.3]" or "(1.5, 2.3)" → optional<(x, y)>. Tolerates spaces.
std::optional<std::pair<double, double>> parse_xy_pair(const std::string & s)
{
  std::string body;
  body.reserve(s.size());
  for (char c : s) {
    if (c != '[' && c != ']' && c != '(' && c != ')' && c != ' ' && c != '\t') {
      body.push_back(c);
    }
  }
  auto comma = body.find(',');
  if (comma == std::string::npos) return std::nullopt;
  try {
    const double x = std::stod(body.substr(0, comma));
    const double y = std::stod(body.substr(comma + 1));
    return std::make_pair(x, y);
  } catch (const std::exception &) {
    return std::nullopt;
  }
}

}  // namespace

void CFPA2Coordinator::apply_fast_blacklist(
    const std::string & ns,
    const std::map<std::string, std::string> & payload)
{
  const auto state_it = payload.find("state");
  if (state_it == payload.end()) return;
  const std::string & state = state_it->second;
  if (state != "unreachable" && state != "failed") return;

  const auto now_ns = clock_facade_->now_ns();

  // 1) goal_seq dedup — skip if we already acted on this exact seq.
  int goal_seq = -1;
  const auto seq_it = payload.find("goal_seq");
  if (seq_it != payload.end()) {
    try { goal_seq = std::stoi(seq_it->second); } catch (...) { goal_seq = -1; }
    const auto last_seq_it = last_unreachable_goal_seq_.find(ns);
    if (goal_seq >= 0 && last_seq_it != last_unreachable_goal_seq_.end() &&
        last_seq_it->second == goal_seq)
    {
      return;
    }
  }

  // 2) Match reported goal to CFPA2's last-assigned via quantised key —
  //    stale status reports referring to old goals are discarded.
  const auto last_it = last_goal_.find(ns);
  if (last_it == last_goal_.end()) return;
  const auto goal_field_it = payload.find("goal");
  if (goal_field_it == payload.end()) return;
  const auto reported = parse_xy_pair(goal_field_it->second);
  if (!reported.has_value()) return;
  if (goal_key(*reported) != goal_key(last_it->second)) return;

  // 3) Startup grace.
  const double elapsed_sec = static_cast<double>(now_ns - node_start_ns_) / 1e9;
  if (elapsed_sec < fast_unreachable_startup_grace_sec_) {
    CFPA2_LOG_INFO(log_facade_,
        "%s: skipping fast-BL goal=(%.2f,%.2f) — startup grace (%.1f/%.1fs)",
        ns.c_str(), last_it->second.first, last_it->second.second,
        elapsed_sec, fast_unreachable_startup_grace_sec_);
    return;
  }

  // 4) Consecutive-threshold debounce.
  const auto k = goal_key(last_it->second);
  auto & consec = unreachable_consec_[ns];
  const int new_count = ++consec[k];
  if (new_count < fast_unreachable_consecutive_threshold_) {
    CFPA2_LOG_INFO(log_facade_,
        "%s: pending fast-BL goal=(%.2f,%.2f) — consec %d/%d",
        ns.c_str(), last_it->second.first, last_it->second.second,
        new_count, fast_unreachable_consecutive_threshold_);
    return;
  }

  // 5) Latch the blacklist.
  const std::uint64_t bl_until_ns =
      now_ns + static_cast<std::uint64_t>(fast_unreachable_blacklist_sec_ * 1e9);
  auto & bl = goal_blacklist_until_ns_[ns];
  const auto kit = bl.find(k);
  bl[k] = (kit == bl.end()) ? bl_until_ns : std::max(kit->second, bl_until_ns);
  consec[k] = 0;
  add_blacklist_disk(ns, last_it->second, bl_until_ns, blacklist_cluster_radius_m_);
  goal_fail_counts_[ns][k] = 0;
  goal_progress_samples_[ns].clear();
  if (goal_seq >= 0) last_unreachable_goal_seq_[ns] = goal_seq;

  const auto reason_it = payload.find("reason");
  const auto source_it = payload.find("source");
  const std::string reason = reason_it == payload.end() ? "?" : reason_it->second;
  const std::string source = source_it == payload.end() ? "?" : source_it->second;
  CFPA2_LOG_WARN(log_facade_,
      "%s: FAST-BL goal=(%.2f,%.2f) state=%s reason=%s src=%s ttl=%.1fs",
      ns.c_str(), last_it->second.first, last_it->second.second,
      state.c_str(), reason.c_str(), source.c_str(),
      fast_unreachable_blacklist_sec_);
}

// ───────────────────────────────────────────────────────────────────
//  Trajectory append (used by odom_cb)
// ───────────────────────────────────────────────────────────────────
void CFPA2Coordinator::append_trajectory(
    const std::string & ns, double x, double y)
{
  auto & traj = trajectory_history_[ns];
  if (!traj.empty()) {
    const auto & last = traj.back();
    const double dx = x - last.first;
    const double dy = y - last.second;
    if (dx * dx + dy * dy < trajectory_min_point_distance_ * trajectory_min_point_distance_) {
      return;
    }
  }
  traj.push_back({x, y});
  while (static_cast<int>(traj.size()) > trajectory_max_points_) traj.pop_front();
}

// ───────────────────────────────────────────────────────────────────
//  Convenience helpers
// ───────────────────────────────────────────────────────────────────

Goal CFPA2Coordinator::robot_xy(const std::string & ns) const
{
  auto it = odoms_.find(ns);
  if (it == odoms_.end()) return {0.0, 0.0};
  return {it->second.x, it->second.y};
}

std::optional<std::pair<int, int>> CFPA2Coordinator::world_to_grid(
    const core::Grid & msg, double wx, double wy) const
{
  const double res = msg.info.resolution;
  if (res <= 0.0) return std::nullopt;
  const int gx = static_cast<int>((wx - msg.info.origin_x) / res);
  const int gy = static_cast<int>((wy - msg.info.origin_y) / res);
  if (gx < 0 || gy < 0 ||
      gx >= static_cast<int>(msg.info.width) ||
      gy >= static_cast<int>(msg.info.height))
  {
    return std::nullopt;
  }
  return std::make_pair(gx, gy);
}

std::pair<double, double> CFPA2Coordinator::grid_to_world(
    const core::Grid & msg, int gx, int gy) const
{
  return {
      msg.info.origin_x + (gx + 0.5) * msg.info.resolution,
      msg.info.origin_y + (gy + 0.5) * msg.info.resolution,
  };
}

std::vector<std::pair<int, int>> CFPA2Coordinator::grid_line_cells(
    int x0, int y0, int x1, int y1)
{
  // Bresenham line algorithm, inclusive endpoints.
  std::vector<std::pair<int, int>> out;
  int dx = std::abs(x1 - x0);
  int dy = std::abs(y1 - y0);
  int sx = x0 < x1 ? 1 : -1;
  int sy = y0 < y1 ? 1 : -1;
  int err = dx - dy;
  int x = x0;
  int y = y0;
  while (true) {
    out.push_back({x, y});
    if (x == x1 && y == y1) break;
    int e2 = 2 * err;
    if (e2 > -dy) {
      err -= dy;
      x += sx;
    }
    if (e2 < dx) {
      err += dx;
      y += sy;
    }
  }
  return out;
}

bool CFPA2Coordinator::is_free_idx(const std::vector<std::int8_t> & data, int idx) const
{
  if (idx < 0 || idx >= static_cast<int>(data.size())) return false;
  const std::int8_t v = data[idx];
  return v != unknown_value_ && v >= 0 && v < occ_thresh_;
}

bool CFPA2Coordinator::is_unknown_idx(const std::vector<std::int8_t> & data, int idx) const
{
  if (idx < 0 || idx >= static_cast<int>(data.size())) return false;
  return data[idx] == unknown_value_;
}

bool CFPA2Coordinator::has_frontier_obstacle_clearance(
    const std::vector<std::int8_t> & data,
    int gx, int gy, int w, int h, int clearance_cells) const
{
  if (clearance_cells <= 0) return true;
  for (int dy = -clearance_cells; dy <= clearance_cells; ++dy) {
    const int ny = gy + dy;
    if (ny < 0 || ny >= h) continue;
    for (int dx = -clearance_cells; dx <= clearance_cells; ++dx) {
      const int nx = gx + dx;
      if (nx < 0 || nx >= w) continue;
      if (data[ny * w + nx] >= occ_thresh_) return false;
    }
  }
  return true;
}

bool CFPA2Coordinator::goal_has_obstacle_clearance(
    const core::Grid & msg, double wx, double wy,
    double clearance_m) const
{
  if (clearance_m <= 0.0) return true;
  const double res = msg.info.resolution;
  if (res <= 0.0) return true;
  const auto g = world_to_grid(msg, wx, wy);
  if (!g.has_value()) return false;
  const int cells = static_cast<int>(std::ceil(clearance_m / res));
  return has_frontier_obstacle_clearance(
      msg.data, g->first, g->second,
      static_cast<int>(msg.info.width),
      static_cast<int>(msg.info.height),
      cells);
}

// ───────────────────────────────────────────────────────────────────
//  GoalKey + blacklist primitives
// ───────────────────────────────────────────────────────────────────

GoalKey CFPA2Coordinator::goal_key(Goal goal) const
{
  const double r = blacklist_key_resolution_;
  return {
      static_cast<int>(std::round(goal.first / r)),
      static_cast<int>(std::round(goal.second / r)),
  };
}

void CFPA2Coordinator::prune_blacklist(const std::string & ns, std::uint64_t now_ns)
{
  auto & cell_bl = goal_blacklist_until_ns_[ns];
  for (auto it = cell_bl.begin(); it != cell_bl.end();) {
    if (it->second <= now_ns) it = cell_bl.erase(it);
    else ++it;
  }
  auto & disks = goal_blacklist_disks_[ns];
  disks.erase(
      std::remove_if(disks.begin(), disks.end(),
          [now_ns](const BlacklistDisk & d) { return d.until_ns <= now_ns; }),
      disks.end());
}

bool CFPA2Coordinator::is_blacklisted(
    const std::string & ns, Goal goal, std::uint64_t now_ns) const
{
  const auto cell_it = goal_blacklist_until_ns_.find(ns);
  if (cell_it != goal_blacklist_until_ns_.end()) {
    const auto k = goal_key(goal);
    const auto kit = cell_it->second.find(k);
    if (kit != cell_it->second.end() && kit->second > now_ns) return true;
  }
  const auto disks_it = goal_blacklist_disks_.find(ns);
  if (disks_it != goal_blacklist_disks_.end()) {
    for (const auto & d : disks_it->second) {
      if (d.until_ns <= now_ns) continue;
      const double dx = goal.first - d.x;
      const double dy = goal.second - d.y;
      if (dx * dx + dy * dy <= d.radius_m * d.radius_m) return true;
    }
  }
  return false;
}

void CFPA2Coordinator::add_blacklist_disk(
    const std::string & ns, Goal goal,
    std::uint64_t until_ns, double radius_m)
{
  if (radius_m <= 0.0) return;
  goal_blacklist_disks_[ns].push_back({goal.first, goal.second, radius_m, until_ns});
}

void CFPA2Coordinator::register_goal_failure(
    const std::string & ns, Goal goal,
    std::uint64_t now_ns, const std::string & /*reason*/)
{
  const auto k = goal_key(goal);
  auto & counts = goal_fail_counts_[ns];
  const int c = ++counts[k];
  if (c >= blacklist_fail_count_) {
    const std::uint64_t ttl_ns = static_cast<std::uint64_t>(blacklist_ttl_sec_ * 1e9);
    goal_blacklist_until_ns_[ns][k] = now_ns + ttl_ns;
    counts.erase(k);
    add_blacklist_disk(ns, goal, now_ns + ttl_ns, blacklist_cluster_radius_m_);
  }
}

std::set<std::string> CFPA2Coordinator::consume_local_nav_stall_blacklists(
    std::uint64_t /*now_ns*/)
{
  // TODO: full impl. For now return empty set; the slower fail-count path
  // already handles repeated nav failures.
  return {};
}

void CFPA2Coordinator::update_reached_goal_blacklist(
    const std::string & ns, std::uint64_t now_ns)
{
  const auto goal_it = last_goal_.find(ns);
  if (goal_it == last_goal_.end()) return;
  if (!goal_satisfied(ns, goal_it->second, goal_satisfaction_map(ns))) return;

  const auto k = goal_key(goal_it->second);
  auto & last_key = reached_goal_last_key_[ns];
  auto & repeat = reached_goal_repeat_count_[ns];
  if (last_key.has_value() && *last_key == k) {
    ++repeat;
  } else {
    last_key = k;
    repeat = 1;
  }
  if (repeat >= reached_blacklist_repeat_count_) {
    const std::uint64_t ttl_ns =
        static_cast<std::uint64_t>(reached_blacklist_ttl_sec_ * 1e9);
    goal_blacklist_until_ns_[ns][k] = now_ns + ttl_ns;
    add_blacklist_disk(ns, goal_it->second, now_ns + ttl_ns,
        std::max(reached_blacklist_dist_, blacklist_cluster_radius_m_));
    repeat = 0;
    last_key.reset();
  }
}

void CFPA2Coordinator::blacklist_active_goal(
    const std::string & ns, std::uint64_t now_ns, const std::string & /*reason*/)
{
  const auto it = last_goal_.find(ns);
  if (it == last_goal_.end()) return;
  const auto k = goal_key(it->second);
  const std::uint64_t ttl_ns = static_cast<std::uint64_t>(blacklist_ttl_sec_ * 1e9);
  goal_blacklist_until_ns_[ns][k] = now_ns + ttl_ns;
  add_blacklist_disk(ns, it->second, now_ns + ttl_ns, blacklist_cluster_radius_m_);
}

// ───────────────────────────────────────────────────────────────────
//  Goal predicates
// ───────────────────────────────────────────────────────────────────

double CFPA2Coordinator::distance_robot_to_goal(const std::string & ns, Goal goal) const
{
  const auto r = robot_xy(ns);
  const double dx = goal.first - r.first;
  const double dy = goal.second - r.second;
  return std::sqrt(dx * dx + dy * dy);
}

bool CFPA2Coordinator::goal_too_close(const std::string & ns, Goal goal) const
{
  return distance_robot_to_goal(ns, goal) < min_assign_distance_;
}

bool CFPA2Coordinator::goal_too_far(const std::string & ns, Goal goal) const
{
  if (cfpa2_max_goal_distance_m_ <= 0.0) return false;
  return distance_robot_to_goal(ns, goal) > cfpa2_max_goal_distance_m_;
}

bool CFPA2Coordinator::goals_equivalent(Goal a, Goal b) const
{
  const double dx = a.first - b.first;
  const double dy = a.second - b.second;
  const double r = std::max(blacklist_key_resolution_, 0.25);
  return dx * dx + dy * dy <= r * r;
}

bool CFPA2Coordinator::goal_reachable(
    const core::Grid & msg,
    const std::unordered_map<int, int> & dist_map, Goal goal) const
{
  const auto g = world_to_grid(msg, goal.first, goal.second);
  if (!g.has_value()) return false;
  return dist_map.find(g->second * static_cast<int>(msg.info.width) + g->first) !=
      dist_map.end();
}

bool CFPA2Coordinator::goal_line_of_sight_clear(
    const core::Grid & map_msg, Goal robot_w, Goal goal_w) const
{
  const auto a = world_to_grid(map_msg, robot_w.first, robot_w.second);
  const auto b = world_to_grid(map_msg, goal_w.first, goal_w.second);
  if (!a.has_value() || !b.has_value()) return false;
  const int w = static_cast<int>(map_msg.info.width);
  for (const auto & [x, y] : grid_line_cells(a->first, a->second, b->first, b->second)) {
    const int idx = y * w + x;
    if (idx < 0 || idx >= static_cast<int>(map_msg.data.size())) return false;
    if (map_msg.data[idx] >= occ_thresh_) return false;
  }
  return true;
}

const core::Grid * CFPA2Coordinator::goal_satisfaction_map(
    const std::string & ns) const
{
  auto it = maps_.find(ns);
  if (it != maps_.end()) return &it->second;
  if (cur_planning_map_.has_value()) return &(*cur_planning_map_);
  return nullptr;
}

bool CFPA2Coordinator::goal_satisfied(
    const std::string & ns, Goal goal,
    const core::Grid * map_msg) const
{
  const double d = distance_robot_to_goal(ns, goal);
  if (d <= goal_satisfied_direct_dist_) return true;
  if (d > goal_satisfied_dist_) return false;
  if (!goal_satisfied_requires_los_) return true;
  const auto * mm = map_msg ? map_msg : goal_satisfaction_map(ns);
  if (!mm) return true;
  return goal_line_of_sight_clear(*mm, robot_xy(ns), goal);
}

std::optional<std::string> CFPA2Coordinator::held_goal_safety_failure(
    const core::Grid & msg,
    const std::unordered_map<int, int> & dist_map, Goal goal) const
{
  if (!goal_reachable(msg, dist_map, goal)) return std::string("unreachable");
  if (!goal_has_obstacle_clearance(
        msg, goal.first, goal.second, cfpa2_goal_obstacle_clearance_m_))
  {
    return std::string("unsafe_clearance");
  }
  return std::nullopt;
}

// ───────────────────────────────────────────────────────────────────
//  Map ops (wrappers around cfpa2::ops::*)
// ───────────────────────────────────────────────────────────────────

int CFPA2Coordinator::frontier_raw_capacity(int width, int height, int max_targets) const
{
  const int map_cells = std::max(1, width * height);
  const int factor = std::max(1, frontier_raw_overfetch_factor_);
  const int minimum = std::max(max_targets, frontier_raw_overfetch_min_);
  const int requested = std::max({max_targets, minimum, max_targets * factor});
  return std::min(map_cells, requested);
}

std::vector<Goal> CFPA2Coordinator::extract_frontiers(
    const core::Grid & msg)
{
  const int w = static_cast<int>(msg.info.width);
  const int h = static_cast<int>(msg.info.height);
  const double res = std::max(1e-6, static_cast<double>(msg.info.resolution));
  const int stride = std::max(1, adaptive_frontier_stride_);
  const int max_targets = adaptive_max_targets_;
  const int raw_cap = frontier_raw_capacity(w, h, max_targets);
  const int clearance_cells = static_cast<int>(
      std::ceil(cfpa2_frontier_obstacle_clearance_m_ / res));

  std::vector<float> out_x(raw_cap);
  std::vector<float> out_y(raw_cap);
  const int n_raw = ops::extract_frontiers(
      msg.data.data(), w, h,
      static_cast<float>(res),
      static_cast<float>(msg.info.origin_x),
      static_cast<float>(msg.info.origin_y),
      stride,
      static_cast<float>(cfpa2_frontier_min_cluster_area_m2_),
      clearance_cells,
      free_value_, unknown_value_, occ_thresh_,
      out_x.data(), out_y.data(),
      raw_cap);

  std::vector<Goal> raw_goals;
  raw_goals.reserve(n_raw);
  for (int i = 0; i < n_raw; ++i) {
    raw_goals.emplace_back(static_cast<double>(out_x[i]), static_cast<double>(out_y[i]));
  }
  auto clustered = cluster_representatives(raw_goals, cfpa2_frontier_cluster_radius_m_);
  auto live = filter_dead_frontiers(clustered, msg);
  if (static_cast<int>(live.size()) > max_targets) live.resize(max_targets);
  return live;
}

std::vector<Goal> CFPA2Coordinator::cluster_representatives(
    const std::vector<Goal> & points, double cluster_radius_m)
{
  if (points.empty() || cluster_radius_m <= 0.0) return points;
  std::vector<float> in_x(points.size());
  std::vector<float> in_y(points.size());
  for (std::size_t i = 0; i < points.size(); ++i) {
    in_x[i] = static_cast<float>(points[i].first);
    in_y[i] = static_cast<float>(points[i].second);
  }
  std::vector<float> out_x(points.size());
  std::vector<float> out_y(points.size());
  const int n_out = ops::cluster_representatives(
      in_x.data(), in_y.data(), static_cast<int>(points.size()),
      static_cast<float>(cluster_radius_m),
      out_x.data(), out_y.data());
  std::vector<Goal> reps;
  reps.reserve(n_out);
  for (int i = 0; i < n_out; ++i) {
    reps.emplace_back(static_cast<double>(out_x[i]), static_cast<double>(out_y[i]));
  }
  return reps;
}

std::vector<Goal> CFPA2Coordinator::filter_dead_frontiers(
    const std::vector<Goal> & points, const core::Grid & msg)
{
  if (points.empty() || cfpa2_frontier_min_unknown_cells_ <= 0) return points;
  const int w = static_cast<int>(msg.info.width);
  const int h = static_cast<int>(msg.info.height);
  const double res = msg.info.resolution;
  if (res <= 0.0) return points;

  std::vector<float> in_x(points.size());
  std::vector<float> in_y(points.size());
  for (std::size_t i = 0; i < points.size(); ++i) {
    in_x[i] = static_cast<float>(points[i].first);
    in_y[i] = static_cast<float>(points[i].second);
  }
  std::vector<float> out_x(points.size());
  std::vector<float> out_y(points.size());
  const int r_cells = std::max(
      1, static_cast<int>(std::round(cfpa2_frontier_unknown_check_radius_m_ / res)));
  const int n_out = ops::filter_dead_frontiers(
      msg.data.data(), w, h,
      static_cast<float>(res),
      static_cast<float>(msg.info.origin_x),
      static_cast<float>(msg.info.origin_y),
      occ_thresh_, r_cells, cfpa2_frontier_min_unknown_cells_,
      in_x.data(), in_y.data(), static_cast<int>(points.size()),
      out_x.data(), out_y.data());
  std::vector<Goal> live;
  live.reserve(n_out);
  for (int i = 0; i < n_out; ++i) {
    live.emplace_back(static_cast<double>(out_x[i]), static_cast<double>(out_y[i]));
  }
  return live;
}

std::vector<Goal> CFPA2Coordinator::merge_targets(
    const std::vector<std::vector<Goal>> & target_lists, double merge_resolution)
{
  std::vector<Goal> merged;
  if (merge_resolution <= 0.0) {
    for (const auto & list : target_lists) {
      merged.insert(merged.end(), list.begin(), list.end());
    }
    return merged;
  }
  std::set<std::pair<int, int>> seen;
  for (const auto & list : target_lists) {
    for (const auto & g : list) {
      std::pair<int, int> key {
          static_cast<int>(std::round(g.first / merge_resolution)),
          static_cast<int>(std::round(g.second / merge_resolution))};
      if (seen.insert(key).second) merged.push_back(g);
    }
  }
  return merged;
}

std::unordered_map<int, int> CFPA2Coordinator::distance_transform(
    const core::Grid & msg, std::pair<double, double> start_w)
{
  const int w = static_cast<int>(msg.info.width);
  const int h = static_cast<int>(msg.info.height);
  const auto g = world_to_grid(msg, start_w.first, start_w.second);
  std::unordered_map<int, int> dist_map;
  if (!g.has_value()) return dist_map;
  std::vector<int> dist(w * h, -1);
  const std::int8_t unk = cfpa2_reachability_allow_unknown_
      ? static_cast<std::int8_t>(-127) : unknown_value_;
  ops::distance_transform_range(
      msg.data.data(), w, h,
      g->first, g->second,
      unk, cfpa2_reachability_occ_threshold_, dist.data());
  for (int i = 0; i < w * h; ++i) {
    if (dist[i] >= 0) dist_map[i] = dist[i];
  }
  return dist_map;
}

// ───────────────────────────────────────────────────────────────────
//  Information gain
// ───────────────────────────────────────────────────────────────────

double CFPA2Coordinator::frontier_information_gain(
    const core::Grid & msg, Goal goal) const
{
  if (cfpa2_ig_mode_ == "floodfill") {
    return frontier_information_gain_floodfill(msg, goal);
  }
  const auto g = world_to_grid(msg, goal.first, goal.second);
  if (!g.has_value()) return 0.0;
  const int w = static_cast<int>(msg.info.width);
  const int h = static_cast<int>(msg.info.height);
  const int r = adaptive_exploration_gain_radius_cells_;
  const int x0 = std::max(0, g->first - r);
  const int x1 = std::min(w, g->first + r + 1);
  const int y0 = std::max(0, g->second - r);
  const int y1 = std::min(h, g->second + r + 1);
  double gain = 0.0;
  for (int yy = y0; yy < y1; ++yy) {
    const int row = yy * w;
    for (int xx = x0; xx < x1; ++xx) {
      if (msg.data[row + xx] == unknown_value_) gain += 1.0;
    }
  }
  return gain;
}

double CFPA2Coordinator::frontier_information_gain_floodfill(
    const core::Grid & msg, Goal goal) const
{
  // Stub — batch path used in tick. Returns 0 for the single-call path so
  // downstream callers fall back. Full impl uses ops::batch_info_gain_floodfill.
  (void)msg;
  (void)goal;
  return 0.0;
}

std::vector<double> CFPA2Coordinator::batch_frontier_information_gain(
    const core::Grid & msg, const std::vector<Goal> & goals)
{
  std::vector<double> out(goals.size(), 0.0);
  if (goals.empty()) return out;
  const int w = static_cast<int>(msg.info.width);
  const int h = static_cast<int>(msg.info.height);
  const int n = static_cast<int>(goals.size());

  std::vector<float> gx(n);
  std::vector<float> gy(n);
  for (int i = 0; i < n; ++i) {
    gx[i] = static_cast<float>(goals[i].first);
    gy[i] = static_cast<float>(goals[i].second);
  }
  std::vector<float> gains(n);

  if (cfpa2_ig_mode_ == "floodfill") {
    if (static_cast<int>(floodfill_scratch_.size()) < w * h) {
      floodfill_scratch_.assign(w * h, 0);
    }
    ops::batch_info_gain_floodfill(
        msg.data.data(), w, h,
        static_cast<float>(msg.info.resolution),
        static_cast<float>(msg.info.origin_x),
        static_cast<float>(msg.info.origin_y),
        gx.data(), gy.data(), n,
        cfpa2_floodfill_budget_, cfpa2_floodfill_max_radius_cells_,
        unknown_value_, floodfill_scratch_.data(), gains.data());
  } else {
    ops::batch_info_gain(
        msg.data.data(), w, h,
        static_cast<float>(msg.info.resolution),
        static_cast<float>(msg.info.origin_x),
        static_cast<float>(msg.info.origin_y),
        gx.data(), gy.data(), n,
        adaptive_exploration_gain_radius_cells_,
        unknown_value_, gains.data());
  }
  for (int i = 0; i < n; ++i) out[i] = static_cast<double>(gains[i]);
  return out;
}

std::optional<double> CFPA2Coordinator::grid_path_cost_m(
    const core::Grid & msg,
    const std::unordered_map<int, int> & dist_map, Goal goal) const
{
  const auto g = world_to_grid(msg, goal.first, goal.second);
  if (!g.has_value()) return std::nullopt;
  const int w = static_cast<int>(msg.info.width);
  const auto it = dist_map.find(g->second * w + g->first);
  if (it == dist_map.end()) return std::nullopt;
  return static_cast<double>(it->second) * static_cast<double>(msg.info.resolution);
}

// ───────────────────────────────────────────────────────────────────
//  Utility scoring
// ───────────────────────────────────────────────────────────────────

double CFPA2Coordinator::cfpa2_overlap_penalty(Goal goal_i, Goal goal_j) const
{
  const double sigma = cfpa2_sigma_overlap_m_ > 0.0
      ? cfpa2_sigma_overlap_m_ : (2.0 * sensor_range_);
  const double s = std::max(1e-3, sigma);
  const double dx = goal_i.first - goal_j.first;
  const double dy = goal_i.second - goal_j.second;
  const double d2 = dx * dx + dy * dy;
  return std::exp(-d2 / (2.0 * s * s));
}

double CFPA2Coordinator::cfpa2_switch_penalty(const std::string & ns, Goal goal) const
{
  const auto it = last_goal_.find(ns);
  if (it == last_goal_.end()) return 0.0;
  const double dx = goal.first - it->second.first;
  const double dy = goal.second - it->second.second;
  return std::sqrt(dx * dx + dy * dy);
}

double CFPA2Coordinator::cfpa2_momentum(const std::string & ns) const
{
  const auto it = odom_velocity_xy_.find(ns);
  if (it == odom_velocity_xy_.end()) return 0.0;
  return std::sqrt(it->second.first * it->second.first +
                   it->second.second * it->second.second);
}

double CFPA2Coordinator::cfpa2_momentum_bonus(const std::string & ns, Goal goal) const
{
  const auto vit = odom_velocity_xy_.find(ns);
  const double speed = vit != odom_velocity_xy_.end()
      ? std::sqrt(vit->second.first * vit->second.first +
                  vit->second.second * vit->second.second)
      : 0.0;
  const auto r = robot_xy(ns);
  const double dx = goal.first - r.first;
  const double dy = goal.second - r.second;
  const double dn = std::sqrt(dx * dx + dy * dy);
  if (dn < 1e-6) return 0.0;
  double cos_h = 0.0;
  if (vit != odom_velocity_xy_.end() && speed > 1e-6) {
    cos_h = (vit->second.first * dx + vit->second.second * dy) / (speed * dn);
  } else {
    // No velocity → no momentum bonus.
    cos_h = 0.0;
  }
  return cos_h * (cfpa2_momentum_alpha_ + cfpa2_momentum_beta_ * speed);
}

double CFPA2Coordinator::cfpa2_single_utility(
    const std::string & ns, Goal goal,
    const core::Grid & map_msg,
    const std::unordered_map<int, int> & dist_map)
{
  const auto dist_m = grid_path_cost_m(map_msg, dist_map, goal);
  if (!dist_m.has_value() || *dist_m <= 0.0) return -1e18;
  const double info_gain = frontier_information_gain(map_msg, goal);
  if (info_gain < 3.0) return -1e18;
  const double sw = cfpa2_switch_penalty(ns, goal);
  const double mom = cfpa2_momentum_bonus(ns, goal);
  return (cfpa2_w_ig_ * info_gain)
       - (cfpa2_w_c_ * *dist_m)
       - (cfpa2_w_sw_ * sw)
       + (cfpa2_w_momentum_ * mom);
}

// ───────────────────────────────────────────────────────────────────
//  Tick — minimal Phase B skeleton; full logic will land progressively.
// ───────────────────────────────────────────────────────────────────

void CFPA2Coordinator::tick()
{
  if (paused_) return;
  if (adaptive_skip_ticks_ > 0) {
    if (++adaptive_tick_skip_counter_ <= adaptive_skip_ticks_) return;
    adaptive_tick_skip_counter_ = 0;
  }
  const auto t0 = clock_facade_->now_ns();
  try {
    tick_impl();
  } catch (const std::exception & e) {
    CFPA2_LOG_ERROR(log_facade_,"tick_impl raised: %s", e.what());
  }
  record_tick_perf(t0);
  if (perf_enable_) maybe_log_perf_summary();
}

void CFPA2Coordinator::tick_impl()
{
  const auto now_ns = clock_facade_->now_ns();
  // Wait for all maps; pick first as viz reference.
  for (const auto & ns : namespaces_) {
    if (maps_.find(ns) == maps_.end()) {
      if (now_ns - last_prereq_warn_ns_ > static_cast<std::uint64_t>(2e9)) {
        CFPA2_LOG_WARN(log_facade_,"Waiting for map topics from at least one of %zu namespaces",
            namespaces_.size());
        last_prereq_warn_ns_ = now_ns;
      }
      return;
    }
  }
  const auto & planning_map = maps_[namespaces_.front()];
  cur_planning_map_ = planning_map;

  publish_coordinator_map(planning_map);
  publish_robot_markers(planning_map);

  // Per-ns frontier extraction.
  std::unordered_map<std::string, std::vector<Goal>> per_ns_targets;
  for (const auto & ns : namespaces_) {
    if (maps_.find(ns) != maps_.end()) {
      per_ns_targets[ns] = extract_frontiers(maps_[ns]);
    }
  }

  const double merge_res = std::max(0.1, planning_map.info.resolution * 2.0);
  std::vector<std::vector<Goal>> per_ns_lists;
  per_ns_lists.reserve(namespaces_.size());
  for (const auto & ns : namespaces_) per_ns_lists.push_back(per_ns_targets[ns]);
  std::vector<Goal> targets = merge_targets(per_ns_lists, merge_res);
  publish_frontier_markers(planning_map, targets);

  if (targets.empty()) return;

  // Reached-goal blacklist housekeeping — must run BEFORE candidate
  // selection so the freshly-blacklisted goal can't re-enter the pool.
  for (const auto & ns : namespaces_) {
    if (odoms_.find(ns) == odoms_.end()) continue;
    update_reached_goal_blacklist(ns, now_ns);
  }

  // Consume any local nav stall events queued since last tick.
  const auto forced_switch_set = consume_local_nav_stall_blacklists(now_ns);

  // Pass 1: per-ns utility lists (filtered + scored, sorted desc).
  std::unordered_map<std::string, UtilityList> utils_by_ns;
  std::unordered_map<std::string, bool> forced_goal_by_ns;
  std::unordered_map<std::string, std::unordered_map<int, int>> dist_maps_by_ns;
  for (const auto & ns : namespaces_) {
    if (odoms_.find(ns) == odoms_.end()) continue;
    const auto r = robot_xy(ns);
    auto dist_map = distance_transform(maps_[ns], r);

    prune_blacklist(ns, now_ns);
    UtilityList util;
    util.reserve(targets.size());
    int rej_peer = 0, rej_bl = 0, rej_reach = 0, rej_clear = 0, rej_util = 0;
    double best_reach_ig = 0.0;
    // Primary pass: only REAL reachable frontiers (keeps the fast beeline —
    // the robot commits to its connected reachable region's best frontier).
    for (const auto & g : targets) {
      if (is_goal_peer_claimed(g)) { ++rej_peer; continue; }
      if (is_blacklisted(ns, g, now_ns)) { ++rej_bl; continue; }
      if (!goal_reachable(maps_[ns], dist_map, g)) { ++rej_reach; continue; }
      if (!goal_has_obstacle_clearance(maps_[ns], g.first, g.second,
            cfpa2_goal_obstacle_clearance_m_)) { ++rej_clear; continue; }
      const double u = cfpa2_single_utility(ns, g, maps_[ns], dist_map);
      if (u < cfpa2_min_utility_) { ++rej_util; continue; }
      util.push_back({g, u});
      const double ig = frontier_information_gain(maps_[ns], g);
      if (ig > best_reach_ig) best_reach_ig = ig;
    }

    bool used_fallback = false;
    // ── EXTENT-SEEK (highest priority): bidirectional ±x coverage ──────────
    // Track the robot's PHYSICALLY traversed x-extent. Once one extreme is
    // reached (|x| ≥ extent_target_x) but the other isn't, commit to the
    // frontier furthest toward the unreached extreme (reachable → drive there
    // directly; else snap the furthest unreachable frontier to its nearest
    // reachable cell). Fires at extent_target_x, BELOW the far-corner wedge
    // point, so the robot turns around while still mobile and heads to the
    // other arm — directly satisfying the "span +35 → −35" goal instead of
    // exhausting/wedging one arm. A single dominating goal = stable (no thrash).
    if (cfpa2_extent_seek_enabled_) {
      const auto rr = robot_xy(ns);
      double & rxmin = reached_x_min_.try_emplace(ns, rr.first).first->second;
      double & rxmax = reached_x_max_.try_emplace(ns, rr.first).first->second;
      rxmin = std::min(rxmin, rr.first);
      rxmax = std::max(rxmax, rr.first);
      int seek_dir = 0;
      const bool plus_done = rxmax >= cfpa2_extent_target_x_;
      const bool minus_done = rxmin <= -cfpa2_extent_target_x_;
      if (plus_done && !minus_done) seek_dir = -1;
      else if (minus_done && !plus_done) seek_dir = +1;
      if (seek_dir != 0) {
        bool have = false; Goal pick{0.0, 0.0}; bool pick_reach = false;
        double ext = (seek_dir < 0) ? 1e18 : -1e18;
        // DIRECTIONAL GUARD: only consider goals that are actually in the seek
        // direction RELATIVE TO THE ROBOT. Without this, "min-x reachable
        // frontier" picks a +x frontier (e.g. +10.25) when ALL −x frontiers are
        // unreachable, sending the robot the wrong way → it alternates with the
        // snapped −x goal every tick (run27 GOAL_SENT thrash −8.35 ↔ +10.25) and
        // never commits. With the guard, only −x-of-robot goals qualify, so the
        // snapped −x goal wins consistently → stable single goal → robot drives
        // −x. Among qualifying candidates pick the one FURTHEST in seek dir.
        auto consider = [&](const Goal & g, bool reach) {
          if (seek_dir < 0 && g.first >= rr.first) return;   // not −x of robot
          if (seek_dir > 0 && g.first <= rr.first) return;   // not +x of robot
          if ((seek_dir < 0 && g.first < ext) ||
              (seek_dir > 0 && g.first > ext)) {
            ext = g.first; pick = g; have = true; pick_reach = reach;
          }
        };
        // (1) Reachable frontiers that lie in the seek direction.
        for (const auto & g : targets) {
          if (is_goal_peer_claimed(g) || is_blacklisted(ns, g, now_ns)) continue;
          if (frontier_information_gain(maps_[ns], g) < 3.0) continue;
          if (!goal_reachable(maps_[ns], dist_map, g)) continue;
          consider(g, true);
        }
        // (2) The furthest-in-seek-dir UNREACHABLE frontier, snapped to its
        //     nearest reachable cell (lets the robot push toward an unentered
        //     corridor even when no reachable frontier lies that way).
        {
          double uext = (seek_dir < 0) ? 1e18 : -1e18; Goal uf{0.0, 0.0};
          bool huf = false;
          for (const auto & g : targets) {
            if (is_goal_peer_claimed(g) || is_blacklisted(ns, g, now_ns)) continue;
            if (frontier_information_gain(maps_[ns], g) < 3.0) continue;
            if (goal_reachable(maps_[ns], dist_map, g)) continue;
            if ((seek_dir < 0 && g.first < uext) ||
                (seek_dir > 0 && g.first > uext)) {
              uext = g.first; uf = g; huf = true;
            }
          }
          if (huf) {
            const int gw = static_cast<int>(maps_[ns].info.width);
            const double gres =
                std::max(1e-3, static_cast<double>(maps_[ns].info.resolution));
            const int smr = static_cast<int>(80.0 / gres);
            const long smr2 = static_cast<long>(smr) * static_cast<long>(smr);
            const auto gg = world_to_grid(maps_[ns], uf.first, uf.second);
            if (gg.has_value()) {
              const int gi = gg->first, gj = gg->second;
              long bd = smr2 + 1; int bidx = -1;
              for (const auto & kv : dist_map) {
                const int ci = kv.first % gw, cj = kv.first / gw;
                const long d2 = static_cast<long>(ci - gi) * (ci - gi) +
                                static_cast<long>(cj - gj) * (cj - gj);
                if (d2 < bd) { bd = d2; bidx = kv.first; }
              }
              if (bidx >= 0) {
                consider(Goal{
                    static_cast<double>(maps_[ns].info.origin_x) +
                        (bidx % gw + 0.5) * gres,
                    static_cast<double>(maps_[ns].info.origin_y) +
                        (bidx / gw + 0.5) * gres}, false);
              }
            }
          }
        }
        // (3) Guaranteed candidate: the furthest-in-seek-dir REACHABLE cell
        //     from dist_map (raw cell, never a blacklisted frontier) so
        //     extent-seek ALWAYS has a goal in the seek direction and can never
        //     fall through to a +x primary frontier while seeking −x.
        {
          const int gw = static_cast<int>(maps_[ns].info.width);
          const double gres =
              std::max(1e-3, static_cast<double>(maps_[ns].info.resolution));
          double cell_ext = (seek_dir < 0) ? 1e18 : -1e18; int cidx = -1;
          for (const auto & kv : dist_map) {
            const double wx = static_cast<double>(maps_[ns].info.origin_x) +
                (kv.first % gw + 0.5) * gres;
            if (seek_dir < 0 ? wx < cell_ext : wx > cell_ext) {
              cell_ext = wx; cidx = kv.first;
            }
          }
          if (cidx >= 0) {
            consider(Goal{
                static_cast<double>(maps_[ns].info.origin_x) +
                    (cidx % gw + 0.5) * gres,
                static_cast<double>(maps_[ns].info.origin_y) +
                    (cidx / gw + 0.5) * gres}, false);
          }
        }
        // Commit: NO blacklist check — the extent-seek goal is a strategic
        // must-go. Blacklisting it (a false positive from thrash-induced
        // non-progress) was breaking the challenger streak and letting the +x
        // primary goal back in (run28 −8.35 ↔ +36.05 thrash). Nav2 CAN path to
        // it (verified ComputePathToPose OK 364 poses), so commit it.
        if (have && distance_robot_to_goal(ns, pick) >= min_assign_distance_) {
          util.clear();
          util.push_back({pick, 1e9});   // dominate everything; single stable goal
          used_fallback = true;
          if (now_ns - last_no_goal_debug_ns_ >
              static_cast<std::uint64_t>(debug_no_goal_log_interval_sec_ * 1e9)) {
            last_no_goal_debug_ns_ = now_ns;
            CFPA2_LOG_WARN(log_facade_,
                "[%s] EXTENT-SEEK dir=%+d reached_x=[%.1f,%.1f] -> goal "
                "(%.1f,%.1f) %s", ns.c_str(), seek_dir, rxmin, rxmax,
                pick.first, pick.second, pick_reach ? "reachable" : "snapped");
          }
        }
      }
    }

    // UNREACHABLE-region redirect (fallback + extent-completeness override):
    //  (a) FALLBACK — util empty: the robot exhausted its connected reachable
    //      component; every remaining frontier is across an unentered corridor
    //      that reads unreachable (Mid-360 blind zone fragments the trail).
    //  (b) OVERRIDE — util non-empty but the best reachable frontier is a small
    //      explored-interior pocket (best_reach_ig < override_min_ig) WHILE a
    //      much larger unexplored region exists across an unreachable gap (the
    //      −x corridor in the V-shaped ops2 building). Greedy nearest-first
    //      otherwise grinds +x pockets forever and never returns for −x (run24:
    //      path 54→176 m, x_max pinned 32.6, x_min stuck −2.9). The override
    //      makes CFPA2 information-gain-greedy: prefer the BIG unexplored region
    //      over scraps. Gated on cfpa2_explore_unreachable_override (ops2 only).
    //
    // In BOTH cases: find the single highest-IG UNREACHABLE frontier, snap it
    // to its nearest reachable cell, and commit to that one cell. IG-only +
    // single-goal = deterministic & stable (no velocity-dependent momentum flip,
    // no nearest-of-topK jump → no GOAL_SENT thrash). Its snapped cell LEADS the
    // robot toward the big region; once those cells get sensed they become
    // reachable → the normal path (momentum now HELPING the −x heading) resumes.
    const bool consider_redirect = !used_fallback && !targets.empty() &&
        (util.empty() ||
         (cfpa2_explore_unreachable_override_ &&
          best_reach_ig < cfpa2_override_min_ig_));
    if (consider_redirect) {
      // Single highest-IG UNREACHABLE frontier (deterministic winner).
      double best_un_ig = 0.0; Goal best_un{0.0, 0.0}; bool have_un = false;
      for (const auto & g : targets) {
        if (is_goal_peer_claimed(g) || is_blacklisted(ns, g, now_ns)) continue;
        if (goal_reachable(maps_[ns], dist_map, g)) continue;  // unreachable only
        const double ig = frontier_information_gain(maps_[ns], g);
        if (ig > best_un_ig) { best_un_ig = ig; best_un = g; have_un = true; }
      }
      // Commit if: util empty (any unreachable beats freezing), OR the big
      // region dominates the best reachable pocket by override_dominance.
      const bool dominates = util.empty() ||
          best_un_ig > std::max(cfpa2_override_min_ig_,
                                best_reach_ig * cfpa2_override_dominance_);
      if (have_un && best_un_ig >= 3.0 && dominates) {
        const int gw = static_cast<int>(maps_[ns].info.width);
        const double gres =
            std::max(1e-3, static_cast<double>(maps_[ns].info.resolution));
        const int snap_max_r_cells = static_cast<int>(60.0 / gres);
        const long snap_max_r2 =
            static_cast<long>(snap_max_r_cells) * static_cast<long>(snap_max_r_cells);
        const auto gg = world_to_grid(maps_[ns], best_un.first, best_un.second);
        if (gg.has_value()) {
          const int gi = gg->first, gj = gg->second;
          long best_d2 = snap_max_r2 + 1; int best_idx = -1;
          for (const auto & kv : dist_map) {
            const int ci = kv.first % gw, cj = kv.first / gw;
            const long d2 = static_cast<long>(ci - gi) * (ci - gi) +
                            static_cast<long>(cj - gj) * (cj - gj);
            if (d2 < best_d2) { best_d2 = d2; best_idx = kv.first; }
          }
          if (best_idx >= 0) {
            const Goal eff{
                static_cast<double>(maps_[ns].info.origin_x) +
                    (best_idx % gw + 0.5) * gres,
                static_cast<double>(maps_[ns].info.origin_y) +
                    (best_idx / gw + 0.5) * gres};
            const auto pc = grid_path_cost_m(maps_[ns], dist_map, eff);
            if (!is_blacklisted(ns, eff, now_ns) && pc.has_value() && *pc > 0.0 &&
                distance_robot_to_goal(ns, eff) >= min_assign_distance_) {
              util.clear();                       // override the small pockets
              util.push_back({eff, cfpa2_w_ig_ * best_un_ig});
              used_fallback = true;
            }
          }
        }
      }
    }
    if (util.empty() && !targets.empty() &&
        now_ns - last_no_goal_debug_ns_ >
          static_cast<std::uint64_t>(debug_no_goal_log_interval_sec_ * 1e9)) {
      last_no_goal_debug_ns_ = now_ns;
      CFPA2_LOG_WARN(log_facade_,
          "[%s] NO util from %zu targets: rej peer=%d blacklist=%d "
          "unreachable=%d clearance=%d low_utility=%d (dist_map=%zu cells)",
          ns.c_str(), targets.size(), rej_peer, rej_bl, rej_reach,
          rej_clear, rej_util, dist_map.size());
    }
    // STABLE sort by utility desc — matches Python's `sorted(..., key=-u)`.
    // Important: with unstable std::sort, two ticks with the same utility
    // list could produce different top-K orderings on near-ties, which
    // upstream causes the TSP head to flip → goal jitter → Nav2 unreachable
    // → fast-BL → endless oscillation. Stable sort fixes this.
    std::stable_sort(util.begin(), util.end(),
        [](const ScoredGoal & a, const ScoredGoal & b) { return a.utility > b.utility; });
    // In fallback mode keep ONLY the single best (max-IG) snapped goal so the
    // TSP head can't pick a nearer-but-jumpy alternate and the goal stays put
    // tick-to-tick (switch-hysteresis then holds it). See STABILITY note.
    if (used_fallback && util.size() > 1) {
      util.resize(1);
    }
    forced_goal_by_ns[ns] = used_fallback;
    utils_by_ns[ns] = std::move(util);
    dist_maps_by_ns[ns] = std::move(dist_map);
  }

  // ── Joint allocator (dual-robot only) ─────────────────────────────
  // Iterates every (goal_a, goal_b) pair from each robot's utility list
  // and picks the pair maximising:
  //   joint = (u_a + u_b) * clamp(1 − λ · overlap(a, b), 0, 1)
  // Overlap is a Gaussian on |a−b| / σ; λ=cfpa2_lambda_overlap acts as a
  // "max % deduction when goals fully overlap" (default 0.5 → up to 50%).
  // Multiplicative form (vs the legacy additive λ·overlap) keeps the
  // penalty scale-invariant w.r.t. IG-dominated scores in the hundreds.
  std::unordered_map<std::string, Goal> joint_candidates;
  std::unordered_map<std::string, double> joint_scores;
  if (namespaces_.size() == 2) {
    const std::string & ns_a = namespaces_[0];
    const std::string & ns_b = namespaces_[1];
    const auto & ua = utils_by_ns[ns_a];
    const auto & ub = utils_by_ns[ns_b];
    if (!ua.empty() && !ub.empty()) {
      double best_joint = -1e18;
      const ScoredGoal * best_a = nullptr;
      const ScoredGoal * best_b = nullptr;
      for (const auto & sa : ua) {
        for (const auto & sb : ub) {
          // Reject near-duplicates (same cell or within footprint).
          if (goals_equivalent(sa.goal, sb.goal)) continue;
          const double overlap = cfpa2_overlap_penalty(sa.goal, sb.goal);
          const double mult = std::max(0.0,
              std::min(1.0, 1.0 - cfpa2_lambda_overlap_ * overlap));
          const double joint = (sa.utility + sb.utility) * mult;
          if (joint > best_joint) {
            best_joint = joint;
            best_a = &sa;
            best_b = &sb;
          }
        }
      }
      if (best_a && best_b) {
        joint_candidates[ns_a] = best_a->goal;
        joint_candidates[ns_b] = best_b->goal;
        joint_scores[ns_a] = best_a->utility;
        joint_scores[ns_b] = best_b->utility;
        set_policy_reason(ns_a, "switch/cfpa2_joint");
        set_policy_reason(ns_b, "switch/cfpa2_joint");
      }
    }
  }

  // Pass 2: per-ns commit. Use joint pair if available, else single-robot
  // TSP top-K head / max utility.
  for (const auto & ns : namespaces_) {
    if (odoms_.find(ns) == odoms_.end()) continue;
    auto & util = utils_by_ns[ns];
    auto & dist_map = dist_maps_by_ns[ns];

    if (util.empty()) {
      // Even with no candidate utility list, stuck-recovery still wants to
      // observe the wedged-robot state so it can blacklist the held goal.
      maybe_force_cfpa2_stuck_recovery(ns, now_ns, util, targets, &maps_[ns]);
      continue;
    }

    Goal candidate;
    double cand_score;
    const auto joint_it = joint_candidates.find(ns);
    if (joint_it != joint_candidates.end()) {
      // Joint allocator picked for us; score lookup from joint_scores.
      candidate = joint_it->second;
      cand_score = joint_scores[ns];
    } else {
      // Single-robot path: TSP top-K head (default K=5).
      candidate = util.front().goal;
      cand_score = util.front().utility;
      if (cfpa2_tsp_k_ > 1) {
        const auto tsp_head = tsp_top_k_head(ns, util, cfpa2_tsp_k_);
        if (tsp_head.has_value()) {
          candidate = *tsp_head;
          for (const auto & sg : util) {
            if (goals_equivalent(sg.goal, candidate)) { cand_score = sg.utility; break; }
          }
        }
      }
    }

    // Stuck recovery may force a different candidate (or hold).
    const auto stuck_pick =
        maybe_force_cfpa2_stuck_recovery(ns, now_ns, util, targets, &maps_[ns]);
    if (stuck_pick.has_value()) {
      candidate = *stuck_pick;
    } else if (forced_switch_set.count(ns) > 0) {
      // Nav-stall blacklisted the held goal upstream — keep `candidate`.
    } else if (forced_goal_by_ns[ns]) {
      // Extent-seek / fallback produced a single deterministic strategic goal —
      // commit it DIRECTLY, bypassing apply_goal_policy. The policy's
      // stalled→blacklist + challenger-streak logic was thrashing the −x
      // redirect (blacklisting the −x goal on slow progress → +x primary back
      // in). The goal is a stable single candidate so there is no jitter risk,
      // and stuck-recovery above still handles physical wedging.
    } else {
      candidate = apply_goal_policy(
          ns, candidate, cand_score, maps_[ns], dist_map, now_ns, &targets);
    }

    set_active_goal(ns, candidate, now_ns);
    publish_goal(ns, maps_[ns], candidate);
    publish_goal_marker(ns,
        marker_frame_override_.empty() ? maps_[ns].info.frame_id : marker_frame_override_,
        candidate);
  }
}

// ───────────────────────────────────────────────────────────────────
//  Goal selection helpers (minimal stubs — full impl follows)
// ───────────────────────────────────────────────────────────────────

void CFPA2Coordinator::set_policy_reason(const std::string & ns, const std::string & reason)
{
  last_policy_reason_[ns] = reason;
}

Goal CFPA2Coordinator::apply_switch_hysteresis(
    const std::string & /*ns*/, Goal goal, double /*assignment_score*/)
{
  return goal;
}

Goal CFPA2Coordinator::apply_goal_policy(
    const std::string & ns, Goal candidate_goal, double assignment_score,
    const core::Grid & map_msg,
    const std::unordered_map<int, int> & dist_map,
    std::uint64_t now_ns,
    const std::vector<Goal> * current_targets)
{
  // 1) No previously held goal → commit immediately.
  const auto last_it = last_goal_.find(ns);
  if (last_it == last_goal_.end()) {
    set_policy_reason(ns, "switch/no_previous_goal");
    return candidate_goal;
  }
  const Goal last = last_it->second;

  // 2) Candidate is blacklisted → hold.
  if (is_blacklisted(ns, candidate_goal, now_ns)) {
    set_policy_reason(ns, "hold/candidate_blacklisted");
    return last;
  }

  // 3) Stranded-frontier: held goal far from every current frontier →
  //    underlying unknown cells were resolved; held goal is dead. Force
  //    switch + fast-blacklist the dead goal so we don't repick it.
  if (current_targets && !current_targets->empty()) {
    const double stale_r = cfpa2_stale_frontier_radius_m_;
    const double stale_r2 = stale_r * stale_r;
    bool still_frontier = false;
    for (const auto & t : *current_targets) {
      const double dx = t.first - last.first;
      const double dy = t.second - last.second;
      if (dx * dx + dy * dy <= stale_r2) {
        still_frontier = true;
        break;
      }
    }
    if (!still_frontier) {
      set_policy_reason(ns, "switch/stranded_frontier");
      const auto k = goal_key(last);
      const std::uint64_t bl_until =
          now_ns + static_cast<std::uint64_t>(std::max(30.0, blacklist_ttl_sec_) * 1e9);
      auto & bl = goal_blacklist_until_ns_[ns];
      auto kit = bl.find(k);
      bl[k] = kit == bl.end() ? bl_until : std::max(kit->second, bl_until);
      add_blacklist_disk(ns, last, bl_until, blacklist_cluster_radius_m_);
      return candidate_goal;
    }
  }

  const bool reached_last = goal_satisfied(ns, last, &map_msg);
  const auto safety = held_goal_safety_failure(map_msg, dist_map, last);
  const bool hard_failure = safety.has_value();

  const auto set_it = last_goal_set_time_ns_.find(ns);
  const std::uint64_t last_set_ns = set_it == last_goal_set_time_ns_.end() ? 0 : set_it->second;
  const bool lock_active =
      goal_lock_sec_ > 0.0 && last_set_ns > 0 &&
      (now_ns - last_set_ns) < static_cast<std::uint64_t>(goal_lock_sec_ * 1e9);

  // 4) Stable-challenger override (placed BEFORE goal_lock / hold so a
  //    consistent high-utility candidate can preempt mid-flight).
  const GoalKey cand_id = goal_key(candidate_goal);
  const GoalKey last_id = goal_key(last);
  if (cand_id == last_id) {
    challenger_id_.erase(ns);
    challenger_streak_.erase(ns);
  } else {
    auto cid_it = challenger_id_.find(ns);
    if (cid_it != challenger_id_.end() && cid_it->second == cand_id) {
      challenger_streak_[ns] = challenger_streak_[ns] + 1;
    } else {
      challenger_id_[ns] = cand_id;
      challenger_streak_[ns] = 1;
    }
    const int streak = challenger_streak_[ns];
    const double lock_age_sec = last_set_ns > 0
        ? std::max(0.0, static_cast<double>(now_ns - last_set_ns) / 1e9)
        : 1e9;
    if (cfpa2_challenger_streak_required_ > 0 &&
        streak >= cfpa2_challenger_streak_required_ &&
        lock_age_sec >= cfpa2_challenger_min_lock_age_sec_)
    {
      const double last_score = cfpa2_single_utility(ns, last, map_msg, dist_map);
      if (last_score <= -1e17) {
        set_policy_reason(ns, "switch/held_goal_dead");
        challenger_id_.erase(ns);
        challenger_streak_.erase(ns);
        return candidate_goal;
      }
      if (assignment_score > last_score * cfpa2_challenger_improvement_factor_) {
        std::ostringstream rr;
        rr << "switch/stable_challenger_u=" << assignment_score
           << "_vs_held=" << last_score;
        set_policy_reason(ns, rr.str());
        challenger_id_.erase(ns);
        challenger_streak_.erase(ns);
        return candidate_goal;
      }
    }
  }

  // 5) Goal-lock window: commit unless reached or hard failure.
  if (lock_active && !hard_failure && !reached_last) {
    set_policy_reason(ns, "hold/goal_lock_active");
    return last;
  }

  const auto delta = progress_delta(ns);
  const bool stalled =
      delta.has_value() && *delta < progress_min_delta_m_;

  // 6) Reject candidates within `switch_min_dist` of held goal — too close
  //    to bother switching.
  const double move_dx = candidate_goal.first - last.first;
  const double move_dy = candidate_goal.second - last.second;
  const double candidate_move = std::sqrt(move_dx * move_dx + move_dy * move_dy);
  if (candidate_move < switch_min_dist_) {
    set_policy_reason(ns, "hold/small_candidate_move");
    challenger_id_.erase(ns);
    challenger_streak_.erase(ns);
    return last;
  }

  // 7) Commitment-based hold: still progressing → keep last goal.
  if (!reached_last && !hard_failure) {
    if (!stalled) {
      set_policy_reason(ns, "hold/progressing");
      return last;
    }
    // Stalled but candidate's utility advantage is below hysteresis → still hold.
    if (assignment_score < switch_hysteresis_) {
      set_policy_reason(ns, "hold/stalled_but_low_score");
      return last;
    }
  }

  // 8) Either hard failure or stalled-with-decent-candidate → switch +
  //    record failure for fail-count blacklist.
  if (!reached_last && (hard_failure || stalled)) {
    const std::string reason = hard_failure ? *safety : "stalled";
    register_goal_failure(ns, last, now_ns, reason);
    set_policy_reason(ns, "switch/" + reason);
    return candidate_goal;
  }

  set_policy_reason(ns, "switch/reached_or_improved");
  return candidate_goal;
}

std::optional<Goal> CFPA2Coordinator::tsp_top_k_head(
    const std::string & ns, const UtilityList & utilities, int k)
{
  // Faithful port of Python `_tsp_top_k_head`: top-K by utility, then
  // nearest-neighbor TSP tour starting from the robot. Return the head
  // of that tour (= the first candidate visited, i.e. the one nearest
  // to the robot). The full tour is computed even though only the head
  // is used, matching the comment "answers 'which frontier should I
  // visit FIRST if I were going to sweep all top-K?'".
  //
  // Two ticks with the same top-K + same robot pose must produce the
  // same head — that's the anti-jitter property. Achieved by:
  //  (a) caller passing a STABLE-sorted utility list (std::stable_sort),
  //  (b) NN inner loop iterating in deterministic index order so ties
  //      break consistently.
  if (utilities.empty()) return std::nullopt;
  if (k <= 1) return utilities.front().goal;
  const std::size_t kk = std::min<std::size_t>(k, utilities.size());
  if (kk == 1) return utilities.front().goal;

  const auto r = robot_xy(ns);

  // Build the candidate index pool (0..kk-1) and the working
  // "remaining" bitmask. cur_xy starts at the robot's pose, walks
  // along the tour. first_idx is captured on the first iteration.
  std::vector<bool> remaining(kk, true);
  int first_idx = -1;
  double cur_x = r.first;
  double cur_y = r.second;
  for (std::size_t step = 0; step < kk; ++step) {
    int best_i = -1;
    double best_d2 = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < kk; ++i) {
      if (!remaining[i]) continue;
      const double dx = utilities[i].goal.first - cur_x;
      const double dy = utilities[i].goal.second - cur_y;
      const double d2 = dx * dx + dy * dy;
      if (d2 < best_d2) {
        best_d2 = d2;
        best_i = static_cast<int>(i);
      }
    }
    if (best_i < 0) break;
    if (first_idx < 0) first_idx = best_i;
    cur_x = utilities[best_i].goal.first;
    cur_y = utilities[best_i].goal.second;
    remaining[best_i] = false;
  }
  if (first_idx < 0) return utilities.front().goal;
  return utilities[first_idx].goal;
}

std::optional<Goal> CFPA2Coordinator::cfpa2_best_available_goal(
    const std::string & /*ns*/, std::uint64_t /*now_ns*/,
    const UtilityList & utilities,
    const std::optional<Goal> & /*exclude_goal*/,
    const std::vector<Goal> * /*fallback_targets*/,
    const core::Grid * /*map_msg*/,
    const std::unordered_map<int, int> * /*dist_map*/)
{
  if (utilities.empty()) return std::nullopt;
  return utilities.front().goal;
}

std::optional<Goal> CFPA2Coordinator::maybe_force_cfpa2_stuck_recovery(
    const std::string & ns, std::uint64_t now_ns,
    const UtilityList & utilities,
    const std::vector<Goal> & /*fallback_targets*/,
    const core::Grid * /*map_msg*/)
{
  // Only fire if a goal is currently held and stuck_lock_sec elapsed
  // since it was set.
  const auto held_it = last_goal_.find(ns);
  if (held_it == last_goal_.end()) return std::nullopt;
  const auto set_it = last_goal_set_time_ns_.find(ns);
  if (set_it == last_goal_set_time_ns_.end()) return std::nullopt;
  const std::uint64_t lock_thr_ns =
      static_cast<std::uint64_t>(cfpa2_stuck_lock_sec_ * 1e9);
  if (now_ns - set_it->second < lock_thr_ns) return std::nullopt;

  // Cooldown after the last stuck event — don't fire again immediately.
  const auto cd_it = cfpa2_last_stuck_event_ns_.find(ns);
  if (cd_it != cfpa2_last_stuck_event_ns_.end()) {
    const std::uint64_t cd_ns =
        static_cast<std::uint64_t>(cfpa2_stuck_blacklist_sec_ * 1e9);
    if (now_ns - cd_it->second < cd_ns) return std::nullopt;
  }

  const std::uint64_t window_ns =
      static_cast<std::uint64_t>(cfpa2_stuck_window_sec_ * 1e9);
  const auto disp = max_displacement_in_window(ns, window_ns);
  if (!disp.has_value()) return std::nullopt;
  if (*disp >= cfpa2_stuck_min_motion_m_) return std::nullopt;

  CFPA2_LOG_WARN(log_facade_,
      "[%s] stuck-recovery: held goal (%.2f, %.2f) for >%.0fs, "
      "displacement=%.3fm < %.3fm. Blacklisting + switching.",
      ns.c_str(), held_it->second.first, held_it->second.second,
      cfpa2_stuck_lock_sec_, *disp, cfpa2_stuck_min_motion_m_);

  blacklist_active_goal(ns, now_ns, "stuck_recovery");
  cfpa2_last_stuck_event_ns_[ns] = now_ns;

  // Return the next-best non-blacklisted utility candidate, if any.
  for (const auto & cand : utilities) {
    if (!is_blacklisted(ns, cand.goal, now_ns)) return cand.goal;
  }
  return std::nullopt;
}

std::optional<Goal> CFPA2Coordinator::apply_cfpa2_proximity_stop(
    const std::string & /*ns*/, Goal /*goal*/,
    const core::Grid & /*map_msg*/)
{
  return std::nullopt;
}

std::optional<double> CFPA2Coordinator::progress_delta(const std::string & /*ns*/) const
{
  return std::nullopt;
}

bool CFPA2Coordinator::pivot_clearance_blocked(const std::string & /*ns*/) const
{
  return false;
}

std::optional<double> CFPA2Coordinator::max_displacement_in_window(
    const std::string & ns, std::uint64_t window_ns) const
{
  const auto it = goal_lock_pose_history_.find(ns);
  if (it == goal_lock_pose_history_.end() || it->second.empty()) return std::nullopt;
  const auto & h = it->second;
  const std::uint64_t now_ns = h.back().ns;
  const std::uint64_t cutoff_ns = now_ns > window_ns ? now_ns - window_ns : 0;
  const auto p_now = std::make_pair(h.back().x, h.back().y);
  double best_d2 = 0.0;
  for (const auto & s : h) {
    if (s.ns < cutoff_ns) continue;
    const double dx = s.x - p_now.first;
    const double dy = s.y - p_now.second;
    const double d2 = dx * dx + dy * dy;
    if (d2 > best_d2) best_d2 = d2;
  }
  return std::sqrt(best_d2);
}

Goal CFPA2Coordinator::set_active_goal(
    const std::string & ns, Goal goal, std::uint64_t now_ns)
{
  // Only reset the lock-age clock when the goal CHANGES — matches
  // Python `_set_active_goal`:
  //   if prev is None or math.hypot(prev - goal) > 1e-6:
  //       self.last_goal_set_time_ns[ns] = now_ns
  // Without this, last_goal_set_time_ns_ updates every tick and
  // (now - last_set_ns) never exceeds the tick period — so goal_lock
  // appears infinite, challenger_min_lock_age (2 s) never fires, and
  // none of the time-based gates in apply_goal_policy can ever expire.
  const auto prev_it = last_goal_.find(ns);
  const bool goal_changed = (prev_it == last_goal_.end()) ||
      std::hypot(prev_it->second.first - goal.first,
                 prev_it->second.second - goal.second) > 1e-6;
  last_goal_[ns] = goal;
  if (goal_changed) {
    last_goal_set_time_ns_[ns] = now_ns;
    goal_lock_start_xy_[ns] = robot_xy(ns);
    goal_progress_samples_[ns].clear();
    goal_lock_pose_history_[ns].clear();
    last_unreachable_goal_seq_.erase(ns);
    unreachable_consec_.erase(ns);
  }
  return goal;
}

// ───────────────────────────────────────────────────────────────────
//  Space-time A* (stubs — TODO: port full algorithm)
// ───────────────────────────────────────────────────────────────────

std::optional<std::vector<std::pair<int, int>>>
CFPA2Coordinator::space_time_astar_cells(
    const std::string & /*ns*/,
    const core::Grid & /*map_msg*/,
    Goal /*final_goal*/,
    const std::unordered_map<std::string, Goal> & /*planned_goals*/)
{
  return std::nullopt;
}

std::vector<std::set<std::pair<int, int>>>
CFPA2Coordinator::predict_other_robot_blocks(
    const core::Grid & /*map_msg*/,
    const std::string & /*ns*/,
    const std::unordered_map<std::string, Goal> & /*planned_goals*/,
    int steps, double /*dt_sec*/, int /*safety_radius_cells*/)
{
  return std::vector<std::set<std::pair<int, int>>>(std::max(1, steps + 1));
}

std::optional<std::pair<int, int>> CFPA2Coordinator::find_nearest_free_cell(
    const core::Grid & msg,
    std::pair<int, int> start, int search_radius)
{
  const int w = static_cast<int>(msg.info.width);
  const int h = static_cast<int>(msg.info.height);
  for (int r = 0; r <= search_radius; ++r) {
    for (int dy = -r; dy <= r; ++dy) {
      const int ny = start.second + dy;
      if (ny < 0 || ny >= h) continue;
      for (int dx = -r; dx <= r; ++dx) {
        const int nx = start.first + dx;
        if (nx < 0 || nx >= w) continue;
        if (is_free_idx(msg.data, ny * w + nx)) {
          return std::make_pair(nx, ny);
        }
      }
    }
  }
  return std::nullopt;
}

std::optional<Goal> CFPA2Coordinator::cfpa2_space_time_waypoint(
    const std::string & /*ns*/,
    const std::vector<std::pair<int, int>> & /*path*/,
    const core::Grid & /*msg*/)
{
  return std::nullopt;
}

// ───────────────────────────────────────────────────────────────────
//  Publishers
// ───────────────────────────────────────────────────────────────────

void CFPA2Coordinator::publish_goal(
    const std::string & ns, const core::Grid & map_msg, Goal goal_w)
{
  if (!goal_is_finite(goal_w) || !goal_pub_facade_) return;
  const std::string frame_id = marker_frame_override_.empty()
      ? map_msg.info.frame_id : marker_frame_override_;
  goal_pub_facade_->publish_goal(ns, goal_w, frame_id);
}

void CFPA2Coordinator::publish_goal_marker(
    const std::string & ns, const std::string & frame_id, Goal goal_w)
{
  if (!goal_is_finite(goal_w) || !goal_pub_facade_) return;
  const auto rgb = ns_color(ns);
  goal_pub_facade_->publish_goal_marker(ns, goal_w, frame_id, rgb);
}

bool CFPA2Coordinator::goal_is_finite(Goal goal_w)
{
  return std::isfinite(goal_w.first) && std::isfinite(goal_w.second);
}

std::array<float, 3> CFPA2Coordinator::ns_color(const std::string & ns) const
{
  // Deterministic hash → RGB.
  std::size_t h = std::hash<std::string>{}(ns);
  const float r = static_cast<float>((h & 0xFF) / 255.0);
  const float g = static_cast<float>(((h >> 8) & 0xFF) / 255.0);
  const float b = static_cast<float>(((h >> 16) & 0xFF) / 255.0);
  return {r, g, b};
}

void CFPA2Coordinator::publish_coordinator_map(
    const core::Grid & target_map)
{
  if (viz_facade_) viz_facade_->publish_coordinator_map(target_map);
}

void CFPA2Coordinator::publish_robot_markers(
    const core::Grid & target_map)
{
  if (!viz_facade_) return;
  const std::string frame_id = marker_frame_override_.empty()
      ? target_map.info.frame_id : marker_frame_override_;
  std::vector<core::RobotPoseView> poses;
  std::vector<core::TrajectoryView> trajs;
  for (const auto & ns : namespaces_) {
    const auto oit = odoms_.find(ns);
    if (oit == odoms_.end()) continue;
    const auto rgb = ns_color(ns);
    core::RobotPoseView pv;
    pv.frame_id = frame_id;
    pv.x = oit->second.x;
    pv.y = oit->second.y;
    pv.z = 0.15;
    pv.yaw = oit->second.yaw;
    pv.color = rgb;
    pv.scale = robot_marker_scale_;
    poses.push_back(std::move(pv));

    core::TrajectoryView tv;
    tv.frame_id = frame_id;
    tv.color = rgb;
    const auto tit = trajectory_history_.find(ns);
    if (tit != trajectory_history_.end()) {
      tv.points_xy.assign(tit->second.begin(), tit->second.end());
    }
    if (!tv.points_xy.empty()) trajs.push_back(std::move(tv));
  }
  viz_facade_->publish_robot_markers(poses, trajs);
}

void CFPA2Coordinator::publish_frontier_markers(
    const core::Grid & target_map,
    const std::vector<Goal> & targets)
{
  if (!viz_facade_) return;
  const std::string frame_id = marker_frame_override_.empty()
      ? target_map.info.frame_id : marker_frame_override_;
  viz_facade_->publish_frontier_markers(frame_id, targets);
}

// ───────────────────────────────────────────────────────────────────
//  Logging / perf
// ───────────────────────────────────────────────────────────────────

void CFPA2Coordinator::maybe_log_summary(
    std::uint64_t /*now_ns*/,
    const std::unordered_map<std::string, std::size_t> & /*per_ns_frontiers*/,
    const std::unordered_map<std::string, std::size_t> & /*per_ns_reachable*/,
    const std::unordered_map<std::string, std::optional<Goal>> & /*per_ns_assigned*/,
    const std::unordered_map<std::string, double> & /*per_ns_dist*/,
    const std::unordered_map<std::string, double> & /*per_ns_util*/,
    std::size_t /*targets_total*/)
{
  // TODO: periodic ASSIGN log
}

std::tuple<int, int, int> CFPA2Coordinator::map_cell_stats(
    const core::Grid & msg) const
{
  int free_n = 0;
  int occ_n = 0;
  int unknown_n = 0;
  for (const auto v : msg.data) {
    if (v == unknown_value_) ++unknown_n;
    else if (v >= occ_thresh_) ++occ_n;
    else if (v == free_value_) ++free_n;
    else ++unknown_n;
  }
  return {free_n, occ_n, unknown_n};
}

bool CFPA2Coordinator::should_log_no_goal_debug(std::uint64_t now_ns)
{
  if (!debug_no_goal_logging_) return false;
  const std::uint64_t window_ns =
      static_cast<std::uint64_t>(debug_no_goal_log_interval_sec_ * 1e9);
  if (now_ns - last_no_goal_debug_ns_ < window_ns) return false;
  last_no_goal_debug_ns_ = now_ns;
  return true;
}

void CFPA2Coordinator::log_no_goal_debug(
    std::uint64_t now_ns, const std::string & reason,
    const core::Grid & planning_map,
    const std::unordered_map<std::string, std::vector<Goal>> & per_ns_targets)
{
  if (!should_log_no_goal_debug(now_ns)) return;
  const auto [free_n, occ_n, unk_n] = map_cell_stats(planning_map);
  std::ostringstream ss;
  ss << "NO_GOAL[" << reason << "] planning_map(free=" << free_n
     << " occ=" << occ_n << " unk=" << unk_n << ")";
  for (const auto & [ns, targets] : per_ns_targets) {
    ss << "\n  [" << ns << "] targets=" << targets.size();
  }
  CFPA2_LOG_WARN(log_facade_,"%s", ss.str().c_str());
}

double CFPA2Coordinator::percentile(
    const std::vector<double> & sorted_values, double quantile)
{
  if (sorted_values.empty()) return 0.0;
  const double q = std::max(0.0, std::min(1.0, quantile));
  const double idx_f = q * (sorted_values.size() - 1);
  const auto idx = static_cast<std::size_t>(idx_f);
  const double frac = idx_f - static_cast<double>(idx);
  if (idx + 1 < sorted_values.size()) {
    return sorted_values[idx] * (1.0 - frac) + sorted_values[idx + 1] * frac;
  }
  return sorted_values.back();
}

void CFPA2Coordinator::record_tick_perf(std::uint64_t tick_start_ns)
{
  if (!perf_enable_) return;
  const auto t1 = clock_facade_->now_ns();
  const double dt_ms = static_cast<double>(t1 - tick_start_ns) / 1e6;
  perf_tick_durations_ms_.push_back(dt_ms);
  while (static_cast<int>(perf_tick_durations_ms_.size()) > perf_tick_window_size_) {
    perf_tick_durations_ms_.pop_front();
  }
}

void CFPA2Coordinator::maybe_log_perf_summary()
{
  const auto now_ns = clock_facade_->now_ns();
  if (now_ns - last_perf_summary_ns_ < static_cast<std::uint64_t>(10e9)) return;
  if (static_cast<int>(perf_tick_durations_ms_.size()) < perf_min_samples_) return;
  std::vector<double> sorted(perf_tick_durations_ms_.begin(), perf_tick_durations_ms_.end());
  std::sort(sorted.begin(), sorted.end());
  const double p50 = percentile(sorted, 0.50);
  const double p95 = percentile(sorted, 0.95);
  const double pmax = sorted.back();
  last_perf_summary_ns_ = now_ns;
  CFPA2_LOG_INFO(log_facade_,"[perf] tick p50=%.1fms p95=%.1fms max=%.1fms n=%zu",
      p50, p95, pmax, sorted.size());
  if (p95 > perf_tick_warn_p95_ms_) {
    CFPA2_LOG_WARN(log_facade_,"[perf] tick p95 (%.1fms) > warn (%.1fms)",
        p95, perf_tick_warn_p95_ms_);
  }
}

void CFPA2Coordinator::update_adaptive_load_shedding(std::uint64_t /*now_ns*/)
{
  // TODO: adjust adaptive_frontier_stride_, max_targets_, gain_radius_,
  // skip_ticks_ based on perf samples.
}

}  // namespace cfpa2
