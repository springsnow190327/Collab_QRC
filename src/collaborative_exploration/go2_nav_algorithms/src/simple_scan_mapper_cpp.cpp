#include <chrono>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"

namespace
{
double yaw_from_quat(double x, double y, double z, double w)
{
  const double siny = 2.0 * (w * z + x * y);
  const double cosy = 1.0 - 2.0 * (y * y + z * z);
  return std::atan2(siny, cosy);
}
}  // namespace

class SimpleScanMapperCpp : public rclcpp::Node
{
public:
  SimpleScanMapperCpp()
  : Node("simple_scan_mapper")
  {
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom/nav");
    map_topic_ = declare_parameter<std::string>("map_topic", "/map");
    map_frame_ = declare_parameter<std::string>("map_frame", "world");
    scan_frame_ = declare_parameter<std::string>("scan_frame", "");
    lidar_offset_x_ = declare_parameter<double>("lidar_offset_x", 0.0);
    lidar_offset_y_ = declare_parameter<double>("lidar_offset_y", 0.0);

    resolution_ = declare_parameter<double>("resolution", 0.10);
    width_ = declare_parameter<int>("width", 400);
    height_ = declare_parameter<int>("height", 400);
    origin_x_ = declare_parameter<double>("origin_x", -20.0);
    origin_y_ = declare_parameter<double>("origin_y", -20.0);

    max_range_ = std::max(0.1, declare_parameter<double>("max_range", 12.0));
    max_clear_distance_ = std::max(0.0, declare_parameter<double>("max_clear_distance", 4.0));
    clear_on_nohit_ = declare_parameter<bool>("clear_on_nohit", false);
    update_rate_ = std::max(0.5, declare_parameter<double>("update_rate", 4.0));
    startup_delay_ = std::max(0.0, declare_parameter<double>("startup_delay", 4.0));
    tf_timeout_ms_ = std::max(1, static_cast<int>(declare_parameter("tf_timeout_ms", 1000)));
    max_scan_odom_dt_ = std::max(0.0, declare_parameter<double>("max_scan_odom_dt", 0.10));

    hit_increment_ = std::max(1, static_cast<int>(declare_parameter("hit_increment", 3)));
    miss_decrement_ = std::max(1, static_cast<int>(declare_parameter("miss_decrement", 1)));
    score_min_ = static_cast<int>(declare_parameter("score_min", -20));
    score_max_ = static_cast<int>(declare_parameter("score_max", 20));
    if (score_min_ >= score_max_) {
      score_min_ = -20;
      score_max_ = 20;
    }
    occupied_score_threshold_ = static_cast<int>(declare_parameter("occupied_score_threshold", 3));
    free_score_threshold_ = static_cast<int>(declare_parameter("free_score_threshold", -3));
    if (free_score_threshold_ >= occupied_score_threshold_) {
      free_score_threshold_ = -3;
      occupied_score_threshold_ = 3;
    }

    // Score decay: every decay_interval_sec, all positive scores are
    // decremented by decay_amount.  Real walls are constantly re-hit by
    // LiDAR and stay occupied; a moved door's ghost cells fade away.
    decay_interval_sec_ = std::max(
      0.0, declare_parameter<double>("decay_interval_sec", 2.0));
    decay_amount_ = std::max(
      0, static_cast<int>(declare_parameter("decay_amount", 1)));

    // Exemption zone: rectangular area where occupied cells are forced free.
    // Used for the door corridor so RRT* can plan through the opening.
    // Disabled when all four values are 0.0 (default).
    exempt_x_min_ = declare_parameter<double>("exempt_x_min", 0.0);
    exempt_x_max_ = declare_parameter<double>("exempt_x_max", 0.0);
    exempt_y_min_ = declare_parameter<double>("exempt_y_min", 0.0);
    exempt_y_max_ = declare_parameter<double>("exempt_y_max", 0.0);
    has_exemption_zone_ = (exempt_x_max_ > exempt_x_min_ &&
                           exempt_y_max_ > exempt_y_min_);
    if (has_exemption_zone_) {
      // Pre-compute grid bounds for the exemption zone
      exempt_gx_min_ = std::max(0, static_cast<int>((exempt_x_min_ - origin_x_) / resolution_));
      exempt_gx_max_ = std::min(width_ - 1, static_cast<int>((exempt_x_max_ - origin_x_) / resolution_));
      exempt_gy_min_ = std::max(0, static_cast<int>((exempt_y_min_ - origin_y_) / resolution_));
      exempt_gy_max_ = std::min(height_ - 1, static_cast<int>((exempt_y_max_ - origin_y_) / resolution_));
    }

    // Motion compensation parameters
    pose_filter_alpha_ = std::clamp(
      declare_parameter<double>("pose_filter_alpha", 0.7), 0.1, 1.0);
    max_angular_velocity_ = std::max(
      0.0, declare_parameter<double>("max_angular_velocity", 0.4));

    const int64_t n_cells = static_cast<int64_t>(width_) * static_cast<int64_t>(height_);
    grid_.assign(static_cast<size_t>(n_cells), -1);
    scores_.assign(static_cast<size_t>(n_cells), 0);
    observed_.assign(static_cast<size_t>(n_cells), false);

    // broadcast_tf: When true (default), the mapper broadcasts map -> base_link
    // from odom.  Works for Gazebo where slam_odom_relay provides odom.
    // Set false for real-robot Cartographer, which already provides the full
    // TF chain (map -> odom -> body -> base_link) and would conflict.
    broadcast_tf_ = declare_parameter<bool>("broadcast_tf", true);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    if (broadcast_tf_) {
      tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    }

    const auto scan_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    const auto odom_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, scan_qos,
      [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        last_scan_ = msg;
      });

    // Odom sub: omega gating, scan-odom dt, and optional TF broadcast
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, odom_qos,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        last_odom_ = msg;
        odom_omega_ = msg->twist.twist.angular.z;
        if (broadcast_tf_) {
          broadcast_odom_tf(msg);
        }
      });

    // TRANSIENT_LOCAL durability: standard for map topics.
    // Matches default_nav's subscriber QoS — without this, the nav
    // never receives the map and falls back to local scan-based planning.
    auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    map_qos.transient_local();
    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(map_topic_, map_qos);
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / update_rate_),
      std::bind(&SimpleScanMapperCpp::update, this));

    // Decay timer: fade stale occupied cells (dynamic obstacles like doors)
    if (decay_interval_sec_ > 0.0 && decay_amount_ > 0) {
      decay_timer_ = create_wall_timer(
        std::chrono::duration<double>(decay_interval_sec_),
        std::bind(&SimpleScanMapperCpp::decay_scores, this));
    }

    RCLCPP_INFO(
      get_logger(),
      "Simple scan mapper (C++, TF-based) started | scan=%s odom=%s map=%s size=%dx%d res=%.2f "
      "lidar_offset=(%.3f,%.3f) score=[%d,%d] hit=%d miss=%d tf_timeout=%dms "
      "pose_filter_alpha=%.2f max_angular_velocity=%.2f",
      scan_topic_.c_str(), odom_topic_.c_str(), map_topic_.c_str(), width_, height_, resolution_,
      lidar_offset_x_, lidar_offset_y_, score_min_, score_max_, hit_increment_, miss_decrement_,
      tf_timeout_ms_, pose_filter_alpha_, max_angular_velocity_);
  }

private:
  using OdomMsg = nav_msgs::msg::Odometry;
  using ScanMsg = sensor_msgs::msg::LaserScan;

  std::optional<std::pair<int, int>> world_to_grid(double x, double y) const
  {
    const int gx = static_cast<int>((x - origin_x_) / resolution_);
    const int gy = static_cast<int>((y - origin_y_) / resolution_);
    if (gx < 0 || gy < 0 || gx >= width_ || gy >= height_) {
      return std::nullopt;
    }
    return std::make_pair(gx, gy);
  }

  inline size_t idx(int gx, int gy) const
  {
    return static_cast<size_t>(gy * width_ + gx);
  }

  void apply_evidence(int gx, int gy, int delta)
  {
    const size_t i = idx(gx, gy);
    observed_[i] = true;
    int score = scores_[i] + delta;
    score = std::clamp(score, score_min_, score_max_);
    scores_[i] = score;
  }

  // Raytrace from (x0,y0) to (x1,y1), marking cells as free.
  // STOPS when hitting a cell already above occupied threshold — this
  // prevents free rays from erasing confirmed walls when viewing from
  // the other side (the closure/wall-erasure problem).
  void raytrace_free(int x0, int y0, int x1, int y1)
  {
    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);
    int x = x0;
    int y = y0;
    const int sx = (x0 < x1) ? 1 : -1;
    const int sy = (y0 < y1) ? 1 : -1;

    if (dx > dy) {
      double err = static_cast<double>(dx) / 2.0;
      while (x != x1) {
        const size_t i = idx(x, y);
        // Stop ray at long-confirmed walls (score_max) — but allow
        // clearing recently-occupied cells (e.g. door that just moved).
        if (scores_[i] >= score_max_) return;
        observed_[i] = true;
        scores_[i] = std::clamp(scores_[i] - miss_decrement_, score_min_, score_max_);
        err -= static_cast<double>(dy);
        if (err < 0.0) {
          y += sy;
          err += static_cast<double>(dx);
        }
        x += sx;
      }
      return;
    }

    double err = static_cast<double>(dy) / 2.0;
    while (y != y1) {
      const size_t i = idx(x, y);
      // Stop ray at confirmed walls — don't erase through them
      if (scores_[i] >= occupied_score_threshold_) return;
      observed_[i] = true;
      scores_[i] = std::clamp(scores_[i] - miss_decrement_, score_min_, score_max_);
      err -= static_cast<double>(dx);
      if (err < 0.0) {
        x += sx;
        err += static_cast<double>(dy);
      }
      y += sy;
    }
  }

  // Broadcast odom as TF (only when broadcast_tf_ == true)
  void broadcast_odom_tf(const OdomMsg::SharedPtr & msg)
  {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = msg->header.stamp;
    t.header.frame_id = map_frame_;
    t.child_frame_id = "base_link";
    t.transform.translation.x = msg->pose.pose.position.x;
    t.transform.translation.y = msg->pose.pose.position.y;
    t.transform.translation.z = msg->pose.pose.position.z;
    t.transform.rotation = msg->pose.pose.orientation;
    tf_broadcaster_->sendTransform(t);
  }


  // Periodic score decay: decrement all positive scores so stale
  // occupied cells (e.g. door that moved) fade to free over time.
  // Cells actively being hit by LiDAR get re-incremented each scan
  // and remain occupied.
  void decay_scores()
  {
    if (!start_time_) return;
    const size_t n = scores_.size();
    int decayed = 0;
    for (size_t i = 0; i < n; ++i) {
      if (scores_[i] > 0) {
        scores_[i] = std::max(0, scores_[i] - decay_amount_);
        ++decayed;
      }
    }
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000,
      "DECAY: decremented %d positive cells by %d", decayed, decay_amount_);
  }

  void publish_map(const builtin_interfaces::msg::Time & stamp)
  {
    const size_t n = scores_.size();
    for (size_t i = 0; i < n; ++i) {
      if (!observed_[i]) {
        grid_[i] = -1;
      } else if (scores_[i] >= occupied_score_threshold_) {
        grid_[i] = 100;
      } else if (scores_[i] <= free_score_threshold_) {
        grid_[i] = 0;
      } else {
        grid_[i] = -1;
      }
    }

    // Exemption zone: force cells in the door corridor to free (0).
    // RRT* can then plan straight through the opening.
    if (has_exemption_zone_) {
      for (int gy = exempt_gy_min_; gy <= exempt_gy_max_; ++gy) {
        for (int gx = exempt_gx_min_; gx <= exempt_gx_max_; ++gx) {
          const size_t i = idx(gx, gy);
          grid_[i] = 0;           // force free in published map
          scores_[i] = std::min(scores_[i], 0);  // prevent score buildup
        }
      }
    }

    nav_msgs::msg::OccupancyGrid msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = map_frame_;
    msg.info.resolution = static_cast<float>(resolution_);
    msg.info.width = static_cast<uint32_t>(width_);
    msg.info.height = static_cast<uint32_t>(height_);
    msg.info.origin.position.x = origin_x_;
    msg.info.origin.position.y = origin_y_;
    msg.info.origin.orientation.w = 1.0;
    msg.data = grid_;
    map_pub_->publish(msg);
  }

  // Normalize angle to [-pi, pi]
  static double normalize_angle(double a)
  {
    while (a > M_PI)  a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
  }

  void update()
  {
    if (!last_scan_ || !last_odom_) {
      return;
    }

    const auto now_t = now();
    if (!start_time_) {
      start_time_ = now_t;
    }
    if ((now_t - *start_time_).seconds() < startup_delay_) {
      return;
    }

    const auto scan = last_scan_;
    const auto odom = last_odom_;

    // Gate: reject scan-odom pairs with too much timestamp difference.
    {
      const double scan_t = rclcpp::Time(scan->header.stamp).seconds();
      const double odom_t = rclcpp::Time(odom->header.stamp).seconds();
      const double dt = std::abs(scan_t - odom_t);
      if (dt > max_scan_odom_dt_) {
        if ((now_t.nanoseconds() - last_tf_warn_ns_) > static_cast<int64_t>(3e9)) {
          RCLCPP_WARN(get_logger(),
            "Dropping scan: scan-odom dt=%.3fs exceeds max_scan_odom_dt=%.3fs",
            dt, max_scan_odom_dt_);
          last_tf_warn_ns_ = now_t.nanoseconds();
        }
        return;
      }
    }

    // Gate: skip scans during fast rotation.
    // At range R, angular velocity omega with desync dt creates displacement
    // R * omega * dt.  At R=5m, omega=0.5 rad/s, dt=40ms => 10cm = 1 cell.
    if (max_angular_velocity_ > 0.0 && std::abs(odom_omega_) > max_angular_velocity_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
        "Dropping scan: angular velocity %.2f rad/s exceeds %.2f rad/s threshold",
        odom_omega_, max_angular_velocity_);
      last_scan_.reset();
      return;
    }

    // Determine scan frame: use parameter override, else scan header
    const std::string lookup_frame = scan_frame_.empty()
      ? scan->header.frame_id
      : scan_frame_;

    // TF-based pose lookup: interpolates odom at the exact scan timestamp.
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_->lookupTransform(
        map_frame_, lookup_frame,
        scan->header.stamp,
        std::chrono::milliseconds(tf_timeout_ms_));
    } catch (const tf2::TransformException & ex) {
      if ((now_t.nanoseconds() - last_tf_warn_ns_) > static_cast<int64_t>(3e9)) {
        RCLCPP_WARN(get_logger(), "TF lookup failed (%s -> %s): %s",
          map_frame_.c_str(), lookup_frame.c_str(), ex.what());
        last_tf_warn_ns_ = now_t.nanoseconds();
      }
      return;
    }

    const double rx = transform.transform.translation.x;
    const double ry = transform.transform.translation.y;
    const double yaw = yaw_from_quat(
      transform.transform.rotation.x,
      transform.transform.rotation.y,
      transform.transform.rotation.z,
      transform.transform.rotation.w);

    // Diagnostic: log every scan update to detect yaw jumps / timestamp issues
    {
      const double scan_t = rclcpp::Time(scan->header.stamp).seconds();
      const double odom_t = rclcpp::Time(odom->header.stamp).seconds();
      const double tf_t = rclcpp::Time(transform.header.stamp).seconds();
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
        "MAPPER_DIAG: pose=(%.2f, %.2f, yaw=%.1f°) scan_t=%.3f odom_t=%.3f tf_t=%.3f dt_so=%.3f dt_st=%.3f",
        rx, ry, yaw * 57.2958, scan_t, odom_t, tf_t,
        scan_t - odom_t, scan_t - tf_t);
    }

    // Ray origin from physical lidar mount, not base center.
    const double sx = rx + (std::cos(yaw) * lidar_offset_x_ - std::sin(yaw) * lidar_offset_y_);
    const double sy = ry + (std::sin(yaw) * lidar_offset_x_ + std::cos(yaw) * lidar_offset_y_);

    const auto origin_cell = world_to_grid(sx, sy);
    if (!origin_cell) {
      return;
    }

    double angle = static_cast<double>(scan->angle_min);
    const double inc = static_cast<double>(scan->angle_increment);
    const double range_min = static_cast<double>(scan->range_min);

    for (const float rng_f : scan->ranges) {
      const double rng = static_cast<double>(rng_f);
      const bool finite = std::isfinite(rng);
      if (finite && rng < range_min) {
        angle += inc;
        continue;
      }

      const double dist = finite ? std::min(rng, max_range_) : max_range_;
      const bool has_hit = finite && (rng < max_range_ * 0.99);
      const double world_bearing = yaw + angle;
      const double ex = sx + dist * std::cos(world_bearing);
      const double ey = sy + dist * std::sin(world_bearing);
      const auto end_cell = world_to_grid(ex, ey);

      // For rays that HIT a wall, clear up to the hit point so old
      // ghost wall cells get erased during turns.  For no-hit rays
      // (max-range / infinity), cap at max_clear_distance to avoid
      // erasing distant real walls with long free rays.
      const double clear_dist =
        has_hit ? dist
                : ((max_clear_distance_ <= 0.0) ? dist : std::min(dist, max_clear_distance_));
      const double cex = sx + clear_dist * std::cos(world_bearing);
      const double cey = sy + clear_dist * std::sin(world_bearing);
      const auto clear_end_cell = world_to_grid(cex, cey);

      angle += inc;
      if (!clear_end_cell) {
        continue;
      }

      if (has_hit || clear_on_nohit_) {
        raytrace_free(origin_cell->first, origin_cell->second, clear_end_cell->first, clear_end_cell->second);
      }

      if (has_hit && end_cell) {
        apply_evidence(end_cell->first, end_cell->second, hit_increment_);
      } else if (finite && clear_on_nohit_ && end_cell) {
        apply_evidence(end_cell->first, end_cell->second, -miss_decrement_);
      }
    }

    publish_map(scan->header.stamp);

    // CRITICAL: clear the scan so it's only painted ONCE. Without this,
    // the timer re-paints the same scan with newer odom poses on each tick,
    // creating starburst/doubled-wall artifacts.
    last_scan_.reset();

    if (last_summary_ns_ == 0 || (now_t.nanoseconds() - last_summary_ns_) > static_cast<int64_t>(10e9)) {
      last_summary_ns_ = now_t.nanoseconds();
      int free_n = 0;
      int occ_n = 0;
      for (const int8_t v : grid_) {
        if (v == 0) {
          ++free_n;
        } else if (v == 100) {
          ++occ_n;
        }
      }
      const int unknown_n = static_cast<int>(grid_.size()) - free_n - occ_n;
      RCLCPP_INFO(
        get_logger(), "MAP step: free=%d occ=%d unknown=%d (TF-based)",
        free_n, occ_n, unknown_n);
    }
  }

  std::string scan_topic_;
  std::string odom_topic_;
  std::string map_topic_;
  std::string map_frame_;
  std::string scan_frame_;

  double lidar_offset_x_{0.0};
  double lidar_offset_y_{0.0};
  double resolution_{0.10};
  int width_{400};
  int height_{400};
  double origin_x_{-20.0};
  double origin_y_{-20.0};

  double max_range_{12.0};
  double max_clear_distance_{4.0};
  bool clear_on_nohit_{false};
  double update_rate_{4.0};
  double startup_delay_{4.0};
  int tf_timeout_ms_{1000};
  double max_scan_odom_dt_{0.10};

  int hit_increment_{3};
  int miss_decrement_{1};
  int score_min_{-20};
  int score_max_{20};
  int occupied_score_threshold_{3};
  int free_score_threshold_{-3};

  // Score decay for dynamic obstacles
  double decay_interval_sec_{2.0};
  int decay_amount_{1};

  // Exemption zone (door corridor)
  double exempt_x_min_{0.0}, exempt_x_max_{0.0};
  double exempt_y_min_{0.0}, exempt_y_max_{0.0};
  bool has_exemption_zone_{false};
  int exempt_gx_min_{0}, exempt_gx_max_{0};
  int exempt_gy_min_{0}, exempt_gy_max_{0};

  // Motion compensation
  double pose_filter_alpha_{0.7};
  double max_angular_velocity_{0.4};
  double odom_omega_{0.0};

  // EMA pose filter state
  bool has_filtered_pose_{false};
  double filtered_rx_{0.0};
  double filtered_ry_{0.0};
  double filtered_yaw_{0.0};

  std::vector<int8_t> grid_;
  std::vector<int> scores_;
  std::vector<bool> observed_;

  ScanMsg::SharedPtr last_scan_;
  OdomMsg::SharedPtr last_odom_;

  bool broadcast_tf_{true};
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::optional<rclcpp::Time> start_time_;
  int64_t last_tf_warn_ns_{0};
  int64_t last_summary_ns_{0};

  rclcpp::Subscription<ScanMsg>::SharedPtr scan_sub_;
  rclcpp::Subscription<OdomMsg>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr decay_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SimpleScanMapperCpp>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

