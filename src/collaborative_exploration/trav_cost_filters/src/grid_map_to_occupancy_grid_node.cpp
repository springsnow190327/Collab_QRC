// grid_map_to_occupancy_grid_node — C++ port of the Python adapter.
//
// Converts the traversability layer of a GridMap to a Nav2-style OccupancyGrid.
// Subscribes to a filtered GridMap (rolling, robot-centred). Maintains a
// persistent world-fixed buffer so Nav2 StaticLayer sees a stable origin.
//
// The Python adapter (trav_cost_filters/grid_map_to_occupancy_grid.py +
// occupancy_conversion.py) was CPU-bound on ARM Cortex-A78AE (Jetson Orin
// Nano: 0.59 Hz output on a 200×200-cell fixed grid). This C++ port targets
// the same algorithmic surface area and parameter contract — drop-in
// replacement, no topic / param renames.
//
// Features ported (all toggled by the same parameters as the Python version):
//   1. layer-flat → world (h, w) array (reshape + double-flip)
//   2. traversability → occupancy cost (free/lethal thresholds, linear middle)
//   3. apply_slope_verified_ramp_override (slope+step_residual gate)
//   4. elevation-based extra cost (max-blend with trav cost)
//   5. upper_bound clearance for overhangs / walk-under bridges
//   6. apply_cliff_proximity_cost (radial max-pool by step_height)
//   7. project_rolling_grid_to_fixed_grid (overlap copy, no allocation)
//   8. _update_fixed_grid (hit-count temporal filtering)
//   9. apply_rectangular_workspace_mask (boundary fill)
//  10. seed_robot_footprint (bounded disk stamp via tf2)
//
// Build: see trav_cost_filters/CMakeLists.txt. Depends on rclcpp,
// grid_map_core, grid_map_msgs, grid_map_ros, nav_msgs, tf2_ros.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>


namespace trav_cost_filters
{

class GridMapToOccupancyGridCpp : public rclcpp::Node
{
public:
  GridMapToOccupancyGridCpp()
  : rclcpp::Node("grid_map_to_occupancy_grid")
  {
    // ── Topics + layers ────────────────────────────────────────────────
    input_topic_ = declare_parameter<std::string>("input_topic", "elevation_map_filtered");
    output_topic_ = declare_parameter<std::string>("output_topic", "traversability_grid");
    trav_layer_ = declare_parameter<std::string>("traversability_layer", "trav_eth");

    // ── Threshold params ────────────────────────────────────────────────
    free_threshold_ = declare_parameter<double>("free_threshold", 0.7);
    lethal_threshold_ = declare_parameter<double>("lethal_threshold", 0.3);

    // ── Robot footprint seed ────────────────────────────────────────────
    seed_robot_footprint_ = declare_parameter<bool>("seed_robot_footprint", true);
    robot_frame_ = declare_parameter<std::string>("robot_frame", "base_link");
    robot_seed_radius_m_ = declare_parameter<double>("robot_seed_radius_m", 0.65);
    seed_max_clear_cost_ = declare_parameter<int>("seed_max_clear_cost", 50);

    // ── Ramp override ──────────────────────────────────────────────────
    ramp_override_enabled_ = declare_parameter<bool>("ramp_override_enabled", false);
    slope_layer_ = declare_parameter<std::string>("slope_layer", "slope");
    step_residual_layer_ = declare_parameter<std::string>("step_residual_layer", "step_residual");
    ramp_min_slope_rad_ = declare_parameter<double>("ramp_min_slope_rad", 0.13962634015954636);
    ramp_max_slope_rad_ = declare_parameter<double>("ramp_max_slope_rad", 0.5235987755982988);
    ramp_max_step_residual_m_ = declare_parameter<double>("ramp_max_step_residual_m", 0.06);

    // ── Elevation-based cost ───────────────────────────────────────────
    elevation_cost_enabled_ = declare_parameter<bool>("elevation_cost_enabled", false);
    elevation_layer_ = declare_parameter<std::string>("elevation_layer", "elevation");
    elevation_cost_min_h_ = declare_parameter<double>("elevation_cost_min_h", 0.05);
    elevation_cost_max_h_ = declare_parameter<double>("elevation_cost_max_h", 1.00);
    elevation_cost_max_value_ = declare_parameter<int>("elevation_cost_max_value", 90);

    // ── Cliff proximity cost ────────────────────────────────────────────
    cliff_proximity_cost_enabled_ = declare_parameter<bool>("cliff_proximity_cost_enabled", false);
    cliff_step_layer_ = declare_parameter<std::string>("cliff_step_layer", "step_height");
    cliff_proximity_radius_m_ = declare_parameter<double>("cliff_proximity_radius_m", 0.25);
    cliff_step_threshold_m_ = declare_parameter<double>("cliff_step_threshold_m", 0.30);
    cliff_step_saturation_m_ = declare_parameter<double>("cliff_step_saturation_m", 0.45);
    cliff_proximity_cost_max_value_ = declare_parameter<int>("cliff_proximity_cost_max_value", 90);

    // ── Upper-bound overhang clearance ─────────────────────────────────
    upper_bound_clearance_enabled_ =
      declare_parameter<bool>("upper_bound_clearance_enabled", false);
    upper_bound_layer_ = declare_parameter<std::string>("upper_bound_layer", "upper_bound");
    upper_bound_overhang_threshold_m_ =
      declare_parameter<double>("upper_bound_overhang_threshold_m", 0.30);
    upper_bound_clear_cost_ = declare_parameter<int>("upper_bound_clear_cost", 0);

    // ── Fixed-grid (hit-counted) mode ───────────────────────────────────
    fixed_grid_enabled_ = declare_parameter<bool>("fixed_grid_enabled", false);
    fixed_origin_x_ = declare_parameter<double>("fixed_origin_x", 0.0);
    fixed_origin_y_ = declare_parameter<double>("fixed_origin_y", 0.0);
    fixed_width_cells_ = declare_parameter<int>("fixed_width_cells", 0);
    fixed_height_cells_ = declare_parameter<int>("fixed_height_cells", 0);
    unknown_clears_history_ = declare_parameter<bool>("unknown_clears_history", false);
    occupied_cost_threshold_ = declare_parameter<int>("occupied_cost_threshold", 100);
    free_cost_threshold_ = declare_parameter<int>("free_cost_threshold", 30);
    occupied_confirm_hits_ = declare_parameter<int>("occupied_confirm_hits", 2);
    occupied_clear_hits_ = declare_parameter<int>("occupied_clear_hits", 0);
    occupied_hit_increment_ = declare_parameter<int>("occupied_hit_increment", 1);
    free_hit_decrement_ = declare_parameter<int>("free_hit_decrement", 1);
    max_hit_count_ = declare_parameter<int>("max_hit_count", 8);

    // ── Workspace boundary mask ────────────────────────────────────────
    workspace_mask_enabled_ = declare_parameter<bool>("workspace_mask_enabled", false);
    workspace_min_x_ = declare_parameter<double>("workspace_min_x", 0.0);
    workspace_max_x_ = declare_parameter<double>("workspace_max_x", 0.0);
    workspace_min_y_ = declare_parameter<double>("workspace_min_y", 0.0);
    workspace_max_y_ = declare_parameter<double>("workspace_max_y", 0.0);
    workspace_wall_thickness_m_ = declare_parameter<double>("workspace_wall_thickness_m", 0.0);

    if (seed_robot_footprint_) {
      tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);
    }

    // ── QoS: publisher TRANSIENT_LOCAL (Nav2/CFPA2 see last map on join). ─
    // Subscriber depth=5 so back-to-back elevation_map frames don't drop while
    // a previous callback is mid-flight (the trav-grid publish at 4MB/msg
    // takes ~10ms in DDS reliable mode on Jetson; input arrives every ~250ms
    // so 5 slots is comfortable headroom).
    rclcpp::QoS pub_qos(rclcpp::KeepLast(1));
    pub_qos.reliable().transient_local();
    rclcpp::QoS sub_qos(rclcpp::KeepLast(5));
    sub_qos.reliable();

    pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(output_topic_, pub_qos);
    sub_ = create_subscription<grid_map_msgs::msg::GridMap>(
      input_topic_, sub_qos,
      [this](const grid_map_msgs::msg::GridMap::SharedPtr msg) { on_map(msg); });

    RCLCPP_INFO(
      get_logger(),
      "grid_map_to_occupancy_grid (C++): %s → %s layer=%s "
      "free>=%.3f lethal<%.3f seed_robot_footprint=%d robot_frame=%s radius=%.2fm "
      "ramp_override=%d cliff_proximity=%d fixed_grid=%d",
      input_topic_.c_str(), output_topic_.c_str(), trav_layer_.c_str(),
      free_threshold_, lethal_threshold_, seed_robot_footprint_ ? 1 : 0,
      robot_frame_.c_str(), robot_seed_radius_m_,
      ramp_override_enabled_ ? 1 : 0, cliff_proximity_cost_enabled_ ? 1 : 0,
      fixed_grid_enabled_ ? 1 : 0);
  }

private:
  // ───── Layer access ──────────────────────────────────────────────────
  // Extract a layer into a (n_y, n_x) row-major buffer in world-XY convention
  // (row 0 = min-y, col 0 = min-x). GridMap publishes flat data that, when
  // viewed as (n_y, n_x) C-order, has both axes max→min — so the conversion
  // is a single 180° rotation (equivalent to reverse(buf)).
  // Returns false if the layer is missing or has wrong size.
  bool layer_to_world(
    const grid_map_msgs::msg::GridMap & msg,
    const std::string & layer_name,
    int n_y, int n_x,
    std::vector<float> & out) const
  {
    auto it = std::find(msg.layers.begin(), msg.layers.end(), layer_name);
    if (it == msg.layers.end()) {
      return false;
    }
    const size_t idx = static_cast<size_t>(std::distance(msg.layers.begin(), it));
    const auto & src = msg.data[idx].data;
    const int N = n_y * n_x;
    if (static_cast<int>(src.size()) != N) {
      return false;
    }
    out.resize(N);
    // arr.reshape(n_y, n_x)[::-1, ::-1]  ≡  out[i] = src[N - 1 - i]
    for (int i = 0; i < N; ++i) {
      out[i] = src[N - 1 - i];
    }
    return true;
  }

  // ───── traversability_to_occupancy ───────────────────────────────────
  // trav ∈ [0,1] (NaN → -1 unknown). cost ∈ {-1} ∪ [0, 100]. Free above
  // free_threshold (→0), lethal below lethal_threshold (→100), middle band
  // linearly mapped to 1..99 (clipped).
  void trav_to_occupancy(
    const std::vector<float> & trav,
    std::vector<int8_t> & cost,
    int n_y, int n_x) const
  {
    const int N = n_y * n_x;
    cost.assign(N, -1);
    if (free_threshold_ <= lethal_threshold_) {
      return;
    }
    const float ft = static_cast<float>(free_threshold_);
    const float lt = static_cast<float>(lethal_threshold_);
    const float denom = ft - lt;
    for (int i = 0; i < N; ++i) {
      const float t = trav[i];
      if (!std::isfinite(t)) { cost[i] = -1; continue; }
      const float tc = std::clamp(t, 0.0f, 1.0f);
      if (tc >= ft) {
        cost[i] = 0;
      } else if (tc < lt) {
        cost[i] = 100;
      } else {
        int v = static_cast<int>(std::lround((ft - tc) / denom * 100.0f));
        cost[i] = static_cast<int8_t>(std::clamp(v, 1, 99));
      }
    }
  }

  // ───── apply_slope_verified_ramp_override ────────────────────────────
  // Clears (→ free) cells where (min_slope ≤ slope ≤ max_slope) ∧
  // (step_residual ≤ max_step_residual). Returns # cells changed.
  int apply_ramp_override(
    std::vector<int8_t> & cost,
    const std::vector<float> & slope,
    const std::vector<float> & step_residual,
    int N) const
  {
    int changed = 0;
    const float lo = static_cast<float>(ramp_min_slope_rad_);
    const float hi = static_cast<float>(ramp_max_slope_rad_);
    const float step_max = static_cast<float>(ramp_max_step_residual_m_);
    for (int i = 0; i < N; ++i) {
      const float s = slope[i];
      const float sr = step_residual[i];
      if (std::isfinite(s) && std::isfinite(sr) &&
          s >= lo && s <= hi && sr <= step_max && cost[i] != 0)
      {
        cost[i] = 0;
        ++changed;
      }
    }
    return changed;
  }

  // ───── elevation_cost: max-blend height penalty into known cells ──────
  void apply_elevation_cost(
    std::vector<int8_t> & cost,
    const std::vector<float> & elev,
    int N) const
  {
    const float h_min = static_cast<float>(elevation_cost_min_h_);
    const float h_max = static_cast<float>(elevation_cost_max_h_);
    const float v_max = static_cast<float>(elevation_cost_max_value_);
    const float denom = std::max(1e-6f, h_max - h_min);
    for (int i = 0; i < N; ++i) {
      if (cost[i] < 0) { continue; }  // unknown stays unknown
      const float e = elev[i];
      if (!std::isfinite(e)) { continue; }
      float over = std::clamp((e - h_min) / denom, 0.0f, 1.0f);
      int h_cost = static_cast<int>(std::lround(over * v_max));
      if (h_cost > cost[i]) {
        cost[i] = static_cast<int8_t>(h_cost);
      }
    }
  }

  // ───── upper_bound clearance: overhang → free ─────────────────────────
  int apply_upper_bound_clearance(
    std::vector<int8_t> & cost,
    const std::vector<float> & elev,
    const std::vector<float> & ubnd,
    int N) const
  {
    int cleared = 0;
    const float thresh = static_cast<float>(upper_bound_overhang_threshold_m_);
    const int8_t v = static_cast<int8_t>(upper_bound_clear_cost_);
    for (int i = 0; i < N; ++i) {
      const float e = elev[i];
      const float u = ubnd[i];
      if (std::isfinite(e) && std::isfinite(u) && (e - u) > thresh) {
        if (cost[i] != v) { cost[i] = v; ++cleared; }
      }
    }
    return cleared;
  }

  // ───── apply_cliff_proximity_cost: radial max-pool over step_height ──
  // For each cell (r, c), find max(step_height) within proximity_radius.
  // Map to cost via (local_max - threshold) / (sat - threshold) * max_cost,
  // applied only to known cells where it would RAISE existing cost.
  int apply_cliff_proximity(
    std::vector<int8_t> & cost,
    const std::vector<float> & step_height,
    int n_y, int n_x, double resolution) const
  {
    if (resolution <= 0.0 || cliff_proximity_radius_m_ <= 0.0) { return 0; }
    if (cliff_step_saturation_m_ <= cliff_step_threshold_m_) { return 0; }

    const int N = n_y * n_x;
    const int radius_cells = static_cast<int>(
      std::ceil(cliff_proximity_radius_m_ / resolution));
    if (radius_cells <= 0) { return 0; }

    std::vector<float> local_max(N, -std::numeric_limits<float>::infinity());
    // Precompute valid (dx, dy) offset list (circular mask)
    const float r2_max = static_cast<float>(
      (cliff_proximity_radius_m_ + 1e-9) * (cliff_proximity_radius_m_ + 1e-9));
    std::vector<std::pair<int, int>> offsets;
    offsets.reserve((2 * radius_cells + 1) * (2 * radius_cells + 1));
    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
      for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
        const float dxm = dx * static_cast<float>(resolution);
        const float dym = dy * static_cast<float>(resolution);
        if (dxm * dxm + dym * dym <= r2_max) {
          offsets.emplace_back(dy, dx);
        }
      }
    }

    // Box-walk: for each cell, max-pool over the circular neighborhood.
    // Cost per cell: O(R²); over N cells: O(N · R²). For R=2 cells, R²=21
    // valid offsets — fast enough at 200x200 grid.
    for (int r = 0; r < n_y; ++r) {
      for (int c = 0; c < n_x; ++c) {
        float m = -std::numeric_limits<float>::infinity();
        for (const auto & off : offsets) {
          const int rr = r + off.first;
          const int cc = c + off.second;
          if (rr < 0 || rr >= n_y || cc < 0 || cc >= n_x) { continue; }
          const float s = step_height[rr * n_x + cc];
          if (std::isfinite(s) && s > m) { m = s; }
        }
        local_max[r * n_x + c] = m;
      }
    }

    const float denom = static_cast<float>(cliff_step_saturation_m_ - cliff_step_threshold_m_);
    int changed = 0;
    for (int i = 0; i < N; ++i) {
      if (cost[i] < 0) { continue; }
      const float m = local_max[i];
      if (!std::isfinite(m)) { continue; }
      float risk = std::clamp(
        (m - static_cast<float>(cliff_step_threshold_m_)) / denom, 0.0f, 1.0f);
      int risk_cost = static_cast<int>(std::lround(risk * cliff_proximity_cost_max_value_));
      if (risk_cost > cost[i]) {
        cost[i] = static_cast<int8_t>(risk_cost);
        ++changed;
      }
    }
    return changed;
  }

  // ───── First-frame-locked buffer (default mode) ──────────────────────
  // Lock dimensions + origin on first frame; project each subsequent
  // rolling-window into the SAME world-fixed grid (overlap copy). No
  // hit-counting, no thresholds — simplest stable mode.
  void project_rolling_into_locked_buffer(
    const std::vector<int8_t> & cost,    // rolling-window cost
    int n_y, int n_x,                    // rolling dims
    double roll_ox, double roll_oy,      // rolling origin (world)
    double resolution,                   // m/cell
    const std::string & frame_id)
  {
    if (!locked_buf_initialized_) {
      locked_ox_ = roll_ox;
      locked_oy_ = roll_oy;
      locked_res_ = resolution;
      locked_fw_ = n_x;
      locked_fh_ = n_y;
      locked_frame_id_ = frame_id;
      locked_buf_.assign(static_cast<size_t>(n_y) * n_x, -1);
      locked_buf_initialized_ = true;
      RCLCPP_INFO(
        get_logger(),
        "Fixed-origin buffer initialised: origin=(%.2f, %.2f) size=%dx%d @ %.3fm/cell",
        locked_ox_, locked_oy_, n_x, n_y, resolution);
    }

    const int fw = locked_fw_;
    const int fh = locked_fh_;

    const int dc = static_cast<int>(std::lround((roll_ox - locked_ox_) / resolution));
    const int dr = static_cast<int>(std::lround((roll_oy - locked_oy_) / resolution));

    const int R_lo = std::max(0, dr);
    const int R_hi = std::min(fh, dr + n_y);
    const int C_lo = std::max(0, dc);
    const int C_hi = std::min(fw, dc + n_x);
    if (R_hi <= R_lo || C_hi <= C_lo) { return; }

    const int r_off = R_lo - dr;
    const int c_off = C_lo - dc;
    for (int R = R_lo; R < R_hi; ++R) {
      const int r = r_off + (R - R_lo);
      const int8_t * src_row = &cost[r * n_x + c_off];
      int8_t * dst_row = &locked_buf_[R * fw + C_lo];
      const int w = C_hi - C_lo;
      for (int k = 0; k < w; ++k) {
        if (src_row[k] >= 0) { dst_row[k] = src_row[k]; }
      }
    }
  }

  // ───── Fixed-grid hit-counted mode ──────────────────────────────────
  // (fixed_grid_enabled=true) — fully configurable origin/size, temporal
  // filtering via hit counts so single-frame lidar speckles don't latch.
  void update_fixed_grid(
    const std::vector<int8_t> & rolling_cost,
    int rolling_h, int rolling_w,
    double rolling_origin_x, double rolling_origin_y,
    double resolution)
  {
    const int width  = (fixed_width_cells_ > 0)  ? fixed_width_cells_  : rolling_w;
    const int height = (fixed_height_cells_ > 0) ? fixed_height_cells_ : rolling_h;
    const size_t shape_n = static_cast<size_t>(height) * width;

    if (fixed_cost_.size() != shape_n || std::abs(fixed_res_ - resolution) > 1e-9) {
      fixed_cost_.assign(shape_n, -1);
      fixed_hits_.assign(shape_n, 0);
      fixed_res_ = resolution;
      fixed_width_ = width;
      fixed_height_ = height;
      RCLCPP_INFO(
        get_logger(),
        "fixed traversability grid initialized: %dx%d res=%.3f origin=(%.2f,%.2f)",
        width, height, resolution, fixed_origin_x_, fixed_origin_y_);
    }

    // For each rolling cell, compute destination world index and apply
    // hit-count logic. The Python version vectorises by pre-mapping rows/
    // cols and using np.meshgrid; here we just iterate (fast enough at
    // typical 300×300 input, no per-cell allocation).
    const int dst_w = fixed_width_;
    const int dst_h = fixed_height_;
    for (int r = 0; r < rolling_h; ++r) {
      const double yw = rolling_origin_y + (r + 0.5) * resolution;
      const int64_t dr = static_cast<int64_t>(
        std::floor((yw - fixed_origin_y_) / resolution + 1e-6));
      if (dr < 0 || dr >= dst_h) { continue; }
      for (int c = 0; c < rolling_w; ++c) {
        const double xw = rolling_origin_x + (c + 0.5) * resolution;
        const int64_t dc = static_cast<int64_t>(
          std::floor((xw - fixed_origin_x_) / resolution + 1e-6));
        if (dc < 0 || dc >= dst_w) { continue; }

        const int8_t v = rolling_cost[r * rolling_w + c];
        const size_t fidx = static_cast<size_t>(dr) * dst_w + dc;

        if (v < 0) {  // unknown
          if (unknown_clears_history_) {
            fixed_cost_[fidx] = -1;
            fixed_hits_[fidx] = 0;
          }
          continue;
        }

        // Observed (known) cell — apply hit-count temporal filter.
        if (v >= static_cast<int8_t>(occupied_cost_threshold_)) {
          // Lethal observation
          int16_t h = fixed_hits_[fidx] + static_cast<int16_t>(occupied_hit_increment_);
          h = std::min<int16_t>(h, static_cast<int16_t>(max_hit_count_));
          fixed_hits_[fidx] = h;
          if (h >= static_cast<int16_t>(occupied_confirm_hits_)) {
            fixed_cost_[fidx] = 100;
          }
        } else {
          // Free / middle observation
          int16_t h = fixed_hits_[fidx] - static_cast<int16_t>(free_hit_decrement_);
          h = std::max<int16_t>(h, 0);
          fixed_hits_[fidx] = h;
          const int8_t prev = fixed_cost_[fidx];
          // can_update = (prev != 100) | (new_hits <= occupied_clear_hits)
          if (prev != 100 || h <= static_cast<int16_t>(occupied_clear_hits_)) {
            fixed_cost_[fidx] = v;
          }
        }
      }
    }
  }

  // ───── Rectangular workspace mask ───────────────────────────────────
  void apply_workspace_mask(
    std::vector<int8_t> & grid,
    int width, int height,
    double origin_x, double origin_y, double resolution) const
  {
    if (workspace_max_x_ <= workspace_min_x_ || workspace_max_y_ <= workspace_min_y_) {
      return;
    }
    const double wall = workspace_wall_thickness_m_;
    for (int r = 0; r < height; ++r) {
      const double y = origin_y + (r + 0.5) * resolution;
      for (int c = 0; c < width; ++c) {
        const double x = origin_x + (c + 0.5) * resolution;
        const size_t idx = static_cast<size_t>(r) * width + c;
        const bool inside =
          (x >= workspace_min_x_) && (x <= workspace_max_x_) &&
          (y >= workspace_min_y_) && (y <= workspace_max_y_);
        if (!inside) {
          grid[idx] = -1;
        } else if (wall > 0.0 && (
          x <= workspace_min_x_ + wall ||
          x >= workspace_max_x_ - wall ||
          y <= workspace_min_y_ + wall ||
          y >= workspace_max_y_ - wall))
        {
          grid[idx] = 100;
        }
      }
    }
  }

  // ───── seed_robot_footprint: bounded disk via tf2 lookup ───────────
  void seed_robot_footprint_disk(
    std::vector<int8_t> & grid,
    int width, int height,
    double origin_x, double origin_y, double resolution,
    const std::string & frame_id)
  {
    if (!seed_robot_footprint_ || !tf_buffer_) { return; }
    geometry_msgs::msg::TransformStamped tf_msg;
    try {
      tf_msg = tf_buffer_->lookupTransform(
        frame_id, robot_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      auto & clk = *this->get_clock();
      RCLCPP_WARN_THROTTLE(
        get_logger(), clk, 5000,
        "robot footprint seed skipped: cannot transform %s <- %s: %s",
        frame_id.c_str(), robot_frame_.c_str(), ex.what());
      return;
    }
    const double cx = tf_msg.transform.translation.x;
    const double cy = tf_msg.transform.translation.y;
    const double r2 = robot_seed_radius_m_ * robot_seed_radius_m_;
    if (robot_seed_radius_m_ <= 0.0 || resolution <= 0.0) { return; }

    const int min_x = std::max(0, static_cast<int>(
      std::floor((cx - robot_seed_radius_m_ - origin_x) / resolution)));
    const int max_x = std::min(width - 1, static_cast<int>(
      std::floor((cx + robot_seed_radius_m_ - origin_x) / resolution)));
    const int min_y = std::max(0, static_cast<int>(
      std::floor((cy - robot_seed_radius_m_ - origin_y) / resolution)));
    const int max_y = std::min(height - 1, static_cast<int>(
      std::floor((cy + robot_seed_radius_m_ - origin_y) / resolution)));
    if (min_x > max_x || min_y > max_y) { return; }

    const int8_t free_v = 0;
    const int8_t clear_max = static_cast<int8_t>(seed_max_clear_cost_);
    int changed = 0;
    for (int r = min_y; r <= max_y; ++r) {
      const double yw = origin_y + (r + 0.5) * resolution;
      const double dy = yw - cy;
      for (int c = min_x; c <= max_x; ++c) {
        const double xw = origin_x + (c + 0.5) * resolution;
        const double dx = xw - cx;
        if (dx * dx + dy * dy > r2) { continue; }
        const size_t idx = static_cast<size_t>(r) * width + c;
        const int8_t prev = grid[idx];
        const bool clearable = (prev < 0) || (prev >= 0 && prev <= clear_max);
        if (clearable && prev != free_v) {
          grid[idx] = free_v;
          ++changed;
        }
      }
    }
    if (changed > 0) {
      RCLCPP_DEBUG(
        get_logger(),
        "seeded %d robot-footprint cells free at (%.2f, %.2f)",
        changed, cx, cy);
    }
  }

  // ───── Main callback ────────────────────────────────────────────────
  void on_map(const grid_map_msgs::msg::GridMap::SharedPtr msg)
  {
    // Layer presence check.
    auto it = std::find(msg->layers.begin(), msg->layers.end(), trav_layer_);
    if (it == msg->layers.end()) {
      auto & clk = *this->get_clock();
      RCLCPP_WARN_THROTTLE(
        get_logger(), clk, 5000,
        "layer '%s' not in GridMap", trav_layer_.c_str());
      return;
    }

    const auto & info = msg->info;
    const double res = info.resolution;
    const int n_y = static_cast<int>(std::lround(info.length_y / res));
    const int n_x = static_cast<int>(std::lround(info.length_x / res));
    const int N = n_y * n_x;

    // ── Layer extraction (double-flip → world XY layout) ─────────────
    std::vector<float> trav;
    if (!layer_to_world(*msg, trav_layer_, n_y, n_x, trav)) {
      auto & clk = *this->get_clock();
      RCLCPP_WARN_THROTTLE(
        get_logger(), clk, 5000,
        "size mismatch on layer '%s'", trav_layer_.c_str());
      return;
    }

    // ── Convert to occupancy cost ────────────────────────────────────
    std::vector<int8_t> cost;
    trav_to_occupancy(trav, cost, n_y, n_x);

    // ── Optional: ramp override ──────────────────────────────────────
    if (ramp_override_enabled_) {
      std::vector<float> slope, step_residual;
      if (layer_to_world(*msg, slope_layer_, n_y, n_x, slope) &&
          layer_to_world(*msg, step_residual_layer_, n_y, n_x, step_residual))
      {
        apply_ramp_override(cost, slope, step_residual, N);
      }
    }

    // ── Optional: elevation cost ─────────────────────────────────────
    if (elevation_cost_enabled_) {
      std::vector<float> elev;
      if (layer_to_world(*msg, elevation_layer_, n_y, n_x, elev) &&
          static_cast<int>(elev.size()) == N)
      {
        apply_elevation_cost(cost, elev, N);
      }
    }

    // ── Optional: upper_bound overhang clearance ─────────────────────
    if (upper_bound_clearance_enabled_) {
      std::vector<float> elev, ubnd;
      if (layer_to_world(*msg, elevation_layer_, n_y, n_x, elev) &&
          layer_to_world(*msg, upper_bound_layer_, n_y, n_x, ubnd))
      {
        apply_upper_bound_clearance(cost, elev, ubnd, N);
      }
    }

    // ── Optional: cliff proximity cost (radial max-pool) ─────────────
    if (cliff_proximity_cost_enabled_) {
      std::vector<float> step_height;
      if (layer_to_world(*msg, cliff_step_layer_, n_y, n_x, step_height)) {
        apply_cliff_proximity(cost, step_height, n_y, n_x, res);
      }
    }

    // ── Rolling-window world origin ──────────────────────────────────
    const double roll_ox = info.pose.position.x - info.length_x / 2.0;
    const double roll_oy = info.pose.position.y - info.length_y / 2.0;

    // ── Project rolling → locked persistent buffer (default mode) ────
    project_rolling_into_locked_buffer(
      cost, n_y, n_x, roll_ox, roll_oy, res, msg->header.frame_id);

    // ── Build outgoing OccupancyGrid ──────────────────────────────────
    auto occ = std::make_unique<nav_msgs::msg::OccupancyGrid>();
    occ->header.stamp = msg->header.stamp;
    occ->header.frame_id = locked_frame_id_;
    occ->info.resolution = res;
    occ->info.origin.orientation.w = 1.0;
    occ->info.origin.position.z = 0.0;

    if (fixed_grid_enabled_) {
      // Hit-counted fixed grid: configurable origin/size, optional workspace mask.
      update_fixed_grid(cost, n_y, n_x, roll_ox, roll_oy, res);
      if (workspace_mask_enabled_) {
        apply_workspace_mask(
          fixed_cost_, fixed_width_, fixed_height_,
          fixed_origin_x_, fixed_origin_y_, res);
      }
      occ->info.width = fixed_width_;
      occ->info.height = fixed_height_;
      occ->info.origin.position.x = fixed_origin_x_;
      occ->info.origin.position.y = fixed_origin_y_;
      seed_robot_footprint_disk(
        fixed_cost_, fixed_width_, fixed_height_,
        fixed_origin_x_, fixed_origin_y_, res, occ->header.frame_id);
      occ->data.assign(fixed_cost_.begin(), fixed_cost_.end());
    } else {
      // Locked-buffer mode (Python's default path).
      occ->info.width = locked_fw_;
      occ->info.height = locked_fh_;
      occ->info.origin.position.x = locked_ox_;
      occ->info.origin.position.y = locked_oy_;
      seed_robot_footprint_disk(
        locked_buf_, locked_fw_, locked_fh_,
        locked_ox_, locked_oy_, res, occ->header.frame_id);
      occ->data.assign(locked_buf_.begin(), locked_buf_.end());
    }

    pub_->publish(std::move(occ));
  }

  // ─── Params ───────────────────────────────────────────────────────
  std::string input_topic_, output_topic_, trav_layer_;
  double free_threshold_{0.7}, lethal_threshold_{0.3};
  bool seed_robot_footprint_{true};
  std::string robot_frame_{"base_link"};
  double robot_seed_radius_m_{0.65};
  int seed_max_clear_cost_{50};

  bool ramp_override_enabled_{false};
  std::string slope_layer_{"slope"}, step_residual_layer_{"step_residual"};
  double ramp_min_slope_rad_{0.139}, ramp_max_slope_rad_{0.524};
  double ramp_max_step_residual_m_{0.06};

  bool elevation_cost_enabled_{false};
  std::string elevation_layer_{"elevation"};
  double elevation_cost_min_h_{0.05}, elevation_cost_max_h_{1.0};
  int elevation_cost_max_value_{90};

  bool cliff_proximity_cost_enabled_{false};
  std::string cliff_step_layer_{"step_height"};
  double cliff_proximity_radius_m_{0.25};
  double cliff_step_threshold_m_{0.30}, cliff_step_saturation_m_{0.45};
  int cliff_proximity_cost_max_value_{90};

  bool upper_bound_clearance_enabled_{false};
  std::string upper_bound_layer_{"upper_bound"};
  double upper_bound_overhang_threshold_m_{0.30};
  int upper_bound_clear_cost_{0};

  bool fixed_grid_enabled_{false};
  double fixed_origin_x_{0.0}, fixed_origin_y_{0.0};
  int fixed_width_cells_{0}, fixed_height_cells_{0};
  bool unknown_clears_history_{false};
  int occupied_cost_threshold_{100}, free_cost_threshold_{30};
  int occupied_confirm_hits_{2}, occupied_clear_hits_{0};
  int occupied_hit_increment_{1}, free_hit_decrement_{1};
  int max_hit_count_{8};

  bool workspace_mask_enabled_{false};
  double workspace_min_x_{0.0}, workspace_max_x_{0.0};
  double workspace_min_y_{0.0}, workspace_max_y_{0.0};
  double workspace_wall_thickness_m_{0.0};

  // ─── State ────────────────────────────────────────────────────────
  // Locked-buffer mode (default)
  bool locked_buf_initialized_{false};
  std::vector<int8_t> locked_buf_;
  double locked_ox_{0.0}, locked_oy_{0.0}, locked_res_{0.0};
  int locked_fw_{0}, locked_fh_{0};
  std::string locked_frame_id_;

  // Fixed-grid mode (hit-counted)
  std::vector<int8_t> fixed_cost_;
  std::vector<int16_t> fixed_hits_;
  double fixed_res_{0.0};
  int fixed_width_{0}, fixed_height_{0};

  // ROS interfaces
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_;
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr sub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace trav_cost_filters


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<trav_cost_filters::GridMapToOccupancyGridCpp>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
