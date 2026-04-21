/*
 * mppi_nav_node.cpp — MPPI-based local navigation (C++ ROS2 node).
 *
 * Drop-in replacement for reactive_nav_node.  Same topic interface.
 *
 * Instead of RRT* + pure-pursuit, this node uses Model Predictive Path
 * Integral (MPPI) control: it forward-simulates many candidate trajectories
 * through a unicycle dynamics model, scores them against a cost function
 * that includes obstacle proximity (distance field), goal progress, heading
 * alignment, and smoothness, then takes the exponentially-weighted average
 * of the best trajectories as the command.
 *
 * Every candidate trajectory is kinematically feasible by construction,
 * so the robot never receives a path it can't track.
 *
 * Retains the global coarse A* from reactive_nav for long-range routing,
 * and the reactive scan-based safety layer for emergency braking.
 */

#include <cmath>
#include <cstdint>
#include <cstring>
#include <algorithm>
#include <limits>
#include <queue>
#include <random>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker.hpp>

// ─── Helpers ──────────────────────────────────────────────────────────

static inline double wrap_angle(double a) {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

// ─── Grid Builder ─────────────────────────────────────────────────────

struct LocalGrid {
    std::vector<uint8_t> data;  // row-major, 0=free, 1=blocked
    int n;                      // grid dimension (square)
    double resolution;

    int center() const { return n / 2; }

    bool in_bounds(int x, int y) const {
        return x >= 0 && x < n && y >= 0 && y < n;
    }

    bool blocked(int x, int y) const {
        return !in_bounds(x, y) || data[y * n + x];
    }

    void set_blocked(int x, int y) {
        if (in_bounds(x, y)) data[y * n + x] = 1;
    }

    void set_free(int x, int y) {
        if (in_bounds(x, y)) data[y * n + x] = 0;
    }

    bool local_to_cell(double lx, double ly, int &gx, int &gy) const {
        int c = center();
        gx = static_cast<int>(std::round(lx / resolution)) + c;
        gy = static_cast<int>(std::round(ly / resolution)) + c;
        return in_bounds(gx, gy);
    }

    void cell_to_local(int gx, int gy, double &lx, double &ly) const {
        int c = center();
        lx = (gx - c) * resolution;
        ly = (gy - c) * resolution;
    }
};

static void build_local_grid(
    const nav_msgs::msg::OccupancyGrid *map,
    LocalGrid &grid,
    double robot_x, double robot_y, double robot_yaw,
    double inflation_radius,
    double start_clearance_radius,
    bool unknown_is_obstacle,
    int map_occupied_thresh)
{
    int c = grid.center();
    double res = grid.resolution;
    int n = grid.n;
    uint8_t unknown_val = unknown_is_obstacle ? 1 : 0;

    if (map && !map->data.empty()) {
        double map_ox = map->info.origin.position.x;
        double map_oy = map->info.origin.position.y;
        double map_res = map->info.resolution;
        int map_w = static_cast<int>(map->info.width);
        int map_h = static_cast<int>(map->info.height);
        double cos_y = std::cos(robot_yaw), sin_y = std::sin(robot_yaw);

        for (int gy = 0; gy < n; gy++) {
            for (int gx = 0; gx < n; gx++) {
                double lx = (gx - c) * res;
                double ly = (gy - c) * res;
                double wx = robot_x + cos_y * lx - sin_y * ly;
                double wy = robot_y + sin_y * lx + cos_y * ly;

                int mx = static_cast<int>((wx - map_ox) / map_res);
                int my = static_cast<int>((wy - map_oy) / map_res);

                if (mx >= 0 && mx < map_w && my >= 0 && my < map_h) {
                    int8_t val = map->data[my * map_w + mx];
                    if (val < 0) {
                        grid.data[gy * n + gx] = unknown_val;
                    } else if (val >= map_occupied_thresh) {
                        grid.data[gy * n + gx] = 1;
                    } else {
                        grid.data[gy * n + gx] = 0;
                    }
                } else {
                    grid.data[gy * n + gx] = unknown_val;
                }
            }
        }
    } else {
        std::fill(grid.data.begin(), grid.data.end(), unknown_val);
    }

    // Inflate obstacles
    std::vector<std::pair<int,int>> obstacles;
    for (int gy = 0; gy < n; gy++)
        for (int gx = 0; gx < n; gx++)
            if (grid.data[gy * n + gx] == 1)
                obstacles.emplace_back(gx, gy);

    int inflate_cells = std::max(0, static_cast<int>(std::ceil(inflation_radius / res)));
    if (inflate_cells > 0) {
        int isq = inflate_cells * inflate_cells;
        for (auto [ox, oy] : obstacles) {
            for (int dy = -inflate_cells; dy <= inflate_cells; dy++) {
                for (int dx = -inflate_cells; dx <= inflate_cells; dx++) {
                    if (dx * dx + dy * dy > isq) continue;
                    grid.set_blocked(ox + dx, oy + dy);
                }
            }
        }
    }

    // Clear around start so robot isn't trapped
    grid.set_free(c, c);
    int clear_cells = std::max(0, static_cast<int>(std::round(start_clearance_radius / res)));
    if (clear_cells > 0) {
        int csq = clear_cells * clear_cells;
        for (int dy = -clear_cells; dy <= clear_cells; dy++) {
            for (int dx = -clear_cells; dx <= clear_cells; dx++) {
                if (dx * dx + dy * dy > csq) continue;
                grid.set_free(c + dx, c + dy);
            }
        }
    }
}

// ─── Distance Field (BFS on local grid) ──────────────────────────────
// Chebyshev distance in cells from nearest obstacle.  Multiply by
// resolution for approximate metres.

static void build_distance_field(const LocalGrid &grid,
                                  std::vector<int> &field) {
    int n = grid.n;
    field.assign(n * n, n * n);
    std::queue<std::pair<int,int>> q;

    for (int y = 0; y < n; y++)
        for (int x = 0; x < n; x++)
            if (grid.blocked(x, y)) { field[y * n + x] = 0; q.push({x, y}); }

    while (!q.empty()) {
        auto [cx, cy] = q.front(); q.pop();
        int cd = field[cy * n + cx];
        for (int dy = -1; dy <= 1; dy++) {
            for (int dx = -1; dx <= 1; dx++) {
                if (!dx && !dy) continue;
                int nx = cx + dx, ny = cy + dy;
                if (nx >= 0 && nx < n && ny >= 0 && ny < n &&
                    field[ny * n + nx] > cd + 1) {
                    field[ny * n + nx] = cd + 1;
                    q.push({nx, ny});
                }
            }
        }
    }
}

// ─── Scan Metrics (reactive safety layer) ────────────────────────────

struct ScanMetrics {
    double min_front = std::numeric_limits<double>::infinity();
    double left_push = 0.0;
    double right_push = 0.0;
};

static ScanMetrics analyze_scan(const sensor_msgs::msg::LaserScan &scan,
                                 double front_half, double side_half,
                                 double slow_dist) {
    ScanMetrics m;
    double left_acc = 0, right_acc = 0, left_w = 0, right_w = 0;

    double angle = scan.angle_min;
    for (size_t i = 0; i < scan.ranges.size(); i++) {
        float r = scan.ranges[i];
        if (!std::isfinite(r) || r < 0.05f) { angle += scan.angle_increment; continue; }
        double bearing = wrap_angle(angle);
        if (std::abs(bearing) < front_half && r < m.min_front)
            m.min_front = r;
        if (r < slow_dist) {
            double w = 1.0 - (r / slow_dist);
            if (bearing > 0 && bearing < side_half) { left_acc += w; left_w += 1.0; }
            else if (bearing < 0 && bearing > -side_half) { right_acc += w; right_w += 1.0; }
        }
        angle += scan.angle_increment;
    }
    m.left_push = (left_w > 0) ? left_acc / left_w : 0.0;
    m.right_push = (right_w > 0) ? right_acc / right_w : 0.0;
    return m;
}

// ─── Coarse Global A* ────────────────────────────────────────────────

struct GlobalPlanResult {
    std::vector<std::pair<double,double>> waypoints;
    bool valid = false;
};

static GlobalPlanResult global_astar(
    const nav_msgs::msg::OccupancyGrid &map,
    double robot_x, double robot_y,
    double goal_x, double goal_y,
    int downsample, double inflation_m,
    int occupied_thresh, double waypoint_spacing_m)
{
    GlobalPlanResult result;
    int map_w = static_cast<int>(map.info.width);
    int map_h = static_cast<int>(map.info.height);
    double map_res = map.info.resolution;
    double map_ox = map.info.origin.position.x;
    double map_oy = map.info.origin.position.y;
    if (map_w < 2 || map_h < 2) return result;

    int cw = (map_w + downsample - 1) / downsample;
    int ch = (map_h + downsample - 1) / downsample;
    double cres = map_res * downsample;

    std::vector<uint8_t> cgrid(cw * ch, 0);
    for (int cy = 0; cy < ch; cy++) {
        for (int cx = 0; cx < cw; cx++) {
            bool occ = false;
            for (int dy = 0; dy < downsample && !occ; dy++) {
                for (int dx = 0; dx < downsample && !occ; dx++) {
                    int mx = cx * downsample + dx;
                    int my = cy * downsample + dy;
                    if (mx < map_w && my < map_h) {
                        int8_t val = map.data[my * map_w + mx];
                        if (val >= occupied_thresh) occ = true;
                    }
                }
            }
            if (occ) cgrid[cy * cw + cx] = 1;
        }
    }

    int inflate_cells = std::max(0, static_cast<int>(std::ceil(inflation_m / cres)));
    if (inflate_cells > 0) {
        std::vector<std::pair<int,int>> obs;
        for (int cy = 0; cy < ch; cy++)
            for (int cx = 0; cx < cw; cx++)
                if (cgrid[cy * cw + cx]) obs.emplace_back(cx, cy);
        int isq = inflate_cells * inflate_cells;
        for (auto [ox, oy] : obs) {
            for (int ddy = -inflate_cells; ddy <= inflate_cells; ddy++) {
                for (int ddx = -inflate_cells; ddx <= inflate_cells; ddx++) {
                    if (ddx*ddx + ddy*ddy > isq) continue;
                    int nx = ox+ddx, ny = oy+ddy;
                    if (nx >= 0 && nx < cw && ny >= 0 && ny < ch)
                        cgrid[ny * cw + nx] = 1;
                }
            }
        }
    }

    // World -> coarse cell
    auto to_coarse = [&](double wx, double wy, int &cx, int &cy) {
        cx = static_cast<int>((wx - map_ox) / cres);
        cy = static_cast<int>((wy - map_oy) / cres);
    };
    auto cell_to_world = [&](int cx, int cy, double &wx, double &wy) {
        wx = map_ox + (cx + 0.5) * cres;
        wy = map_oy + (cy + 0.5) * cres;
    };

    int sx, sy, gx, gy;
    to_coarse(robot_x, robot_y, sx, sy);
    to_coarse(goal_x, goal_y, gx, gy);
    sx = std::clamp(sx, 0, cw-1); sy = std::clamp(sy, 0, ch-1);
    gx = std::clamp(gx, 0, cw-1); gy = std::clamp(gy, 0, ch-1);

    // Clear start cell
    cgrid[sy * cw + sx] = 0;

    // A*
    struct ANode { int x, y; float g, f; int px, py; };
    std::vector<float> best(cw * ch, std::numeric_limits<float>::infinity());
    std::vector<int> prev_x(cw * ch, -1), prev_y(cw * ch, -1);
    auto cmp = [](const ANode &a, const ANode &b) { return a.f > b.f; };
    std::priority_queue<ANode, std::vector<ANode>, decltype(cmp)> pq(cmp);

    best[sy * cw + sx] = 0;
    pq.push({sx, sy, 0.0f,
             static_cast<float>(std::hypot(gx-sx, gy-sy)), -1, -1});

    static const int adx[] = {-1, 0, 1, -1, 1, -1, 0, 1};
    static const int ady[] = {-1, -1, -1, 0, 0, 1, 1, 1};
    static const float acost[] = {1.414f, 1.0f, 1.414f, 1.0f, 1.0f, 1.414f, 1.0f, 1.414f};

    bool found = false;
    while (!pq.empty()) {
        auto cur = pq.top(); pq.pop();
        if (cur.g > best[cur.y * cw + cur.x] + 1e-3f) continue;
        if (cur.x == gx && cur.y == gy) { found = true; break; }
        for (int i = 0; i < 8; i++) {
            int nx = cur.x + adx[i], ny = cur.y + ady[i];
            if (nx < 0 || nx >= cw || ny < 0 || ny >= ch) continue;
            if (cgrid[ny * cw + nx]) continue;
            float ng = cur.g + acost[i];
            if (ng < best[ny * cw + nx]) {
                best[ny * cw + nx] = ng;
                prev_x[ny * cw + nx] = cur.x;
                prev_y[ny * cw + nx] = cur.y;
                float h = static_cast<float>(std::hypot(gx-nx, gy-ny));
                pq.push({nx, ny, ng, ng + h, cur.x, cur.y});
            }
        }
    }
    if (!found) return result;

    std::vector<std::pair<int,int>> path_cells;
    int cx = gx, cy = gy;
    while (cx >= 0 && cy >= 0) {
        path_cells.emplace_back(cx, cy);
        int px = prev_x[cy * cw + cx], py = prev_y[cy * cw + cx];
        cx = px; cy = py;
    }
    std::reverse(path_cells.begin(), path_cells.end());

    result.waypoints.clear();
    double last_wx = robot_x, last_wy = robot_y;
    for (auto [pcx, pcy] : path_cells) {
        double wx, wy;
        cell_to_world(pcx, pcy, wx, wy);
        if (std::hypot(wx - last_wx, wy - last_wy) >= waypoint_spacing_m) {
            result.waypoints.emplace_back(wx, wy);
            last_wx = wx; last_wy = wy;
        }
    }
    double fwx, fwy;
    cell_to_world(gx, gy, fwx, fwy);
    if (result.waypoints.empty() ||
        std::hypot(result.waypoints.back().first - fwx,
                   result.waypoints.back().second - fwy) > 0.1) {
        result.waypoints.emplace_back(fwx, fwy);
    }
    result.valid = true;
    return result;
}

// ─── MPPI Core ───────────────────────────────────────────────────────

struct MPPIConfig {
    int num_rollouts     = 512;
    int horizon          = 30;
    double dt            = 0.1;     // seconds per step (3 s lookahead)
    double temperature   = 0.05;    // lower = greedier
    double linear_noise  = 0.20;    // std dev on v
    double angular_noise = 0.40;    // std dev on ω
    double max_v         = 0.70;
    double max_w         = 0.85;

    // Cost weights
    double w_goal        = 5.0;     // terminal distance to goal
    double w_heading     = 2.0;     // alignment toward goal
    double w_obstacle    = 25.0;    // proximity to obstacles
    double w_speed       = -0.5;    // negative = reward forward motion
    double w_angular     = 0.8;     // penalise spinning
    double w_angular_acc = 0.3;     // penalise angular jerk

    // Obstacle cost shaping
    double obstacle_lethal_m = 0.15;  // collision → huge penalty
    double obstacle_influence_m = 1.0; // soft penalty range

    // Path following (when global plan available)
    double w_path        = 3.0;     // cross-track error to global plan
};

struct MPPIState {
    double x, y, theta;
};

class MPPIPlanner {
public:
    explicit MPPIPlanner(const MPPIConfig &cfg)
        : cfg_(cfg),
          rng_(std::random_device{}()),
          mean_v_(cfg.horizon, 0.0),
          mean_w_(cfg.horizon, 0.0) {}

    // Returns (linear_vel, angular_vel) in robot frame.
    std::pair<double, double> compute(
        MPPIState robot,
        double goal_x, double goal_y,
        const LocalGrid &grid,
        const std::vector<int> &dist_field,
        const std::vector<std::pair<double,double>> &path_ref)
    {
        int K = cfg_.num_rollouts;
        int H = cfg_.horizon;

        // Per-rollout noise and cost
        std::vector<std::vector<double>> noise_v(K, std::vector<double>(H));
        std::vector<std::vector<double>> noise_w(K, std::vector<double>(H));
        std::vector<double> costs(K, 0.0);

        std::normal_distribution<double> nv(0.0, cfg_.linear_noise);
        std::normal_distribution<double> nw(0.0, cfg_.angular_noise);

        // Generate noise
        for (int k = 0; k < K; k++) {
            for (int t = 0; t < H; t++) {
                noise_v[k][t] = nv(rng_);
                noise_w[k][t] = nw(rng_);
            }
        }

        double lethal_cells = cfg_.obstacle_lethal_m / grid.resolution;

        // Rollout each trajectory
        for (int k = 0; k < K; k++) {
            MPPIState s = robot;
            double prev_w = mean_w_[0];
            bool collided = false;

            for (int t = 0; t < H; t++) {
                double v = std::clamp(mean_v_[t] + noise_v[k][t], 0.0, cfg_.max_v);
                double w = std::clamp(mean_w_[t] + noise_w[k][t], -cfg_.max_w, cfg_.max_w);

                // Propagate unicycle dynamics
                s.theta = wrap_angle(s.theta + w * cfg_.dt);
                s.x += v * std::cos(s.theta) * cfg_.dt;
                s.y += v * std::sin(s.theta) * cfg_.dt;

                // Obstacle cost via distance field
                int gx, gy;
                if (grid.local_to_cell(
                        local_x(s, robot, grid),
                        local_y(s, robot, grid), gx, gy)) {
                    int d = dist_field[gy * grid.n + gx];
                    double dm = d * grid.resolution;
                    if (d == 0 || dm < lethal_cells * grid.resolution) {
                        costs[k] += 1e4;
                        collided = true;
                        break;
                    } else if (dm < cfg_.obstacle_influence_m) {
                        double ratio = 1.0 - (dm / cfg_.obstacle_influence_m);
                        costs[k] += cfg_.w_obstacle * ratio * ratio;
                    }
                } else {
                    // Out of local grid — penalise
                    costs[k] += cfg_.w_obstacle * 0.5;
                }

                // Speed reward
                costs[k] += cfg_.w_speed * v;

                // Angular rate penalty
                costs[k] += cfg_.w_angular * w * w;

                // Angular acceleration penalty (smoothness)
                double dw = w - prev_w;
                costs[k] += cfg_.w_angular_acc * dw * dw;
                prev_w = w;

                // Heading toward goal
                double desired_heading = std::atan2(goal_y - s.y, goal_x - s.x);
                double herr = std::abs(wrap_angle(s.theta - desired_heading));
                costs[k] += cfg_.w_heading * herr;

                // Path following: cross-track error to nearest global waypoint
                if (!path_ref.empty()) {
                    double min_d2 = std::numeric_limits<double>::max();
                    for (auto &[px, py] : path_ref) {
                        double ddx = s.x - px, ddy = s.y - py;
                        min_d2 = std::min(min_d2, ddx * ddx + ddy * ddy);
                    }
                    costs[k] += cfg_.w_path * std::sqrt(min_d2);
                }
            }

            if (!collided) {
                // Terminal cost: distance to goal
                double goal_dist = std::hypot(s.x - goal_x, s.y - goal_y);
                costs[k] += cfg_.w_goal * goal_dist;
            }
        }

        // MPPI weighting: w_k = exp(-(cost_k - min_cost) / lambda)
        double min_cost = *std::min_element(costs.begin(), costs.end());
        std::vector<double> weights(K);
        double weight_sum = 0.0;
        for (int k = 0; k < K; k++) {
            weights[k] = std::exp(-(costs[k] - min_cost) / cfg_.temperature);
            weight_sum += weights[k];
        }
        if (weight_sum < 1e-30) weight_sum = 1e-30;
        for (int k = 0; k < K; k++) weights[k] /= weight_sum;

        // Weighted average control update
        std::vector<double> new_v(H, 0.0), new_w(H, 0.0);
        for (int k = 0; k < K; k++) {
            for (int t = 0; t < H; t++) {
                double v_k = std::clamp(mean_v_[t] + noise_v[k][t], 0.0, cfg_.max_v);
                double w_k = std::clamp(mean_w_[t] + noise_w[k][t], -cfg_.max_w, cfg_.max_w);
                new_v[t] += weights[k] * v_k;
                new_w[t] += weights[k] * w_k;
            }
        }
        mean_v_ = std::move(new_v);
        mean_w_ = std::move(new_w);

        // Extract command
        double cmd_v = mean_v_[0];
        double cmd_w = mean_w_[0];

        // Warm-start: shift sequence, pad last element
        for (int t = 0; t < H - 1; t++) {
            mean_v_[t] = mean_v_[t + 1];
            mean_w_[t] = mean_w_[t + 1];
        }
        mean_v_[H - 1] = 0.0;
        mean_w_[H - 1] = 0.0;

        return {cmd_v, cmd_w};
    }

    // Generate the best (mean) trajectory for visualisation.
    std::vector<std::pair<double,double>> predicted_trajectory(
        MPPIState robot) const
    {
        std::vector<std::pair<double,double>> traj;
        MPPIState s = robot;
        for (int t = 0; t < static_cast<int>(mean_v_.size()); t++) {
            s.theta = wrap_angle(s.theta + mean_w_[t] * cfg_.dt);
            s.x += mean_v_[t] * std::cos(s.theta) * cfg_.dt;
            s.y += mean_v_[t] * std::sin(s.theta) * cfg_.dt;
            traj.emplace_back(s.x, s.y);
        }
        return traj;
    }

    void reset() {
        std::fill(mean_v_.begin(), mean_v_.end(), 0.0);
        std::fill(mean_w_.begin(), mean_w_.end(), 0.0);
    }

private:
    // Convert world-frame state to robot-local for grid lookup.
    static double local_x(const MPPIState &s, const MPPIState &robot,
                           const LocalGrid & /*grid*/) {
        double dx = s.x - robot.x, dy = s.y - robot.y;
        return  std::cos(robot.theta) * dx + std::sin(robot.theta) * dy;
    }
    static double local_y(const MPPIState &s, const MPPIState &robot,
                           const LocalGrid & /*grid*/) {
        double dx = s.x - robot.x, dy = s.y - robot.y;
        return -std::sin(robot.theta) * dx + std::cos(robot.theta) * dy;
    }

    MPPIConfig cfg_;
    std::mt19937 rng_;
    std::vector<double> mean_v_, mean_w_;
};

// ═══════════════════════════════════════════════════════════════════════
//  ROS2 Node
// ═══════════════════════════════════════════════════════════════════════

class MppiNavNode : public rclcpp::Node {
public:
    MppiNavNode() : Node("mppi_nav") {
        // ── Shared parameters (same names as reactive_nav) ──
        declare_parameter("max_linear_speed", 0.70);
        declare_parameter("max_angular_speed", 0.85);
        declare_parameter("control_rate", 12.0);
        declare_parameter("startup_delay", 0.0);
        declare_parameter("goal_tolerance", 0.20);
        declare_parameter("goal_reached_replan_cooldown_sec", 1.0);
        declare_parameter("obstacle_slow_dist", 0.75);
        declare_parameter("obstacle_stop_dist", 0.35);
        declare_parameter("front_half_angle_deg", 35.0);
        declare_parameter("side_check_angle_deg", 60.0);
        declare_parameter("turn_in_place_on_block", true);
        declare_parameter("map_topic", std::string(""));
        declare_parameter("map_frame", std::string("map"));
        declare_parameter("map_occupied_thresh", 50);
        declare_parameter("frontier_replan_topic", std::string("/frontier_replan"));
        declare_parameter("stop_topic", std::string("/stop"));

        // ── Local grid parameters ──
        declare_parameter("grid_radius", 4.0);
        declare_parameter("grid_resolution", 0.10);
        declare_parameter("inflation_radius", 0.15);
        declare_parameter("start_clearance_radius", 0.12);
        declare_parameter("unknown_is_obstacle", true);

        // ── MPPI parameters ──
        declare_parameter("mppi_num_rollouts", 512);
        declare_parameter("mppi_horizon", 30);
        declare_parameter("mppi_dt", 0.1);
        declare_parameter("mppi_temperature", 0.05);
        declare_parameter("mppi_linear_noise", 0.20);
        declare_parameter("mppi_angular_noise", 0.40);
        declare_parameter("mppi_w_goal", 5.0);
        declare_parameter("mppi_w_heading", 2.0);
        declare_parameter("mppi_w_obstacle", 25.0);
        declare_parameter("mppi_w_speed", -0.5);
        declare_parameter("mppi_w_angular", 0.8);
        declare_parameter("mppi_w_angular_acc", 0.3);
        declare_parameter("mppi_obstacle_lethal_m", 0.15);
        declare_parameter("mppi_obstacle_influence_m", 1.0);
        declare_parameter("mppi_w_path", 3.0);

        // ── Global A* parameters ──
        declare_parameter("global_downsample", 3);
        declare_parameter("global_inflation_m", 0.15);
        declare_parameter("global_replan_sec", 3.0);
        declare_parameter("global_waypoint_spacing_m", 1.5);
        declare_parameter("global_reach_fraction", 0.85);

        load_params();

        // Build MPPI planner
        MPPIConfig mcfg;
        mcfg.num_rollouts        = mppi_num_rollouts_;
        mcfg.horizon             = mppi_horizon_;
        mcfg.dt                  = mppi_dt_;
        mcfg.temperature         = mppi_temperature_;
        mcfg.linear_noise        = mppi_linear_noise_;
        mcfg.angular_noise       = mppi_angular_noise_;
        mcfg.max_v               = max_linear_speed_;
        mcfg.max_w               = max_angular_speed_;
        mcfg.w_goal              = mppi_w_goal_;
        mcfg.w_heading           = mppi_w_heading_;
        mcfg.w_obstacle          = mppi_w_obstacle_;
        mcfg.w_speed             = mppi_w_speed_;
        mcfg.w_angular           = mppi_w_angular_;
        mcfg.w_angular_acc       = mppi_w_angular_acc_;
        mcfg.obstacle_lethal_m   = mppi_obstacle_lethal_m_;
        mcfg.obstacle_influence_m = mppi_obstacle_influence_m_;
        mcfg.w_path              = mppi_w_path_;
        mppi_ = std::make_unique<MPPIPlanner>(mcfg);

        // Allocate local grid
        int cells = std::max(31, static_cast<int>(
            std::ceil(2.0 * grid_radius_ / grid_resolution_)) + 1);
        cells = cells | 1;
        grid_.n = cells;
        grid_.resolution = grid_resolution_;
        grid_.data.resize(cells * cells, 0);

        // TF2
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        map_frame_ = get_parameter("map_frame").as_string();

        // ── Publishers (absolute names, same as reactive_nav) ──
        cmd_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel_stamped", 10);
        status_pub_ = create_publisher<std_msgs::msg::String>("/nav_status", 10);
        path_pub_ = create_publisher<nav_msgs::msg::Path>("/planned_path", 10);
        global_path_pub_ = create_publisher<nav_msgs::msg::Path>("/global_planned_path", 10);
        traj_pub_ = create_publisher<nav_msgs::msg::Path>("/robot_trajectory", 10);
        goal_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("/final_goal_marker", 10);
        pose_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("/robot_pose_marker", 10);

        auto frontier_topic = get_parameter("frontier_replan_topic").as_string();
        auto stop_topic = get_parameter("stop_topic").as_string();
        replan_pub_ = create_publisher<std_msgs::msg::Empty>(frontier_topic, 10);

        // ── Subscribers ──
        auto sensor_qos = rclcpp::SensorDataQoS();
        auto best_effort_qos = rclcpp::QoS(1)
            .reliability(rclcpp::ReliabilityPolicy::BestEffort)
            .durability(rclcpp::DurabilityPolicy::Volatile);

        wp_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
            "/way_point", 10,
            [this](geometry_msgs::msg::PointStamped::SharedPtr msg) {
                double new_gx = msg->point.x, new_gy = msg->point.y;
                if (std::hypot(new_gx - goal_x_, new_gy - goal_y_) > 0.5) {
                    global_waypoints_.clear();
                    last_global_plan_time_ = -1;
                    mppi_->reset();
                }
                goal_x_ = new_gx; goal_y_ = new_gy;
                has_goal_ = true;
            });

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom/ground_truth", sensor_qos,
            [this](nav_msgs::msg::Odometry::SharedPtr msg) {
                robot_x_ = msg->pose.pose.position.x;
                robot_y_ = msg->pose.pose.position.y;
                auto &q = msg->pose.pose.orientation;
                robot_yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                         1.0 - 2.0 * (q.y * q.y + q.z * q.z));
            });

        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", best_effort_qos,
            [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {
                last_scan_ = msg;
            });

        stop_sub_ = create_subscription<std_msgs::msg::Int8>(
            stop_topic, 10,
            [this](std_msgs::msg::Int8::SharedPtr msg) {
                external_stop_ = msg->data;
            });

        auto map_topic_param = get_parameter("map_topic").as_string();
        if (map_topic_param.empty()) {
            map_topic_param = "/" + std::string(get_namespace()) + "/map";
            if (map_topic_param.size() > 1 && map_topic_param[0] == '/' && map_topic_param[1] == '/')
                map_topic_param = map_topic_param.substr(1);
        }
        auto map_qos = rclcpp::QoS(1)
            .reliability(rclcpp::ReliabilityPolicy::Reliable)
            .durability(rclcpp::DurabilityPolicy::TransientLocal);
        map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
            map_topic_param, map_qos,
            [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
                last_map_ = msg;
            });
        RCLCPP_INFO(get_logger(), "Subscribing to map: %s", map_topic_param.c_str());

        // ── Control timer ──
        double period = 1.0 / control_rate_;
        timer_ = create_wall_timer(
            std::chrono::duration<double>(period),
            std::bind(&MppiNavNode::tick, this));

        RCLCPP_INFO(get_logger(),
            "MPPI nav started: rate=%.0fHz rollouts=%d horizon=%d dt=%.2fs",
            control_rate_, mppi_num_rollouts_, mppi_horizon_, mppi_dt_);
    }

private:
    void load_params() {
        max_linear_speed_    = get_parameter("max_linear_speed").as_double();
        max_angular_speed_   = get_parameter("max_angular_speed").as_double();
        control_rate_        = get_parameter("control_rate").as_double();
        startup_delay_       = get_parameter("startup_delay").as_double();
        goal_tolerance_      = get_parameter("goal_tolerance").as_double();
        goal_reached_cooldown_ = get_parameter("goal_reached_replan_cooldown_sec").as_double();
        obstacle_slow_dist_  = get_parameter("obstacle_slow_dist").as_double();
        obstacle_stop_dist_  = get_parameter("obstacle_stop_dist").as_double();
        front_half_          = get_parameter("front_half_angle_deg").as_double() * M_PI / 180.0;
        side_half_           = get_parameter("side_check_angle_deg").as_double() * M_PI / 180.0;
        turn_in_place_       = get_parameter("turn_in_place_on_block").as_bool();
        map_occupied_thresh_ = static_cast<int>(get_parameter("map_occupied_thresh").as_int());

        grid_radius_         = get_parameter("grid_radius").as_double();
        grid_resolution_     = get_parameter("grid_resolution").as_double();
        inflation_radius_    = get_parameter("inflation_radius").as_double();
        start_clearance_     = get_parameter("start_clearance_radius").as_double();
        unknown_is_obstacle_ = get_parameter("unknown_is_obstacle").as_bool();

        mppi_num_rollouts_      = static_cast<int>(get_parameter("mppi_num_rollouts").as_int());
        mppi_horizon_           = static_cast<int>(get_parameter("mppi_horizon").as_int());
        mppi_dt_                = get_parameter("mppi_dt").as_double();
        mppi_temperature_       = get_parameter("mppi_temperature").as_double();
        mppi_linear_noise_      = get_parameter("mppi_linear_noise").as_double();
        mppi_angular_noise_     = get_parameter("mppi_angular_noise").as_double();
        mppi_w_goal_            = get_parameter("mppi_w_goal").as_double();
        mppi_w_heading_         = get_parameter("mppi_w_heading").as_double();
        mppi_w_obstacle_        = get_parameter("mppi_w_obstacle").as_double();
        mppi_w_speed_           = get_parameter("mppi_w_speed").as_double();
        mppi_w_angular_         = get_parameter("mppi_w_angular").as_double();
        mppi_w_angular_acc_     = get_parameter("mppi_w_angular_acc").as_double();
        mppi_obstacle_lethal_m_ = get_parameter("mppi_obstacle_lethal_m").as_double();
        mppi_obstacle_influence_m_ = get_parameter("mppi_obstacle_influence_m").as_double();
        mppi_w_path_            = get_parameter("mppi_w_path").as_double();

        global_downsample_      = static_cast<int>(get_parameter("global_downsample").as_int());
        global_inflation_m_     = get_parameter("global_inflation_m").as_double();
        global_replan_sec_      = get_parameter("global_replan_sec").as_double();
        global_waypoint_spacing_m_ = get_parameter("global_waypoint_spacing_m").as_double();
        global_reach_fraction_  = get_parameter("global_reach_fraction").as_double();
    }

    // ── Main tick ────────────────────────────────────────────────────

    void tick() {
        auto now = this->now();
        double now_sec = now.seconds();

        if (start_time_ < 0) start_time_ = now_sec;
        if ((now_sec - start_time_) < startup_delay_) {
            publish_cmd(0.0, 0.0);
            return;
        }

        if (!has_goal_ || !last_scan_) {
            publish_cmd(0.0, 0.0);
            return;
        }

        // Look up robot pose in map frame via TF2
        try {
            auto tf = tf_buffer_->lookupTransform(
                map_frame_, "base_link", tf2::TimePointZero,
                tf2::durationFromSec(0.1));
            map_robot_x_ = tf.transform.translation.x;
            map_robot_y_ = tf.transform.translation.y;
            tf2::Quaternion q;
            tf2::fromMsg(tf.transform.rotation, q);
            double roll, pitch, yaw;
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
            map_robot_yaw_ = yaw;
            has_map_tf_ = true;
        } catch (tf2::TransformException &ex) {
            if (!has_map_tf_) {
                publish_cmd(0.0, 0.0);
                return;
            }
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "No TF %s→base_link: %s (using last known)", map_frame_.c_str(), ex.what());
        }

        double dist_to_goal = std::hypot(goal_x_ - map_robot_x_, goal_y_ - map_robot_y_);

        // ── Goal reached ──
        if (dist_to_goal < goal_tolerance_) {
            publish_cmd(0.0, 0.0);
            if (last_replan_time_ < 0 || (now_sec - last_replan_time_) >= goal_reached_cooldown_) {
                replan_pub_->publish(std_msgs::msg::Empty());
                last_replan_time_ = now_sec;
            }
            return;
        }

        // ── Scan safety metrics ──
        auto sm = analyze_scan(*last_scan_, front_half_, side_half_, obstacle_slow_dist_);

        // ── Global A* for long-range routing ──
        bool global_replan_due = last_global_plan_time_ < 0
            || (now_sec - last_global_plan_time_) >= global_replan_sec_
            || global_waypoints_.empty();

        if (global_replan_due && last_map_ && dist_to_goal > grid_radius_ * 0.8) {
            auto gplan = global_astar(
                *last_map_, map_robot_x_, map_robot_y_, goal_x_, goal_y_,
                global_downsample_, global_inflation_m_,
                map_occupied_thresh_, global_waypoint_spacing_m_);
            if (gplan.valid && !gplan.waypoints.empty()) {
                global_waypoints_ = std::move(gplan.waypoints);
                last_global_plan_time_ = now_sec;
            } else {
                last_global_plan_time_ = now_sec;
            }
            publish_global_path();
        }

        // ── Pick local goal from global plan ──
        double local_goal_x = goal_x_, local_goal_y = goal_y_;
        if (!global_waypoints_.empty() && dist_to_goal > grid_radius_ * 0.8) {
            // Prune passed waypoints
            while (global_waypoints_.size() > 1) {
                double d = std::hypot(global_waypoints_.front().first - map_robot_x_,
                                       global_waypoints_.front().second - map_robot_y_);
                if (d < goal_tolerance_ * 2.0)
                    global_waypoints_.erase(global_waypoints_.begin());
                else
                    break;
            }
            // Furthest reachable within local grid
            double reach = grid_radius_ * global_reach_fraction_;
            for (size_t i = 0; i < global_waypoints_.size(); i++) {
                double d = std::hypot(global_waypoints_[i].first - map_robot_x_,
                                       global_waypoints_[i].second - map_robot_y_);
                if (d <= reach) {
                    local_goal_x = global_waypoints_[i].first;
                    local_goal_y = global_waypoints_[i].second;
                } else {
                    break;
                }
            }
        }

        // ── Build local grid + distance field ──
        build_local_grid(
            last_map_ ? last_map_.get() : nullptr,
            grid_, map_robot_x_, map_robot_y_, map_robot_yaw_,
            inflation_radius_, start_clearance_,
            unknown_is_obstacle_, map_occupied_thresh_);

        build_distance_field(grid_, dist_field_);

        // ── Run MPPI ──
        MPPIState robot_state{map_robot_x_, map_robot_y_, map_robot_yaw_};
        auto [lin, ang] = mppi_->compute(
            robot_state, local_goal_x, local_goal_y,
            grid_, dist_field_, global_waypoints_);

        // ── Reactive safety layer (scan-based) ──
        // Slow down near obstacles
        if (sm.min_front < obstacle_slow_dist_) {
            double rng_val = obstacle_slow_dist_ - obstacle_stop_dist_;
            if (rng_val > 1e-6)
                lin *= std::max(0.0, (sm.min_front - obstacle_stop_dist_) / rng_val);
        }

        // Emergency stop
        bool blocked_front = sm.min_front < obstacle_stop_dist_;
        if (blocked_front || external_stop_ != 0) {
            lin = 0.0;
            if (blocked_front && turn_in_place_ && external_stop_ == 0) {
                if (std::abs(ang) < 0.1)
                    ang = max_angular_speed_ * 0.5 * ((sm.right_push > sm.left_push) ? 1.0 : -1.0);
            } else {
                ang = 0.0;
            }
        }

        publish_cmd(lin, ang);

        if (blocked_front) {
            replan_pub_->publish(std_msgs::msg::Empty());
        }

        // ── Visualisation ──
        auto pred = mppi_->predicted_trajectory(robot_state);
        publish_predicted_path(pred);

        trajectory_.emplace_back(map_robot_x_, map_robot_y_);
        if (trajectory_.size() > 2000) trajectory_.erase(trajectory_.begin());
        publish_trajectory();
        publish_goal_marker();
        publish_pose_marker();
        publish_global_path();
    }

    // ── Publishing helpers ───────────────────────────────────────────

    void publish_cmd(double lin, double ang) {
        geometry_msgs::msg::TwistStamped msg;
        msg.header.stamp = this->now();
        msg.twist.linear.x = lin;
        msg.twist.angular.z = ang;
        cmd_pub_->publish(msg);
    }

    void publish_predicted_path(
        const std::vector<std::pair<double,double>> &pts) {
        nav_msgs::msg::Path msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = map_frame_;
        for (auto &[wx, wy] : pts) {
            geometry_msgs::msg::PoseStamped ps;
            ps.header = msg.header;
            ps.pose.position.x = wx;
            ps.pose.position.y = wy;
            msg.poses.push_back(ps);
        }
        path_pub_->publish(msg);
    }

    void publish_trajectory() {
        nav_msgs::msg::Path msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = map_frame_;
        for (auto &[wx, wy] : trajectory_) {
            geometry_msgs::msg::PoseStamped ps;
            ps.header = msg.header;
            ps.pose.position.x = wx;
            ps.pose.position.y = wy;
            msg.poses.push_back(ps);
        }
        traj_pub_->publish(msg);
    }

    void publish_goal_marker() {
        visualization_msgs::msg::Marker m;
        m.header.stamp = this->now();
        m.header.frame_id = map_frame_;
        m.ns = "mppi_nav_goal"; m.id = 0;
        m.type = visualization_msgs::msg::Marker::SPHERE;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.position.x = goal_x_; m.pose.position.y = goal_y_; m.pose.position.z = 0.3;
        m.scale.x = m.scale.y = m.scale.z = 0.25;
        m.color.g = 1.0; m.color.b = 0.5; m.color.a = 0.9;
        goal_marker_pub_->publish(m);
    }

    void publish_pose_marker() {
        visualization_msgs::msg::Marker m;
        m.header.stamp = this->now();
        m.header.frame_id = map_frame_;
        m.ns = "mppi_nav_pose"; m.id = 0;
        m.type = visualization_msgs::msg::Marker::ARROW;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.position.x = map_robot_x_; m.pose.position.y = map_robot_y_; m.pose.position.z = 0.15;
        m.pose.orientation.z = std::sin(map_robot_yaw_ / 2.0);
        m.pose.orientation.w = std::cos(map_robot_yaw_ / 2.0);
        m.scale.x = 0.3; m.scale.y = 0.08; m.scale.z = 0.08;
        m.color.r = 0.2; m.color.g = 0.8; m.color.b = 1.0; m.color.a = 0.9;
        pose_marker_pub_->publish(m);
    }

    void publish_global_path() {
        nav_msgs::msg::Path msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = map_frame_;
        for (auto &[wx, wy] : global_waypoints_) {
            geometry_msgs::msg::PoseStamped ps;
            ps.header = msg.header;
            ps.pose.position.x = wx;
            ps.pose.position.y = wy;
            msg.poses.push_back(ps);
        }
        global_path_pub_->publish(msg);
    }

    // ── State ────────────────────────────────────────────────────────

    // Shared parameters
    double max_linear_speed_, max_angular_speed_, control_rate_, startup_delay_;
    double goal_tolerance_, goal_reached_cooldown_;
    double obstacle_slow_dist_, obstacle_stop_dist_;
    double front_half_, side_half_;
    bool turn_in_place_;
    int map_occupied_thresh_;

    // Local grid
    double grid_radius_, grid_resolution_, inflation_radius_, start_clearance_;
    bool unknown_is_obstacle_;

    // MPPI params (stored for planner construction)
    int mppi_num_rollouts_, mppi_horizon_;
    double mppi_dt_, mppi_temperature_;
    double mppi_linear_noise_, mppi_angular_noise_;
    double mppi_w_goal_, mppi_w_heading_, mppi_w_obstacle_;
    double mppi_w_speed_, mppi_w_angular_, mppi_w_angular_acc_;
    double mppi_obstacle_lethal_m_, mppi_obstacle_influence_m_;
    double mppi_w_path_;

    // Global A*
    int global_downsample_;
    double global_inflation_m_, global_replan_sec_, global_waypoint_spacing_m_;
    double global_reach_fraction_;

    // Robot state
    double robot_x_ = 0, robot_y_ = 0, robot_yaw_ = 0;
    double map_robot_x_ = 0, map_robot_y_ = 0, map_robot_yaw_ = 0;
    bool has_map_tf_ = false;
    double goal_x_ = 0, goal_y_ = 0;
    bool has_goal_ = false;
    std::string map_frame_ = "map";
    int external_stop_ = 0;
    double start_time_ = -1;
    double last_replan_time_ = -1;

    // Scan & map
    sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
    nav_msgs::msg::OccupancyGrid::SharedPtr last_map_;

    // Planning state
    LocalGrid grid_;
    std::vector<int> dist_field_;
    std::unique_ptr<MPPIPlanner> mppi_;
    std::vector<std::pair<double,double>> global_waypoints_;
    double last_global_plan_time_ = -1;
    std::vector<std::pair<double,double>> trajectory_;

    // ROS interfaces
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr traj_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pose_marker_pub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr replan_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr wp_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr stop_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

// ═══════════════════════════════════════════════════════════════════════

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MppiNavNode>());
    rclcpp::shutdown();
    return 0;
}
