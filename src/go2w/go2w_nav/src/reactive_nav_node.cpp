/*
 * reactive_nav_node.cpp — RRT*-based reactive navigation (C++ ROS2 node).
 *
 * Subscribes to laser scan, odometry, and waypoint goals.
 * Builds a local occupancy grid from the scan, runs RRT* to find a path,
 * then follows it with a pure-pursuit + obstacle-avoidance controller.
 *
 * Replaces the Python reactive_nav.py for better real-time performance.
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

    // Local (metres, robot-centred) to grid cell.
    bool local_to_cell(double lx, double ly, int &gx, int &gy) const {
        int c = center();
        gx = static_cast<int>(std::round(lx / resolution)) + c;
        gy = static_cast<int>(std::round(ly / resolution)) + c;
        return in_bounds(gx, gy);
    }

    // Grid cell to local (metres).
    void cell_to_local(int gx, int gy, double &lx, double &ly) const {
        int c = center();
        lx = (gx - c) * resolution;
        ly = (gy - c) * resolution;
    }
};

// Bresenham line between two grid cells.
static void bresenham_line(int x0, int y0, int x1, int y1,
                           std::vector<std::pair<int,int>> &out) {
    out.clear();
    int dx = std::abs(x1 - x0), dy = std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1, sy = (y0 < y1) ? 1 : -1;
    int x = x0, y = y0;
    if (dx >= dy) {
        int err = dx / 2;
        while (x != x1) {
            out.emplace_back(x, y);
            err -= dy;
            if (err < 0) { y += sy; err += dx; }
            x += sx;
        }
    } else {
        int err = dy / 2;
        while (y != y1) {
            out.emplace_back(x, y);
            err -= dx;
            if (err < 0) { x += sx; err += dy; }
            y += sy;
        }
    }
    out.emplace_back(x1, y1);
}

// Build local grid from the global occupancy map.
// The scan is NOT used here — Cartographer (or whichever map source) already
// incorporates laser data with proper pose correction.  The scan is only used
// by the reactive controller (analyze_scan) for real-time braking/avoidance.
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

    // Populate from global map
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

    // Collect obstacle cells for inflation
    std::vector<std::pair<int,int>> obstacles;
    for (int gy = 0; gy < n; gy++) {
        for (int gx = 0; gx < n; gx++) {
            if (grid.data[gy * n + gx] == 1)
                obstacles.emplace_back(gx, gy);
        }
    }

    // Inflate obstacles
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

// ─── RRT* Planner ─────────────────────────────────────────────────────

struct RRTNode {
    int gx, gy;
    int parent;   // -1 = root
    float cost;   // from root
};

static bool is_edge_free(const LocalGrid &grid, int x0, int y0, int x1, int y1) {
    int dx = std::abs(x1 - x0), dy = std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1, sy = (y0 < y1) ? 1 : -1;
    int x = x0, y = y0;
    if (dx >= dy) {
        int err = dx / 2;
        while (x != x1) {
            if (grid.blocked(x, y)) return false;
            err -= dy;
            if (err < 0) { y += sy; err += dx; }
            x += sx;
        }
    } else {
        int err = dy / 2;
        while (y != y1) {
            if (grid.blocked(x, y)) return false;
            err -= dx;
            if (err < 0) { x += sx; err += dy; }
            y += sy;
        }
    }
    return !grid.blocked(x1, y1);
}

struct RRTResult {
    std::vector<std::pair<int,int>> path_cells;
    float cost = std::numeric_limits<float>::infinity();
};

static RRTResult run_rrt_star(
    const LocalGrid &grid,
    int sx, int sy,
    int gx, int gy,
    int sample_count,
    int step_cells,
    int rewire_cells,
    double goal_bias,
    int goal_tol_cells,
    bool use_informed,
    std::mt19937 &rng
) {
    RRTResult result;
    if (grid.blocked(sx, sy)) return result;

    std::vector<RRTNode> nodes;
    nodes.reserve(sample_count + 1);
    nodes.push_back({sx, sy, -1, 0.0f});

    int goal_node_idx = -1;
    float best_cost = std::numeric_limits<float>::infinity();

    int goal_tol_sq = goal_tol_cells * goal_tol_cells;
    int rewire_sq = rewire_cells * rewire_cells;
    int n = grid.n;

    // Ellipse params for informed RRT*
    float c_min = std::sqrt(
        static_cast<float>((gx - sx) * (gx - sx) + (gy - sy) * (gy - sy)));
    float center_x = (sx + gx) * 0.5f;
    float center_y = (sy + gy) * 0.5f;
    float theta = std::atan2(static_cast<float>(gy - sy), static_cast<float>(gx - sx));
    float cos_t = std::cos(theta), sin_t = std::sin(theta);

    std::uniform_int_distribution<int> dist_grid(0, n - 1);
    std::uniform_real_distribution<float> dist_01(0.0f, 1.0f);
    std::uniform_real_distribution<float> dist_m1p1(-1.0f, 1.0f);

    for (int iter = 0; iter < sample_count; iter++) {
        int sample_x, sample_y;

        if (dist_01(rng) < goal_bias) {
            sample_x = gx;
            sample_y = gy;
        } else if (use_informed && best_cost < 1e20f) {
            // Informed RRT*: sample inside ellipse
            float a = best_cost * 0.5f;
            float b2 = (best_cost * best_cost - c_min * c_min) * 0.25f;
            float b = (b2 > 0.0f) ? std::sqrt(b2) : a * 0.1f;
            float ux, uy;
            do {
                ux = dist_m1p1(rng);
                uy = dist_m1p1(rng);
            } while (ux * ux + uy * uy > 1.0f);
            float ex = a * ux, ey = b * uy;
            float rx = cos_t * ex - sin_t * ey + center_x;
            float ry = sin_t * ex + cos_t * ey + center_y;
            sample_x = std::clamp(static_cast<int>(rx + 0.5f), 0, n - 1);
            sample_y = std::clamp(static_cast<int>(ry + 0.5f), 0, n - 1);
        } else {
            sample_x = dist_grid(rng);
            sample_y = dist_grid(rng);
        }

        // Nearest node
        int nearest_idx = 0;
        float nearest_d2 = std::numeric_limits<float>::infinity();
        for (int i = 0; i < static_cast<int>(nodes.size()); i++) {
            float dx = static_cast<float>(nodes[i].gx - sample_x);
            float dy = static_cast<float>(nodes[i].gy - sample_y);
            float d2 = dx * dx + dy * dy;
            if (d2 < nearest_d2) { nearest_d2 = d2; nearest_idx = i; }
        }

        const auto &nn = nodes[nearest_idx];
        float dx = static_cast<float>(sample_x - nn.gx);
        float dy = static_cast<float>(sample_y - nn.gy);
        float dist = std::sqrt(dx * dx + dy * dy);
        if (dist < 0.5f) continue;

        int new_x, new_y;
        if (dist > static_cast<float>(step_cells)) {
            float scale = static_cast<float>(step_cells) / dist;
            new_x = static_cast<int>(nn.gx + dx * scale + 0.5f);
            new_y = static_cast<int>(nn.gy + dy * scale + 0.5f);
        } else {
            new_x = sample_x;
            new_y = sample_y;
        }

        if (!grid.in_bounds(new_x, new_y) || grid.blocked(new_x, new_y)) continue;
        if (!is_edge_free(grid, nn.gx, nn.gy, new_x, new_y)) continue;

        // Best parent
        float edge_len = std::sqrt(
            static_cast<float>((new_x - nn.gx) * (new_x - nn.gx) +
                               (new_y - nn.gy) * (new_y - nn.gy)));
        float new_cost = nn.cost + edge_len;
        int best_parent = nearest_idx;

        for (int i = 0; i < static_cast<int>(nodes.size()); i++) {
            int ddx = nodes[i].gx - new_x, ddy = nodes[i].gy - new_y;
            int d2 = ddx * ddx + ddy * ddy;
            if (d2 > rewire_sq) continue;
            float cand = nodes[i].cost + std::sqrt(static_cast<float>(d2));
            if (cand < new_cost && is_edge_free(grid, nodes[i].gx, nodes[i].gy, new_x, new_y)) {
                new_cost = cand;
                best_parent = i;
            }
        }

        int new_idx = static_cast<int>(nodes.size());
        nodes.push_back({new_x, new_y, best_parent, new_cost});

        // Rewire
        for (int i = 0; i < new_idx; i++) {
            int ddx = nodes[i].gx - new_x, ddy = nodes[i].gy - new_y;
            int d2 = ddx * ddx + ddy * ddy;
            if (d2 > rewire_sq) continue;
            float rewired = new_cost + std::sqrt(static_cast<float>(d2));
            if (rewired < nodes[i].cost &&
                is_edge_free(grid, new_x, new_y, nodes[i].gx, nodes[i].gy)) {
                nodes[i].parent = new_idx;
                nodes[i].cost = rewired;
                // Propagate cost through subtree
                for (int j = i + 1; j < static_cast<int>(nodes.size()); j++) {
                    if (nodes[j].parent >= 0) {
                        int px = nodes[j].gx - nodes[nodes[j].parent].gx;
                        int py = nodes[j].gy - nodes[nodes[j].parent].gy;
                        float edge = std::sqrt(static_cast<float>(px * px + py * py));
                        nodes[j].cost = nodes[nodes[j].parent].cost + edge;
                    }
                }
            }
        }

        // Goal check
        int gdx = new_x - gx, gdy = new_y - gy;
        if (gdx * gdx + gdy * gdy <= goal_tol_sq) {
            if (new_cost < best_cost) {
                best_cost = new_cost;
                goal_node_idx = new_idx;
            }
        }
    }

    if (goal_node_idx < 0) return result;

    // Extract path
    std::vector<std::pair<int,int>> path;
    int ci = goal_node_idx;
    while (ci >= 0) {
        path.emplace_back(nodes[ci].gx, nodes[ci].gy);
        ci = nodes[ci].parent;
    }
    std::reverse(path.begin(), path.end());
    result.path_cells = std::move(path);
    result.cost = best_cost;
    return result;
}

// ─── Scan Metrics ─────────────────────────────────────────────────────

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

// ─── Coarse Global A* on full occupancy grid ─────────────────────────

struct GlobalPlanResult {
    std::vector<std::pair<double,double>> waypoints;  // world coords
    bool valid = false;
};

static GlobalPlanResult global_astar(
    const nav_msgs::msg::OccupancyGrid &map,
    double robot_x, double robot_y,
    double goal_x, double goal_y,
    int downsample,          // e.g. 4 = use every 4th cell
    double inflation_m,      // inflate obstacles on coarse grid
    int occupied_thresh,
    double waypoint_spacing_m)
{
    GlobalPlanResult result;
    int map_w = static_cast<int>(map.info.width);
    int map_h = static_cast<int>(map.info.height);
    double map_res = map.info.resolution;
    double map_ox = map.info.origin.position.x;
    double map_oy = map.info.origin.position.y;
    if (map_w < 2 || map_h < 2) return result;

    // Coarse grid dimensions
    int cw = (map_w + downsample - 1) / downsample;
    int ch = (map_h + downsample - 1) / downsample;
    double cres = map_res * downsample;

    // Build coarse grid: blocked if any source cell is occupied
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
                        if (val >= occupied_thresh)
                            occ = true;
                    }
                }
            }
            if (occ) cgrid[cy * cw + cx] = 1;
        }
    }

    // Inflate on coarse grid
    int inflate_cells = std::max(0, static_cast<int>(std::ceil(inflation_m / cres)));
    if (inflate_cells > 0) {
        std::vector<std::pair<int,int>> obs;
        for (int cy = 0; cy < ch; cy++)
            for (int cx = 0; cx < cw; cx++)
                if (cgrid[cy * cw + cx]) obs.emplace_back(cx, cy);
        int isq = inflate_cells * inflate_cells;
        for (auto [ox, oy] : obs) {
            for (int dy = -inflate_cells; dy <= inflate_cells; dy++) {
                for (int dx = -inflate_cells; dx <= inflate_cells; dx++) {
                    if (dx*dx + dy*dy > isq) continue;
                    int nx = ox+dx, ny = oy+dy;
                    if (nx >= 0 && nx < cw && ny >= 0 && ny < ch)
                        cgrid[ny * cw + nx] = 1;
                }
            }
        }
    }

    // World → coarse cell
    auto to_cell = [&](double wx, double wy, int &cx, int &cy) {
        cx = static_cast<int>((wx - map_ox) / cres);
        cy = static_cast<int>((wy - map_oy) / cres);
        return cx >= 0 && cx < cw && cy >= 0 && cy < ch;
    };
    auto cell_to_world = [&](int cx, int cy, double &wx, double &wy) {
        wx = map_ox + (cx + 0.5) * cres;
        wy = map_oy + (cy + 0.5) * cres;
    };

    int sx, sy, gx, gy;
    if (!to_cell(robot_x, robot_y, sx, sy)) return result;
    if (!to_cell(goal_x, goal_y, gx, gy)) return result;

    // Clear start cell so robot isn't trapped
    cgrid[sy * cw + sx] = 0;

    // Find nearest free cell for goal if blocked
    if (cgrid[gy * cw + gx]) {
        int best_d2 = std::numeric_limits<int>::max();
        int bx = -1, by = -1;
        int sr = std::max(cw, ch) / 4;
        for (int dy = -sr; dy <= sr; dy++) {
            for (int dx = -sr; dx <= sr; dx++) {
                int nx = gx+dx, ny = gy+dy;
                if (nx >= 0 && nx < cw && ny >= 0 && ny < ch && !cgrid[ny*cw+nx]) {
                    int d2 = dx*dx + dy*dy;
                    if (d2 < best_d2) { best_d2 = d2; bx = nx; by = ny; }
                }
            }
        }
        if (bx < 0) return result;
        gx = bx; gy = by;
    }

    // A* with 8-connected neighbors
    struct ANode { float f; float g; int x, y; };
    auto cmp = [](const ANode &a, const ANode &b) { return a.f > b.f; };
    std::priority_queue<ANode, std::vector<ANode>, decltype(cmp)> open_pq(cmp);

    std::vector<float> g_score(cw * ch, std::numeric_limits<float>::infinity());
    std::vector<int> came_from(cw * ch, -1);
    std::vector<bool> closed(cw * ch, false);

    auto idx = [cw](int x, int y) { return y * cw + x; };
    auto heur = [gx, gy](int x, int y) {
        return std::sqrt(static_cast<float>((x-gx)*(x-gx) + (y-gy)*(y-gy)));
    };

    g_score[idx(sx,sy)] = 0.0f;
    open_pq.push({heur(sx,sy), 0.0f, sx, sy});

    static const int dx8[] = {-1,1,0,0,-1,-1,1,1};
    static const int dy8[] = {0,0,-1,1,-1,1,-1,1};
    static const float cost8[] = {1,1,1,1,1.414f,1.414f,1.414f,1.414f};

    bool found = false;
    while (!open_pq.empty()) {
        auto cur = open_pq.top(); open_pq.pop();
        if (closed[idx(cur.x, cur.y)]) continue;
        closed[idx(cur.x, cur.y)] = true;

        if (cur.x == gx && cur.y == gy) { found = true; break; }

        for (int d = 0; d < 8; d++) {
            int nb_x = cur.x + dx8[d], nb_y = cur.y + dy8[d];
            if (nb_x < 0 || nb_x >= cw || nb_y < 0 || nb_y >= ch) continue;
            if (cgrid[nb_y * cw + nb_x] || closed[idx(nb_x,nb_y)]) continue;
            // Diagonal: check both cardinal neighbors are free
            if (d >= 4) {
                if (cgrid[(cur.y + dy8[d]) * cw + cur.x] ||
                    cgrid[cur.y * cw + (cur.x + dx8[d])])
                    continue;
            }
            float new_g = cur.g + cost8[d];
            if (new_g < g_score[idx(nb_x,nb_y)]) {
                g_score[idx(nb_x,nb_y)] = new_g;
                came_from[idx(nb_x,nb_y)] = idx(cur.x, cur.y);
                open_pq.push({new_g + heur(nb_x,nb_y), new_g, nb_x, nb_y});
            }
        }
    }

    if (!found) return result;

    // Reconstruct path
    std::vector<std::pair<int,int>> path_cells;
    int ci = idx(gx, gy);
    while (ci >= 0) {
        int cy = ci / cw, cx = ci % cw;
        path_cells.emplace_back(cx, cy);
        ci = came_from[ci];
    }
    std::reverse(path_cells.begin(), path_cells.end());

    // Convert to world coords, subsample by spacing
    result.waypoints.clear();
    double last_wx = robot_x, last_wy = robot_y;
    for (auto [cx, cy] : path_cells) {
        double wx, wy;
        cell_to_world(cx, cy, wx, wy);
        if (std::hypot(wx - last_wx, wy - last_wy) >= waypoint_spacing_m) {
            result.waypoints.emplace_back(wx, wy);
            last_wx = wx; last_wy = wy;
        }
    }
    // Always include the final goal
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

// ─── Nearest traversable cell to goal ─────────────────────────────────

static bool nearest_traversable(const LocalGrid &grid, int &gx, int &gy, int search_r) {
    if (!grid.blocked(gx, gy)) return true;
    int n = grid.n;
    int best_x = -1, best_y = -1;
    int best_d2 = std::numeric_limits<int>::max();
    for (int dy = -search_r; dy <= search_r; dy++) {
        for (int dx = -search_r; dx <= search_r; dx++) {
            int nx = gx + dx, ny = gy + dy;
            if (nx < 0 || nx >= n || ny < 0 || ny >= n) continue;
            if (grid.data[ny * n + nx]) continue;
            int d2 = dx * dx + dy * dy;
            if (d2 < best_d2) { best_d2 = d2; best_x = nx; best_y = ny; }
        }
    }
    if (best_x < 0) return false;
    gx = best_x; gy = best_y;
    return true;
}

// ═══════════════════════════════════════════════════════════════════════
//  ROS2 Node
// ═══════════════════════════════════════════════════════════════════════

class ReactiveNavNode : public rclcpp::Node {
public:
    ReactiveNavNode() : Node("reactive_nav"), rng_(std::random_device{}()) {
        // Declare parameters
        declare_parameter("max_linear_speed", 0.45);
        declare_parameter("max_angular_speed", 0.85);
        declare_parameter("control_rate", 12.0);
        declare_parameter("startup_delay", 0.0);
        declare_parameter("goal_tolerance", 0.20);
        declare_parameter("goal_reached_replan_cooldown_sec", 1.0);
        declare_parameter("obstacle_slow_dist", 0.75);
        declare_parameter("obstacle_stop_dist", 0.35);
        declare_parameter("front_half_angle_deg", 35.0);
        declare_parameter("side_check_angle_deg", 60.0);
        declare_parameter("avoidance_gain", 1.0);
        declare_parameter("turn_in_place_on_block", true);
        declare_parameter("rrt_sample_count", 500);
        declare_parameter("rrt_step_size", 0.30);
        declare_parameter("rrt_goal_bias", 0.15);
        declare_parameter("rrt_rewire_radius", 0.60);
        declare_parameter("rrt_grid_radius", 4.0);
        declare_parameter("rrt_grid_resolution", 0.10);
        declare_parameter("rrt_inflation_radius", 0.22);
        declare_parameter("rrt_replan_sec", 0.5);
        declare_parameter("rrt_informed_enabled", true);
        declare_parameter("rrt_goal_clip_distance", 3.5);
        declare_parameter("rrt_waypoint_lookahead", 0.50);
        declare_parameter("rrt_goal_search_radius", 1.5);
        declare_parameter("rrt_start_clearance_radius", 0.12);
        declare_parameter("rrt_unknown_is_obstacle", false);
        declare_parameter("map_topic", std::string(""));
        declare_parameter("map_frame", std::string("map"));
        declare_parameter("map_occupied_thresh", 50);
        declare_parameter("frontier_replan_topic", std::string("/frontier_replan"));
        declare_parameter("stop_topic", std::string("/stop"));

        // Global A* planner parameters
        declare_parameter("global_downsample", 4);
        declare_parameter("global_inflation_m", 0.30);
        declare_parameter("global_replan_sec", 3.0);
        declare_parameter("global_waypoint_spacing_m", 1.5);

        load_params();

        // Allocate grid
        int cells = std::max(31, static_cast<int>(
            std::ceil(2.0 * rrt_grid_radius_ / rrt_grid_resolution_)) + 1);
        cells = cells | 1;  // ensure odd
        grid_.n = cells;
        grid_.resolution = rrt_grid_resolution_;
        grid_.data.resize(cells * cells, 0);

        // TF2 for looking up robot pose in map frame
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        map_frame_ = get_parameter("map_frame").as_string();

        // Publishers — use absolute topic names so launch-file remappings
        // (which also use absolute names) match correctly.
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

        // Subscribers
        auto sensor_qos = rclcpp::SensorDataQoS();
        auto best_effort_qos = rclcpp::QoS(1)
            .reliability(rclcpp::ReliabilityPolicy::BestEffort)
            .durability(rclcpp::DurabilityPolicy::Volatile);

        wp_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
            "/way_point", 10,
            [this](geometry_msgs::msg::PointStamped::SharedPtr msg) {
                double new_gx = msg->point.x, new_gy = msg->point.y;
                // Invalidate global plan if goal changed significantly
                if (std::hypot(new_gx - goal_x_, new_gy - goal_y_) > 0.5) {
                    global_waypoints_.clear();
                    last_global_plan_time_ = -1;
                }
                goal_x_ = new_gx; goal_y_ = new_gy;
                has_goal_ = true;
                goal_frame_ = msg->header.frame_id.empty() ? "world" : msg->header.frame_id;
            });

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom/ground_truth", sensor_qos,
            [this](nav_msgs::msg::Odometry::SharedPtr msg) {
                robot_x_ = msg->pose.pose.position.x;
                robot_y_ = msg->pose.pose.position.y;
                auto &q = msg->pose.pose.orientation;
                robot_yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                         1.0 - 2.0 * (q.y * q.y + q.z * q.z));
                robot_speed_ = std::hypot(msg->twist.twist.linear.x,
                                           msg->twist.twist.linear.y);
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

        // Map subscription (global occupancy grid)
        auto map_topic_param = get_parameter("map_topic").as_string();
        if (map_topic_param.empty()) {
            // Default: /{namespace}/map
            map_topic_param = "/" + std::string(get_namespace()) + "/map";
            // Strip double leading slashes
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

        // Control timer
        double period = 1.0 / control_rate_;
        timer_ = create_wall_timer(
            std::chrono::duration<double>(period),
            std::bind(&ReactiveNavNode::tick, this));

        RCLCPP_INFO(get_logger(),
            "Reactive nav (RRT* C++) started: rate=%.0fHz samples=%d step=%.2fm",
            control_rate_, rrt_sample_count_, rrt_step_size_);
    }

private:
    void load_params() {
        max_linear_speed_ = get_parameter("max_linear_speed").as_double();
        max_angular_speed_ = get_parameter("max_angular_speed").as_double();
        control_rate_ = get_parameter("control_rate").as_double();
        startup_delay_ = get_parameter("startup_delay").as_double();
        goal_tolerance_ = get_parameter("goal_tolerance").as_double();
        goal_reached_replan_cooldown_ = get_parameter("goal_reached_replan_cooldown_sec").as_double();
        obstacle_slow_dist_ = get_parameter("obstacle_slow_dist").as_double();
        obstacle_stop_dist_ = get_parameter("obstacle_stop_dist").as_double();
        front_half_ = get_parameter("front_half_angle_deg").as_double() * M_PI / 180.0;
        side_half_ = get_parameter("side_check_angle_deg").as_double() * M_PI / 180.0;
        avoidance_gain_ = get_parameter("avoidance_gain").as_double();
        turn_in_place_on_block_ = get_parameter("turn_in_place_on_block").as_bool();
        rrt_sample_count_ = get_parameter("rrt_sample_count").as_int();
        rrt_step_size_ = get_parameter("rrt_step_size").as_double();
        rrt_goal_bias_ = get_parameter("rrt_goal_bias").as_double();
        rrt_rewire_radius_ = get_parameter("rrt_rewire_radius").as_double();
        rrt_grid_radius_ = get_parameter("rrt_grid_radius").as_double();
        rrt_grid_resolution_ = get_parameter("rrt_grid_resolution").as_double();
        rrt_inflation_radius_ = get_parameter("rrt_inflation_radius").as_double();
        rrt_replan_sec_ = get_parameter("rrt_replan_sec").as_double();
        rrt_informed_ = get_parameter("rrt_informed_enabled").as_bool();
        rrt_goal_clip_distance_ = get_parameter("rrt_goal_clip_distance").as_double();
        rrt_waypoint_lookahead_ = get_parameter("rrt_waypoint_lookahead").as_double();
        rrt_goal_search_radius_ = get_parameter("rrt_goal_search_radius").as_double();
        rrt_start_clearance_ = get_parameter("rrt_start_clearance_radius").as_double();
        rrt_unknown_is_obstacle_ = get_parameter("rrt_unknown_is_obstacle").as_bool();
        map_occupied_thresh_ = static_cast<int>(get_parameter("map_occupied_thresh").as_int());

        global_downsample_ = static_cast<int>(get_parameter("global_downsample").as_int());
        global_inflation_m_ = get_parameter("global_inflation_m").as_double();
        global_replan_sec_ = get_parameter("global_replan_sec").as_double();
        global_waypoint_spacing_m_ = get_parameter("global_waypoint_spacing_m").as_double();
    }

    // ── Main tick ────────────────────────────────────────────────────

    void tick() {
        auto now = this->now();
        double now_sec = now.seconds();

        // Startup delay
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
        // (odom pose may differ from map pose when Cartographer corrects drift)
        try {
            auto tf = tf_buffer_->lookupTransform(
                map_frame_, "base_link", tf2::TimePointZero);
            map_robot_x_ = tf.transform.translation.x;
            map_robot_y_ = tf.transform.translation.y;
            map_robot_yaw_ = tf2::getYaw(tf.transform.rotation);
            has_map_tf_ = true;
        } catch (const tf2::TransformException &ex) {
            if (!has_map_tf_) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                    "No TF %s→base_link: %s (using odom pose)", map_frame_.c_str(), ex.what());
                map_robot_x_ = robot_x_;
                map_robot_y_ = robot_y_;
                map_robot_yaw_ = robot_yaw_;
            }
            // else keep the last good TF pose
        }

        double goal_dx = goal_x_ - map_robot_x_;
        double goal_dy = goal_y_ - map_robot_y_;
        double dist_to_goal = std::hypot(goal_dx, goal_dy);

        // Goal reached
        if (dist_to_goal < goal_tolerance_) {
            publish_cmd(0.0, 0.0);
            if (last_replan_time_ < 0 || (now_sec - last_replan_time_) >= goal_reached_replan_cooldown_) {
                replan_pub_->publish(std_msgs::msg::Empty());
                last_replan_time_ = now_sec;
            }
            return;
        }

        // Scan metrics
        auto sm = analyze_scan(*last_scan_, front_half_, side_half_, obstacle_slow_dist_);

        // ── Global A* coarse planner ──────────────────────────────────
        // Periodically compute a coarse global route on the full occupancy
        // grid, then feed an intermediate waypoint (within RRT* range) to
        // the local RRT* planner instead of the raw distant frontier goal.
        bool global_replan_due = last_global_plan_time_ < 0
            || (now_sec - last_global_plan_time_) >= global_replan_sec_
            || global_waypoints_.empty();

        if (global_replan_due && last_map_ && dist_to_goal > rrt_grid_radius_ * 0.8) {
            auto gplan = global_astar(
                *last_map_,
                map_robot_x_, map_robot_y_,
                goal_x_, goal_y_,
                global_downsample_,
                global_inflation_m_,
                map_occupied_thresh_,
                global_waypoint_spacing_m_);
            if (gplan.valid && !gplan.waypoints.empty()) {
                global_waypoints_ = std::move(gplan.waypoints);
                last_global_plan_time_ = now_sec;
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                    "Global A*: %zu waypoints to goal (%.1f, %.1f)",
                    global_waypoints_.size(), goal_x_, goal_y_);
            } else {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                    "Global A* failed to find path to (%.1f, %.1f)", goal_x_, goal_y_);
                last_global_plan_time_ = now_sec;  // avoid spamming
            }
            publish_global_path();
        }

        // Pick intermediate goal from global plan for local RRT*
        double local_goal_x = goal_x_, local_goal_y = goal_y_;
        if (!global_waypoints_.empty() && dist_to_goal > rrt_grid_radius_ * 0.8) {
            // Skip past waypoints the robot has already reached
            while (global_waypoints_.size() > 1) {
                double d = std::hypot(global_waypoints_.front().first - map_robot_x_,
                                       global_waypoints_.front().second - map_robot_y_);
                if (d < goal_tolerance_ * 2.0)
                    global_waypoints_.erase(global_waypoints_.begin());
                else
                    break;
            }
            // Find the furthest waypoint within RRT* reach
            double reach = rrt_grid_radius_ * 0.85;
            for (size_t i = 0; i < global_waypoints_.size(); i++) {
                double d = std::hypot(global_waypoints_[i].first - map_robot_x_,
                                       global_waypoints_[i].second - map_robot_y_);
                if (d <= reach) {
                    local_goal_x = global_waypoints_[i].first;
                    local_goal_y = global_waypoints_[i].second;
                } else {
                    break;  // waypoints are ordered; first one out of range stops search
                }
            }
        }

        double local_goal_dx = local_goal_x - map_robot_x_;
        double local_goal_dy = local_goal_y - map_robot_y_;

        // Replan with RRT* if needed
        bool need_replan = path_world_.empty()
            || last_plan_time_ < 0
            || (now_sec - last_plan_time_) >= rrt_replan_sec_;

        if (need_replan) {
            plan_path(local_goal_dx, local_goal_dy);
            last_plan_time_ = now_sec;
        }

        // Select pursuit target
        double target_x = goal_x_, target_y = goal_y_;
        if (!path_world_.empty()) {
            // Find closest waypoint, then look ahead
            size_t best_idx = 0;
            double best_d2 = std::numeric_limits<double>::max();
            for (size_t i = 0; i < path_world_.size(); i++) {
                double dx = path_world_[i].first - map_robot_x_;
                double dy = path_world_[i].second - map_robot_y_;
                double d2 = dx * dx + dy * dy;
                if (d2 < best_d2) { best_d2 = d2; best_idx = i; }
            }
            for (size_t i = best_idx; i < path_world_.size(); i++) {
                double d = std::hypot(path_world_[i].first - map_robot_x_,
                                       path_world_[i].second - map_robot_y_);
                if (d >= rrt_waypoint_lookahead_) {
                    target_x = path_world_[i].first;
                    target_y = path_world_[i].second;
                    break;
                }
                target_x = path_world_[i].first;
                target_y = path_world_[i].second;
            }
        }

        // Controller
        double heading_err = wrap_angle(std::atan2(target_y - map_robot_y_, target_x - map_robot_x_) - map_robot_yaw_);

        // Angular
        double ang = max_angular_speed_ * (2.0 / M_PI) * heading_err;
        ang = std::clamp(ang, -max_angular_speed_, max_angular_speed_);
        double avoid_yaw = avoidance_gain_ * (sm.right_push - sm.left_push);
        ang = std::clamp(ang + avoid_yaw, -max_angular_speed_, max_angular_speed_);

        // Linear
        double heading_factor = std::max(0.0, std::cos(heading_err));
        double lin = max_linear_speed_ * heading_factor;
        if (sm.min_front < obstacle_slow_dist_) {
            double rng = obstacle_slow_dist_ - obstacle_stop_dist_;
            if (rng > 1e-6)
                lin *= std::max(0.0, (sm.min_front - obstacle_stop_dist_) / rng);
        }
        bool blocked_front = sm.min_front < obstacle_stop_dist_;
        if (blocked_front || external_stop_ != 0) {
            lin = 0.0;
            if (blocked_front && turn_in_place_on_block_ && external_stop_ == 0) {
                if (std::abs(ang) < 0.1)
                    ang = max_angular_speed_ * 0.5 * ((sm.right_push > sm.left_push) ? 1.0 : -1.0);
            } else {
                ang = 0.0;
            }
        }

        publish_cmd(lin, ang);

        // Blocked with no path -> request frontier replan
        if (blocked_front && path_world_.empty()) {
            replan_pub_->publish(std_msgs::msg::Empty());
        }

        // Publish path visualization
        publish_path();

        // Trajectory
        trajectory_.emplace_back(map_robot_x_, map_robot_y_);
        if (trajectory_.size() > 2000) trajectory_.erase(trajectory_.begin());
        publish_trajectory();

        // Markers
        publish_goal_marker();
        publish_pose_marker();
    }

    // ── RRT* Planning ────────────────────────────────────────────────

    void plan_path(double goal_dx, double goal_dy) {
        double cos_y = std::cos(map_robot_yaw_), sin_y = std::sin(map_robot_yaw_);
        double goal_lx =  cos_y * goal_dx + sin_y * goal_dy;
        double goal_ly = -sin_y * goal_dx + cos_y * goal_dy;

        double goal_dist = std::hypot(goal_lx, goal_ly);
        double max_plan = std::min(rrt_goal_clip_distance_, rrt_grid_radius_ * 0.9);
        if (goal_dist > max_plan && goal_dist > 1e-6) {
            double s = max_plan / goal_dist;
            goal_lx *= s; goal_ly *= s;
        }

        // Build local grid from global occupancy map only
        build_local_grid(
            last_map_ ? last_map_.get() : nullptr,
            grid_,
            map_robot_x_, map_robot_y_, map_robot_yaw_,
            rrt_inflation_radius_, rrt_start_clearance_,
            rrt_unknown_is_obstacle_, map_occupied_thresh_);

        int c = grid_.center();
        int gx, gy;
        if (!grid_.local_to_cell(goal_lx, goal_ly, gx, gy)) {
            return;  // keep last good plan
        }

        int search_r = std::max(8, static_cast<int>(std::round(
            rrt_goal_search_radius_ / rrt_grid_resolution_)));
        if (!nearest_traversable(grid_, gx, gy, search_r)) {
            return;  // keep last good plan
        }

        int step_cells = std::max(1, static_cast<int>(std::round(
            rrt_step_size_ / rrt_grid_resolution_)));
        int rewire_cells = std::max(1, static_cast<int>(std::round(
            rrt_rewire_radius_ / rrt_grid_resolution_)));
        int goal_tol_cells = std::max(2, static_cast<int>(std::round(
            goal_tolerance_ / rrt_grid_resolution_)));

        auto result = run_rrt_star(
            grid_, c, c, gx, gy,
            rrt_sample_count_, step_cells, rewire_cells,
            rrt_goal_bias_, goal_tol_cells, rrt_informed_, rng_);

        if (result.path_cells.empty()) {
            // Keep following the last good plan rather than falling back
            // to blind direct-goal pursuit which ignores obstacles.
            return;
        }

        // Convert to map-frame coordinates
        path_world_.clear();
        path_world_.reserve(result.path_cells.size());
        for (auto [cx, cy] : result.path_cells) {
            double lx, ly;
            grid_.cell_to_local(cx, cy, lx, ly);
            double wx = map_robot_x_ + cos_y * lx - sin_y * ly;
            double wy = map_robot_y_ + sin_y * lx + cos_y * ly;
            path_world_.emplace_back(wx, wy);
        }
    }

    // ── Publishing helpers ───────────────────────────────────────────

    void publish_cmd(double lin, double ang) {
        geometry_msgs::msg::TwistStamped msg;
        msg.header.stamp = this->now();
        msg.twist.linear.x = lin;
        msg.twist.angular.z = ang;
        cmd_pub_->publish(msg);
    }

    void publish_path() {
        nav_msgs::msg::Path msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = map_frame_;
        for (auto &[wx, wy] : path_world_) {
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
        m.ns = "reactive_nav_goal"; m.id = 0;
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
        m.ns = "reactive_nav_pose"; m.id = 0;
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

    // Parameters
    double max_linear_speed_, max_angular_speed_, control_rate_, startup_delay_;
    double goal_tolerance_, goal_reached_replan_cooldown_;
    double obstacle_slow_dist_, obstacle_stop_dist_;
    double front_half_, side_half_, avoidance_gain_;
    bool turn_in_place_on_block_;
    int rrt_sample_count_;
    double rrt_step_size_, rrt_goal_bias_, rrt_rewire_radius_;
    double rrt_grid_radius_, rrt_grid_resolution_, rrt_inflation_radius_;
    double rrt_replan_sec_, rrt_goal_clip_distance_, rrt_waypoint_lookahead_;
    double rrt_goal_search_radius_, rrt_start_clearance_;
    bool rrt_informed_, rrt_unknown_is_obstacle_;
    int map_occupied_thresh_;

    // Global A* parameters
    int global_downsample_;
    double global_inflation_m_, global_replan_sec_, global_waypoint_spacing_m_;

    // Robot state (odom frame — used only for speed)
    double robot_x_ = 0, robot_y_ = 0, robot_yaw_ = 0, robot_speed_ = 0;
    // Robot state (map frame — from TF2, used for planning & control)
    double map_robot_x_ = 0, map_robot_y_ = 0, map_robot_yaw_ = 0;
    bool has_map_tf_ = false;
    double goal_x_ = 0, goal_y_ = 0;
    bool has_goal_ = false;
    std::string goal_frame_ = "world";
    std::string map_frame_ = "map";
    int external_stop_ = 0;
    double start_time_ = -1;
    double last_plan_time_ = -1;
    double last_replan_time_ = -1;

    // Grid & plan
    LocalGrid grid_;
    std::vector<std::pair<double,double>> path_world_;
    std::vector<std::pair<double,double>> trajectory_;
    std::vector<std::pair<double,double>> global_waypoints_;
    double last_global_plan_time_ = -1;
    std::mt19937 rng_;

    // Scan & map
    sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
    nav_msgs::msg::OccupancyGrid::SharedPtr last_map_;

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
    rclcpp::spin(std::make_shared<ReactiveNavNode>());
    rclcpp::shutdown();
    return 0;
}
