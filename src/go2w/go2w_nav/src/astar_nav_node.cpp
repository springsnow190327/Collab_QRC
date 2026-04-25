/*
 * astar_nav_node.cpp — A* planner + curvature-aware pure-pursuit for Go2W.
 *
 * Same I/O contract as reactive_nav_node / mppi_nav_node (drop-in):
 *   subs:  /way_point (goal)  /odom/ground_truth  /scan  /{ns}/map
 *   pubs:  /cmd_vel_stamped  /nav_status  /planned_path  /robot_trajectory
 *          /final_goal_marker  /robot_pose_marker
 *
 * Design rationale (Go2W-specific):
 *   - Global A* on the 2D OccupancyGrid from octomap. Classic 8-connected,
 *     downsampled, with inflation. Reuses the same routine mppi_nav_node
 *     proved on this platform.
 *   - Pure-pursuit lookahead on the A* path, with lookahead distance that
 *     GROWS with speed (classic) but also GROWS with upcoming curvature's
 *     sign-flip (so a slalom doesn't chase each cusp).
 *   - Speed shaping pipeline:
 *         v = v_max
 *             × ramp_up(t)                  // smooth startup 0→1
 *             × goal_approach(d_goal)       // brake near goal
 *             × curvature_factor(κ_ahead)   // slow in corners
 *             × safety_factor(scan_dist)    // scan-based brake
 *   - Turn-mode selection happens automatically via the hybrid_cmd_router
 *     downstream. We shape the Twist so that:
 *         straight  → v > wheel_linear_threshold, |ω| < wheel_angular_thresh
 *                    → router picks WHEEL mode (fast, quiet, efficient)
 *         corner    → v drops below wheel_linear_threshold, |ω| grows
 *                    → router picks LEGGED mode (CHAMP in-place-turn, safer)
 *     The critical numbers (`wheel_*_threshold_for_bias`) live in this yaml
 *     so the planner KNOWS what the router will do and biases accordingly.
 */

#include <cmath>
#include <cstdint>
#include <algorithm>
#include <limits>
#include <queue>
#include <sstream>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
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

// ───────────── small math helpers ─────────────

static inline double wrap_angle(double a) {
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

static inline double clamp01(double x) {
    return (x < 0.0) ? 0.0 : (x > 1.0) ? 1.0 : x;
}

// Linear ramp: x in [lo, hi] → [out_lo, out_hi], clamped at both ends.
static inline double ramp(double x, double lo, double hi, double out_lo, double out_hi) {
    if (hi - lo < 1e-9) return (x < lo) ? out_lo : out_hi;
    double t = clamp01((x - lo) / (hi - lo));
    return out_lo + t * (out_hi - out_lo);
}

// ───────────── scan safety (same shape as mppi_nav_node) ─────────────

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
            if (bearing > 0 && bearing <  side_half) { left_acc  += w; left_w  += 1.0; }
            else if (bearing < 0 && bearing > -side_half) { right_acc += w; right_w += 1.0; }
        }
        angle += scan.angle_increment;
    }
    m.left_push  = (left_w  > 0) ? left_acc  / left_w  : 0.0;
    m.right_push = (right_w > 0) ? right_acc / right_w : 0.0;
    return m;
}

// ───────────── global A* (copied from mppi_nav_node — proven) ─────────────

struct AStarResult {
    std::vector<std::pair<double,double>> waypoints;
    bool valid = false;
};

// Small circular "no-go" disk injected as blocked cells during A*. Used to
// re-route around places where oriented-footprint validation failed on a
// previous attempt in the same tick (Option B retry loop) or during the
// last few seconds (persistent TTL store).
struct NogoDisc {
    double cx;   // world-frame centre
    double cy;
    double r;    // metres
};

static AStarResult plan_astar(
    const nav_msgs::msg::OccupancyGrid &map,
    double robot_x, double robot_y,
    double goal_x, double goal_y,
    int downsample, double inflation_m,
    int occupied_thresh, double waypoint_spacing_m,
    bool unknown_is_obstacle,
    const std::vector<NogoDisc> &nogo_disks = {},
    int clearance_target_cells = 0,
    float clearance_weight = 0.0f)
{
    AStarResult result;
    int map_w = static_cast<int>(map.info.width);
    int map_h = static_cast<int>(map.info.height);
    double map_res = map.info.resolution;
    double map_ox = map.info.origin.position.x;
    double map_oy = map.info.origin.position.y;
    if (map_w < 2 || map_h < 2) return result;

    int cw = (map_w + downsample - 1) / downsample;
    int ch = (map_h + downsample - 1) / downsample;
    double cres = map_res * downsample;

    // Two separate grids so we can treat them differently downstream:
    //   cgrid_occ: cells with any OCCUPIED fine-cell (real obstacles)
    //   cgrid_unk: cells with NO occupied but at least one UNKNOWN fine-cell
    // Walls get morphological closing + inflation (safety buffer). Unknown
    // regions get only "treat as blocked" — no dilation — because they're
    // already large, and dilating them eats up massive free space around
    // every frontier (prevents A* from reaching goals at the edge of
    // explored territory).
    std::vector<uint8_t> cgrid_occ(cw * ch, 0);
    std::vector<uint8_t> cgrid_unk(cw * ch, 0);
    for (int cy = 0; cy < ch; cy++) {
        for (int cx = 0; cx < cw; cx++) {
            bool has_occ = false, has_unk = false;
            for (int dy = 0; dy < downsample; dy++) {
                for (int dx = 0; dx < downsample; dx++) {
                    int mx = cx * downsample + dx, my = cy * downsample + dy;
                    if (mx < map_w && my < map_h) {
                        int8_t val = map.data[my * map_w + mx];
                        if (val >= occupied_thresh) has_occ = true;
                        else if (val < 0) has_unk = true;
                    } else {
                        has_unk = true;  // outside map = unknown
                    }
                }
            }
            if (has_occ) cgrid_occ[cy * cw + cx] = 1;
            else if (has_unk) cgrid_unk[cy * cw + cx] = 1;
        }
    }
    // cgrid is the working grid for A*: starts as cgrid_occ; closing +
    // inflation mutate it. Unknown is added at the END as a plain block
    // without further dilation.
    std::vector<uint8_t> cgrid = cgrid_occ;

    // Morphological closing to fill sparse gaps in thin-wall octomap
    // projections (e.g. demo1 divider_v_north, 0.15 m thick). Single
    // dilation + erosion (radius 1): fills 1-2 cell holes, preserves
    // corridor widths. Previous 2-pass version over-closed and blocked
    // narrow passages like the south corridor around divider_v_north.
    {
        std::vector<uint8_t> dilated(cw * ch, 0);
        for (int cy = 0; cy < ch; cy++) {
            for (int cx = 0; cx < cw; cx++) {
                if (cgrid[cy * cw + cx]) { dilated[cy * cw + cx] = 1; continue; }
                bool any = false;
                for (int dy = -1; dy <= 1 && !any; dy++) {
                    for (int dx = -1; dx <= 1 && !any; dx++) {
                        int nx = cx + dx, ny = cy + dy;
                        if (nx >= 0 && nx < cw && ny >= 0 && ny < ch &&
                            cgrid[ny * cw + nx]) any = true;
                    }
                }
                if (any) dilated[cy * cw + cx] = 1;
            }
        }
        // Erosion: restore obstacle boundary by keeping only cells with
        // all 8 dilated-neighbors set. True gaps (interior holes) still
        // get filled since they have all-obstacle neighbors after dilate.
        for (int cy = 0; cy < ch; cy++) {
            for (int cx = 0; cx < cw; cx++) {
                if (!dilated[cy * cw + cx]) continue;
                bool all_set = true;
                for (int dy = -1; dy <= 1 && all_set; dy++) {
                    for (int dx = -1; dx <= 1 && all_set; dx++) {
                        int nx = cx + dx, ny = cy + dy;
                        if (nx < 0 || nx >= cw || ny < 0 || ny >= ch) continue;
                        if (!dilated[ny * cw + nx]) all_set = false;
                    }
                }
                if (all_set) cgrid[cy * cw + cx] = 1;
            }
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
                    int nx = ox + ddx, ny = oy + ddy;
                    if (nx >= 0 && nx < cw && ny >= 0 && ny < ch)
                        cgrid[ny * cw + nx] = 1;
                }
            }
        }
    }

    // Now add UNKNOWN cells as blocked — no dilation. This way A* avoids
    // planning through unexplored space (prevents "ghost path through
    // wall" bug) but doesn't inflate huge swaths of map around frontiers.
    if (unknown_is_obstacle) {
        for (int cy = 0; cy < ch; cy++)
            for (int cx = 0; cx < cw; cx++)
                if (cgrid_unk[cy * cw + cx]) cgrid[cy * cw + cx] = 1;
    }

    // Apply no-go disks (from Option B footprint-validation failures and
    // from the persistent short-TTL store). Each disk stamps a solid
    // blocked region on the coarse grid so A* routes around it.
    if (!nogo_disks.empty()) {
        for (const auto &d : nogo_disks) {
            int cx0 = static_cast<int>((d.cx - map_ox) / cres);
            int cy0 = static_cast<int>((d.cy - map_oy) / cres);
            int rc  = std::max(1, static_cast<int>(std::ceil(d.r / cres)));
            int rcsq = rc * rc;
            for (int ddy = -rc; ddy <= rc; ddy++) {
                for (int ddx = -rc; ddx <= rc; ddx++) {
                    if (ddx * ddx + ddy * ddy > rcsq) continue;
                    int nx = cx0 + ddx, ny = cy0 + ddy;
                    if (nx >= 0 && nx < cw && ny >= 0 && ny < ch)
                        cgrid[ny * cw + nx] = 1;
                }
            }
        }
    }

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
    to_coarse(goal_x,  goal_y,  gx, gy);
    sx = std::clamp(sx, 0, cw-1); sy = std::clamp(sy, 0, ch-1);
    gx = std::clamp(gx, 0, cw-1); gy = std::clamp(gy, 0, ch-1);

    // Always-free DISK around the start cell (radius = inflation cells).
    // The previous one-cell free start (`cgrid[sy*cw+sx]=0`) handled the
    // case where the start *cell* sat on an occupied cell, but did nothing
    // for the more common spawn-near-obstacle case: when the robot is
    // inside an obstacle's inflation halo, all 8-connected neighbours of
    // the start cell are still marked occupied → A* has zero successors
    // and exits with no_plan forever.
    //
    // Concrete example from demo3_mixed (2026-04-25 run): robot_b spawns
    // at (4, -6); sw_v_1 box at (5, -4.5), 1.12 m NE; with inflation=0.18
    // and downsample=3 (cres=0.15), inflation halo extends ~3 coarse cells
    // (≈0.45 m) past the box edge → every neighbour of B's start cell is
    // inside that halo → A* fails the very first expansion → nav_state
    // sticks at "no_plan", v=ω=0, robot doesn't move all session.
    //
    // Clearing a disk of `inflate_cells` around the start guarantees the
    // robot can take its first move out. It's safe because:
    //   1. The robot is physically AT the start cell — if a collision
    //      were already present, the sim would have flagged it; the
    //      planner-side belief in inflation isn't a physics constraint.
    //   2. The disk radius equals the inflation radius, so cells just
    //      beyond the disk still carry their inflation buffer; we only
    //      override the inflation that surrounds the robot's own body.
    {
        int rs = std::max(1, inflate_cells);
        int rs2 = rs * rs;
        for (int dy = -rs; dy <= rs; dy++) {
            for (int dx = -rs; dx <= rs; dx++) {
                if (dx*dx + dy*dy > rs2) continue;
                int nx = sx + dx, ny = sy + dy;
                if (nx >= 0 && nx < cw && ny >= 0 && ny < ch)
                    cgrid[ny * cw + nx] = 0;
            }
        }
    }

    // If the goal cell is blocked (obstacle or inflation), relocate to
    // the closest free cell within a small radius — frontiers often sit
    // right on the occupied/unknown boundary and get caught by inflation.
    if (cgrid[gy * cw + gx]) {
        int best_x = -1, best_y = -1;
        int best_d2 = std::numeric_limits<int>::max();
        int R = 6;  // ~6 coarse cells ≈ 0.9 m at downsample=3
        for (int dy = -R; dy <= R; dy++) {
            for (int dx = -R; dx <= R; dx++) {
                int nx = gx + dx, ny = gy + dy;
                if (nx < 0 || nx >= cw || ny < 0 || ny >= ch) continue;
                if (cgrid[ny * cw + nx]) continue;
                int d2 = dx*dx + dy*dy;
                if (d2 < best_d2) { best_d2 = d2; best_x = nx; best_y = ny; }
            }
        }
        if (best_x < 0) return result;  // truly unreachable
        gx = best_x; gy = best_y;
    }

    // ── clearance distance transform (cost shaping) ──────────────
    // For every free cell, store the cell-distance to the nearest occupied
    // cell (occupied = real obstacle ∪ inflation halo ∪ unknown ∪ nogo).
    // Used in the A* expand loop to penalize paths that hug walls. Without
    // this, equal-step-cost A* picks routes tangent to the inflation
    // boundary indistinguishably from corridor centrelines — observed in
    // demo3_mixed 2026-04-25 where A* repeatedly chose path tangents
    // within 0.15 m of zigzag_3 / cross_h_w / wall_west. Under any
    // pose drift the body rectangle then clipped the wall.
    //
    // Two-pass chamfer 3-4 metric: cheap O(cw·ch), gives euclidean
    // distance accurate to ~10%. Result clamped to clearance_target_cells
    // (we don't care about clearance beyond the target).
    std::vector<int> clr;
    if (clearance_target_cells > 0 && clearance_weight > 0.0f) {
        const int kBigDist = 1 << 20;
        clr.assign(cw * ch, kBigDist);
        for (int i = 0; i < cw * ch; i++) if (cgrid[i]) clr[i] = 0;
        // Forward pass.
        for (int y = 0; y < ch; y++) {
            for (int x = 0; x < cw; x++) {
                int idx = y * cw + x;
                if (clr[idx] == 0) continue;
                int v = clr[idx];
                if (x > 0)             v = std::min(v, clr[idx - 1]      + 3);
                if (y > 0)             v = std::min(v, clr[idx - cw]     + 3);
                if (x > 0 && y > 0)    v = std::min(v, clr[idx - cw - 1] + 4);
                if (x < cw-1 && y > 0) v = std::min(v, clr[idx - cw + 1] + 4);
                clr[idx] = v;
            }
        }
        // Backward pass.
        for (int y = ch - 1; y >= 0; y--) {
            for (int x = cw - 1; x >= 0; x--) {
                int idx = y * cw + x;
                if (clr[idx] == 0) continue;
                int v = clr[idx];
                if (x < cw-1)             v = std::min(v, clr[idx + 1]      + 3);
                if (y < ch-1)             v = std::min(v, clr[idx + cw]     + 3);
                if (x < cw-1 && y < ch-1) v = std::min(v, clr[idx + cw + 1] + 4);
                if (x > 0 && y < ch-1)    v = std::min(v, clr[idx + cw - 1] + 4);
                clr[idx] = v;
            }
        }
        // Convert chamfer-units → cells (horizontal step = 3 chamfer units).
        // Cap at target — beyond that the penalty is zero anyway.
        for (int i = 0; i < cw * ch; i++) {
            int cells = clr[i] / 3;
            clr[i] = std::min(clearance_target_cells, cells);
        }
    }

    struct ANode { int x, y; float g, f; };
    std::vector<float> best(cw * ch, std::numeric_limits<float>::infinity());
    std::vector<int> prev_x(cw * ch, -1), prev_y(cw * ch, -1);
    auto cmp = [](const ANode &a, const ANode &b) { return a.f > b.f; };
    std::priority_queue<ANode, std::vector<ANode>, decltype(cmp)> pq(cmp);

    best[sy * cw + sx] = 0;
    pq.push({sx, sy, 0.0f, static_cast<float>(std::hypot(gx-sx, gy-sy))});

    static const int adx[]   = {-1,  0,  1, -1,  1, -1,  0,  1};
    static const int ady[]   = {-1, -1, -1,  0,  0,  1,  1,  1};
    static const float acost[]= {1.414f, 1.0f, 1.414f, 1.0f, 1.0f, 1.414f, 1.0f, 1.414f};

    bool found = false;
    while (!pq.empty()) {
        auto cur = pq.top(); pq.pop();
        if (cur.g > best[cur.y * cw + cur.x] + 1e-3f) continue;
        if (cur.x == gx && cur.y == gy) { found = true; break; }
        for (int i = 0; i < 8; i++) {
            int nx = cur.x + adx[i], ny = cur.y + ady[i];
            if (nx < 0 || nx >= cw || ny < 0 || ny >= ch) continue;
            if (cgrid[ny * cw + nx]) continue;
            float step = acost[i];
            // Clearance penalty: per visited cell, add weight × (target - clr).
            // clr is capped at target so penalty ∈ [0, weight*target]. Larger
            // weight pulls the path further from walls; weight*target must
            // exceed the cost of a 1-cell detour (1.0..1.414) to actually
            // bias A* toward corridor centrelines rather than wall tangents.
            // Defaults: target=4 cells (~0.6m at downsample=3), weight=0.5
            // → max penalty 2.0/cell, cleanly beats a 1-step detour.
            if (!clr.empty()) {
                int c = clr[ny * cw + nx];
                step += clearance_weight * float(clearance_target_cells - c);
            }
            float ng = cur.g + step;
            if (ng < best[ny * cw + nx]) {
                best[ny * cw + nx] = ng;
                prev_x[ny * cw + nx] = cur.x;
                prev_y[ny * cw + nx] = cur.y;
                float h = static_cast<float>(std::hypot(gx-nx, gy-ny));
                pq.push({nx, ny, ng, ng + h});
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

// ───────────── oriented footprint validation (Option B) ─────────────
//
// Returns true if the oriented rectangle [length × width] centred at
// (cx, cy, theta) overlaps any occupied cell (or, optionally, unknown
// cell) in the raw OccupancyGrid. Samples interior points in the body
// frame at roughly `stride_m` and tests the underlying fine-resolution
// cell. Uses the raw map WITHOUT inflation — the rectangle already gives
// the safety margin, so adding inflation would double-count and reject
// valid tight passages.
//
// `buffer_m` is a small halo expansion around the rectangle to absorb
// octomap raster noise (default 0.03 m ≈ half fine-cell at 0.05 res).
static bool footprint_clips(
    const nav_msgs::msg::OccupancyGrid &map,
    double cx, double cy, double theta,
    double length, double width,
    int occupied_thresh, bool unknown_is_obstacle,
    double stride_m, double buffer_m = 0.03)
{
    const auto &mi = map.info;
    if (mi.width < 1 || mi.height < 1 || mi.resolution <= 0.0) return false;

    double c = std::cos(theta), s = std::sin(theta);
    double half_L = 0.5 * length + buffer_m;
    double half_W = 0.5 * width  + buffer_m;

    int nu = std::max(3, static_cast<int>(std::ceil((2.0 * half_L) / std::max(stride_m, 1e-3)) + 1));
    int nv = std::max(3, static_cast<int>(std::ceil((2.0 * half_W) / std::max(stride_m, 1e-3)) + 1));
    double du = (2.0 * half_L) / (nu - 1);
    double dv = (2.0 * half_W) / (nv - 1);

    for (int iu = 0; iu < nu; iu++) {
        double u = -half_L + iu * du;
        for (int iv = 0; iv < nv; iv++) {
            double v = -half_W + iv * dv;
            double wx = cx + c * u - s * v;
            double wy = cy + s * u + c * v;
            int mx = static_cast<int>((wx - mi.origin.position.x) / mi.resolution);
            int my = static_cast<int>((wy - mi.origin.position.y) / mi.resolution);
            if (mx < 0 || mx >= static_cast<int>(mi.width) ||
                my < 0 || my >= static_cast<int>(mi.height)) {
                if (unknown_is_obstacle) return true;
                continue;
            }
            int8_t val = map.data[my * mi.width + mx];
            if (val >= occupied_thresh) return true;
            if (unknown_is_obstacle && val < 0) return true;
        }
    }
    return false;
}

// ───────────── path utilities ─────────────

// Arc-length-uniform resample of a polyline. `step_m` is the output spacing.
static std::vector<std::pair<double,double>> resample_uniform(
    const std::vector<std::pair<double,double>> &poly, double step_m) {
    std::vector<std::pair<double,double>> out;
    if (poly.size() < 2 || step_m <= 1e-6) { out = poly; return out; }

    out.push_back(poly.front());
    double carry = 0.0;
    for (size_t i = 1; i < poly.size(); i++) {
        double x0 = poly[i-1].first,  y0 = poly[i-1].second;
        double x1 = poly[i  ].first,  y1 = poly[i  ].second;
        double seg = std::hypot(x1 - x0, y1 - y0);
        if (seg < 1e-9) continue;
        double t = carry;
        while (t + step_m <= seg) {
            t += step_m;
            double s = t / seg;
            out.emplace_back(x0 + s*(x1 - x0), y0 + s*(y1 - y0));
        }
        carry = t - seg;
    }
    // always keep the final point
    if (out.back() != poly.back()) out.push_back(poly.back());
    return out;
}

// Average absolute curvature over the next `window` segments starting at
// `idx`, using three-point finite difference. Returns κ in 1/metre.
//
// Returns the MAX |κ| in the lookahead window (was AVERAGE) so a single
// sharp upcoming turn dominates the speed shaping. With the average,
// two consecutive 90° turns separated by a short straight stretch hid
// turn-2 from the controller: after turn-1 finished, the window pointed
// at the straight + start-of-turn-2, average κ stayed low, robot
// accelerated, and entered turn-2 above its lateral limit.
static double curvature_ahead(
    const std::vector<std::pair<double,double>> &path, size_t idx, int window) {
    if (path.size() < 3) return 0.0;
    double max_kappa = 0.0;
    size_t end = std::min(path.size() - 1, idx + static_cast<size_t>(window));
    for (size_t i = std::max<size_t>(idx, 1); i + 1 <= end; i++) {
        double x0 = path[i-1].first, y0 = path[i-1].second;
        double x1 = path[i  ].first, y1 = path[i  ].second;
        double x2 = path[i+1].first, y2 = path[i+1].second;
        double dx1 = x1 - x0, dy1 = y1 - y0;
        double dx2 = x2 - x1, dy2 = y2 - y1;
        double s1 = std::hypot(dx1, dy1);
        double s2 = std::hypot(dx2, dy2);
        if (s1 < 1e-6 || s2 < 1e-6) continue;
        double th1 = std::atan2(dy1, dx1);
        double th2 = std::atan2(dy2, dx2);
        double dth = wrap_angle(th2 - th1);
        double ds  = 0.5 * (s1 + s2);
        double kappa = std::abs(dth) / std::max(ds, 1e-6);
        if (kappa > max_kappa) max_kappa = kappa;
    }
    return max_kappa;
}

// Find path index whose point is closest to (x, y), starting the scan at `from`.
// Find the path waypoint nearest to (x, y). `prev_idx` is the previously
// returned cursor; we bias toward staying near it (so a tiny lateral jiggle
// doesn't snap the cursor backward), but we still search the FULL path so
// that on U-shaped or doubled-back plans the cursor can re-anchor correctly
// instead of leaping past a corner. Old "from..end only" search caused
// pure-pursuit to skip past corners on twisty A* paths and aim at lookahead
// points on the far arm of a U → robot cut across into a wall.
static size_t nearest_index(
    const std::vector<std::pair<double,double>> &path,
    double x, double y, size_t prev_idx) {
    if (path.empty()) return 0;
    if (prev_idx >= path.size()) prev_idx = path.size() - 1;
    size_t best = prev_idx;
    double bestd = std::hypot(path[prev_idx].first - x, path[prev_idx].second - y);
    // Hysteresis: a backward candidate must be closer by this margin to win.
    // Keeps the cursor monotonic under normal tracking noise, but allows a
    // genuine off-path snap-back when the geometric mismatch is real.
    constexpr double kBackHysteresis = 0.20; // m
    for (size_t i = 0; i < path.size(); i++) {
        double d = std::hypot(path[i].first - x, path[i].second - y);
        double margin = (i < prev_idx) ? kBackHysteresis : 0.0;
        if (d + margin < bestd) { bestd = d; best = i; }
    }
    return best;
}

// Advance along path from `from` until cumulative arc-length ≥ L. Returns
// that index (or path.size()-1 if end reached).
static size_t lookahead_index(
    const std::vector<std::pair<double,double>> &path,
    size_t from, double L) {
    double acc = 0.0;
    for (size_t i = from + 1; i < path.size(); i++) {
        acc += std::hypot(path[i].first  - path[i-1].first,
                          path[i].second - path[i-1].second);
        if (acc >= L) return i;
    }
    return path.empty() ? 0 : path.size() - 1;
}

// ═══════════════════════════════════════════════════════════════════════
//  Node
// ═══════════════════════════════════════════════════════════════════════

class AStarNavNode : public rclcpp::Node {
public:
    // Default `use_sim_time` to true so MuJoCo / Gazebo runs comply with
    // golden rule #1 even if a launch file forgets to pass it. Launches
    // (real-robot bringup in particular) can still flip it to false via
    // their own `parameters=[{"use_sim_time": False}]`; the launch override
    // is layered on top of this NodeOptions default.
    static rclcpp::NodeOptions make_node_options() {
        rclcpp::NodeOptions opts;
        opts.parameter_overrides({rclcpp::Parameter("use_sim_time", true)});
        return opts;
    }

    AStarNavNode() : Node("astar_nav", make_node_options()) {
        // ── speed/accel envelope ──────────────────────────────────────
        declare_parameter("max_linear_speed",  0.70);
        declare_parameter("max_angular_speed", 1.00);
        declare_parameter("min_linear_speed",  0.05);   // creep speed to keep MPC alive
        declare_parameter("linear_accel_max",  0.50);   // m/s² cap on ramping lin
        declare_parameter("linear_decel_max",  0.50);   // m/s² cap on ramping DOWN
        declare_parameter("angular_accel_max", 1.20);   // rad/s² cap on ramping ang

        declare_parameter("control_rate",      20.0);   // Hz
        declare_parameter("startup_delay",     1.0);    // s before any non-zero cmd
        declare_parameter("startup_ramp_sec",  2.0);    // s to ramp from 0→v_max after startup_delay

        // ── goal behaviour ───────────────────────────────────────────
        declare_parameter("goal_tolerance",          0.25);
        declare_parameter("goal_slow_radius",        1.5);  // start braking when within this
        declare_parameter("goal_slow_floor",         0.15); // min v when approaching goal (m/s)
        declare_parameter("goal_reached_replan_cooldown_sec", 1.0);

        // ── curvature-based shaping (the core of "slow in turns") ────
        // κ (1/m) thresholds: below `curv_full_speed_below`, full v; above
        // `curv_slow_above`, v is scaled by `curv_factor_min`. Linear interp.
        declare_parameter("curvature_lookahead_segments", 6);
        declare_parameter("curv_full_speed_below", 0.30);   // 1/m  — big radius, no slowdown
        declare_parameter("curv_slow_above",       1.50);   // 1/m  — sharp corner
        declare_parameter("curv_factor_min",       0.10);   // clamp floor

        // ── pure pursuit ─────────────────────────────────────────────
        declare_parameter("lookahead_min",  0.25);
        declare_parameter("lookahead_max",  0.80);
        declare_parameter("lookahead_gain", 0.60);          // Ld = max(min, Ld_min + gain * v)
        // Stanley-style cross-track gain. Adds a steering term proportional
        // to the robot's perpendicular distance from the path, so drift
        // off the path actively pulls back. 0.5-1.0 is typical.
        declare_parameter("cross_track_gain", 0.70);

        // ── reactive safety layer (scan) ─────────────────────────────
        declare_parameter("obstacle_slow_dist", 0.75);
        declare_parameter("obstacle_stop_dist", 0.35);
        declare_parameter("front_half_angle_deg", 35.0);
        declare_parameter("side_check_angle_deg", 60.0);
        declare_parameter("turn_in_place_on_block", true);

        // ── A* grid params ───────────────────────────────────────────
        declare_parameter("map_topic", std::string(""));
        declare_parameter("map_frame", std::string("map"));
        declare_parameter("base_frame", std::string("base_link"));
        declare_parameter("map_occupied_thresh", 50);
        declare_parameter("global_downsample",        3);
        declare_parameter("global_inflation_m",       0.18);
        declare_parameter("global_waypoint_spacing_m", 0.40); // finer than mppi: we need curvature
        declare_parameter("global_replan_sec",        1.5);
        declare_parameter("resample_step_m",          0.10);  // arc-length step for pure-pursuit input

        // A* clearance cost shaping: bias paths toward corridor centrelines
        // (away from walls) on top of the binary inflation block. Without
        // this, A* gives equal weight to "graze the inflation halo" and
        // "walk the corridor centre" — under any pose drift the body
        // rectangle then clips. target=4 cells at downsample=3 ≈ 0.60 m
        // preferred clearance from walls; weight=0.5 step-cost units per
        // missing-clearance cell → max penalty 2.0/cell, large enough to
        // beat a 1-step diagonal detour (1.414). Set weight=0 to disable.
        declare_parameter("astar_clearance_target_cells", 4);
        declare_parameter("astar_clearance_weight",       0.5);

        // ── Go2W hybrid-router hints (bias to desired mode) ──────────
        // These SHOULD match go2w_hybrid_motion.yaml on the same stack.
        declare_parameter("wheel_linear_threshold_for_bias",  0.18);
        declare_parameter("wheel_angular_threshold_for_bias", 0.30);
        declare_parameter("prefer_legs_when_curv_above", 0.90);   // 1/m
        declare_parameter("legs_bias_linear_max",        0.15);   // m/s — legacy, retained
        declare_parameter("legs_bias_angular_min",       0.35);   // rad/s — legacy, retained
        // Heading-error trigger for legged mode. When the robot's yaw
        // differs from the path tangent by more than this amount, swap
        // to CHAMP walk-while-turning (Plan B). Tighter than the old
        // curvature-only trigger, which only caught sharp path bends.
        declare_parameter("leg_heading_threshold_deg",  10.0);
        // In legged mode, linear speed is capped to this (CHAMP walks
        // at this speed while turning). 0 = true in-place pivot (old
        // behaviour). 0.15 = gentle walk while turning.
        declare_parameter("legs_mode_v_cap",             0.15);
        // Max |ω| in legged mode. Go2W tipped earlier at ω > 0.5 rad/s.
        declare_parameter("legs_mode_w_cap",             0.45);
        // Hard-gated speed policy for legged turns (degrees):
        //   |ψ_err| ≤ legs_pivot_above → walk @ legs_mode_v_cap
        //   |ψ_err| >  legs_pivot_above → v = 0, pure pivot in place
        // Prevents "walk forward while doing a 180° turn" — used to
        // drive the robot straight into the wall opposite to goal.
        declare_parameter("legs_pivot_above_deg",       30.0);
        // Minimum forward velocity (m/s) used during legged pivot.
        // CHAMP's gait library treats `cmd_vel = (0, 0, ω)` as a stand
        // request — Go2 (legged-only, mixed launch) sits there spinning
        // its setpoint without actually rotating. A small linear creep
        // keeps the gait engaged so the leg-trot trajectory is generated
        // and the body follows ω. 0.0 = legacy v=0 pivot (Go2W default,
        // hybrid_cmd_router converts pure ω to wheel differential, no
        // gait engagement needed).
        declare_parameter("legs_pivot_v_floor",          0.0);
        // Trajectory display: append only when moved this far since last
        // point. Reduces cost of publishing huge Path at 20 Hz.
        declare_parameter("trajectory_min_step_m",       0.02);
        declare_parameter("trajectory_max_points",       50000);

        // ── Option B: oriented footprint validation ──────────────────
        // After A* returns a path, verify that the robot's oriented
        // rectangle [L × W] at each pose along the path does not clip
        // any occupied cell in the RAW map (no inflation — the rect
        // already accounts for the body). If any pose clips, inject a
        // small no-go disk at the offending pose and re-plan. Up to
        // `footprint_max_retries` re-plans per tick. This catches
        // "point-passable, rectangle-not-passable" paths — the exact
        // failure mode where Go2W's 0.65 m long body hits a wall during
        // a corner turn even though the centre line stays in free space.
        declare_parameter("footprint_length",            0.65);
        declare_parameter("footprint_width",             0.45);
        declare_parameter("footprint_check_stride_m",    0.08);
        declare_parameter("footprint_retry_disk_radius_m", 0.30);
        declare_parameter("footprint_max_retries",       4);
        declare_parameter("footprint_nogo_ttl_sec",      15.0);
        // Skip validating poses within this radius of the robot — the
        // robot is already there, pushing a disk onto itself is useless.
        declare_parameter("footprint_skip_near_robot_m", 0.50);
        // Self-pose immunity radius: samples within this distance from
        // robot bypass imminent_clip rejection. Body_clip handler in
        // tick() will engage lateral escape if the pose is actually
        // overlapping. Default 0.10 m covers idx=0 + 1 resample step.
        declare_parameter("footprint_self_pose_skip_m", 0.10);
        // Small safety halo around the rectangle (metres).
        declare_parameter("footprint_buffer_m",          0.03);

        // ── topics for glue ──────────────────────────────────────────
        declare_parameter("frontier_replan_topic", std::string("/frontier_replan"));
        declare_parameter("stop_topic",            std::string("/stop"));

        load_params();

        // TF2 (used for map→base_link pose)
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Publishers (absolute names, remap-friendly — matches reactive_nav/mppi_nav)
        cmd_pub_          = create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel_stamped", 10);
        status_pub_       = create_publisher<std_msgs::msg::String>("/nav_status", 10);
        path_pub_         = create_publisher<nav_msgs::msg::Path>("/planned_path", 10);
        global_path_pub_  = create_publisher<nav_msgs::msg::Path>("/global_planned_path", 10);
        traj_pub_         = create_publisher<nav_msgs::msg::Path>("/robot_trajectory", 10);
        goal_marker_pub_  = create_publisher<visualization_msgs::msg::Marker>("/final_goal_marker", 10);
        pose_marker_pub_  = create_publisher<visualization_msgs::msg::Marker>("/robot_pose_marker", 10);
        nogo_marker_pub_  = create_publisher<visualization_msgs::msg::Marker>("/astar_nogo_disks", 10);
        replan_pub_       = create_publisher<std_msgs::msg::Empty>(
                                get_parameter("frontier_replan_topic").as_string(), 10);

        // Subscribers
        auto sensor_qos = rclcpp::SensorDataQoS();
        auto best_effort_qos = rclcpp::QoS(1)
            .reliability(rclcpp::ReliabilityPolicy::BestEffort)
            .durability(rclcpp::DurabilityPolicy::Volatile);
        auto map_qos = rclcpp::QoS(1)
            .reliability(rclcpp::ReliabilityPolicy::Reliable)
            .durability(rclcpp::DurabilityPolicy::TransientLocal);

        wp_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
            "/way_point", 10,
            [this](geometry_msgs::msg::PointStamped::SharedPtr msg) {
                double ngx = msg->point.x, ngy = msg->point.y;
                if (std::hypot(ngx - goal_x_, ngy - goal_y_) > 0.5) {
                    global_waypoints_.clear();
                    resampled_path_.clear();
                    last_pursuit_idx_ = 0;
                    last_global_plan_time_ = -1;
                }
                goal_x_ = ngx; goal_y_ = ngy; has_goal_ = true;
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
            [this](sensor_msgs::msg::LaserScan::SharedPtr msg) { last_scan_ = msg; });

        stop_sub_ = create_subscription<std_msgs::msg::Int8>(
            get_parameter("stop_topic").as_string(), 10,
            [this](std_msgs::msg::Int8::SharedPtr msg) { external_stop_ = msg->data; });

        auto map_topic_param = get_parameter("map_topic").as_string();
        if (map_topic_param.empty()) {
            map_topic_param = std::string(get_namespace()) + "/map";
            if (map_topic_param.size() >= 2 && map_topic_param[0] == '/' && map_topic_param[1] == '/')
                map_topic_param = map_topic_param.substr(1);
        }
        map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
            map_topic_param, map_qos,
            [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) { last_map_ = msg; });
        RCLCPP_INFO(get_logger(), "Subscribing to map: %s", map_topic_param.c_str());

        timer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / control_rate_),
            std::bind(&AStarNavNode::tick, this));

        RCLCPP_INFO(get_logger(),
            "A* nav started: v_max=%.2f ω_max=%.2f ctrl=%.0fHz "
            "curv_slow=[%.2f,%.2f] legs_above_κ=%.2f",
            max_v_, max_w_, control_rate_, curv_low_, curv_high_, prefer_legs_curv_);
    }

private:
    void load_params() {
        max_v_   = get_parameter("max_linear_speed").as_double();
        max_w_   = get_parameter("max_angular_speed").as_double();
        min_v_   = get_parameter("min_linear_speed").as_double();
        acc_v_   = get_parameter("linear_accel_max").as_double();
        dec_v_   = get_parameter("linear_decel_max").as_double();
        acc_w_   = get_parameter("angular_accel_max").as_double();
        control_rate_   = get_parameter("control_rate").as_double();
        startup_delay_  = get_parameter("startup_delay").as_double();
        startup_ramp_   = get_parameter("startup_ramp_sec").as_double();

        goal_tol_        = get_parameter("goal_tolerance").as_double();
        goal_slow_r_     = get_parameter("goal_slow_radius").as_double();
        goal_slow_floor_ = get_parameter("goal_slow_floor").as_double();
        goal_cooldown_   = get_parameter("goal_reached_replan_cooldown_sec").as_double();

        curv_window_     = get_parameter("curvature_lookahead_segments").as_int();
        curv_low_        = get_parameter("curv_full_speed_below").as_double();
        curv_high_       = get_parameter("curv_slow_above").as_double();
        curv_floor_      = get_parameter("curv_factor_min").as_double();

        cross_track_gain_ = get_parameter("cross_track_gain").as_double();
        Ld_min_  = get_parameter("lookahead_min").as_double();
        Ld_max_  = get_parameter("lookahead_max").as_double();
        Ld_gain_ = get_parameter("lookahead_gain").as_double();

        obs_slow_ = get_parameter("obstacle_slow_dist").as_double();
        obs_stop_ = get_parameter("obstacle_stop_dist").as_double();
        front_half_= get_parameter("front_half_angle_deg").as_double() * M_PI / 180.0;
        side_half_ = get_parameter("side_check_angle_deg").as_double() * M_PI / 180.0;
        turn_in_place_ = get_parameter("turn_in_place_on_block").as_bool();

        map_occupied_thresh_  = get_parameter("map_occupied_thresh").as_int();
        global_downsample_    = get_parameter("global_downsample").as_int();
        global_inflation_m_   = get_parameter("global_inflation_m").as_double();
        global_wp_spacing_    = get_parameter("global_waypoint_spacing_m").as_double();
        global_replan_sec_    = get_parameter("global_replan_sec").as_double();
        resample_step_        = get_parameter("resample_step_m").as_double();
        astar_clr_target_cells_ = get_parameter("astar_clearance_target_cells").as_int();
        astar_clr_weight_       = static_cast<float>(
            get_parameter("astar_clearance_weight").as_double());

        wheel_lin_thresh_ = get_parameter("wheel_linear_threshold_for_bias").as_double();
        wheel_ang_thresh_ = get_parameter("wheel_angular_threshold_for_bias").as_double();
        prefer_legs_curv_ = get_parameter("prefer_legs_when_curv_above").as_double();
        legs_bias_lin_max_= get_parameter("legs_bias_linear_max").as_double();
        legs_bias_ang_min_= get_parameter("legs_bias_angular_min").as_double();
        leg_heading_thresh_ = get_parameter("leg_heading_threshold_deg").as_double() * M_PI / 180.0;
        legs_mode_v_cap_  = get_parameter("legs_mode_v_cap").as_double();
        legs_mode_w_cap_  = get_parameter("legs_mode_w_cap").as_double();
        legs_pivot_above_ =
            get_parameter("legs_pivot_above_deg").as_double() * M_PI / 180.0;
        legs_pivot_v_floor_ = get_parameter("legs_pivot_v_floor").as_double();
        traj_min_step_    = get_parameter("trajectory_min_step_m").as_double();
        traj_max_points_  = get_parameter("trajectory_max_points").as_int();

        fp_length_          = get_parameter("footprint_length").as_double();
        fp_width_           = get_parameter("footprint_width").as_double();
        fp_stride_m_        = get_parameter("footprint_check_stride_m").as_double();
        fp_retry_radius_    = get_parameter("footprint_retry_disk_radius_m").as_double();
        fp_max_retries_     = get_parameter("footprint_max_retries").as_int();
        fp_nogo_ttl_sec_    = get_parameter("footprint_nogo_ttl_sec").as_double();
        fp_skip_near_robot_ = get_parameter("footprint_skip_near_robot_m").as_double();
        fp_self_pose_skip_  = get_parameter("footprint_self_pose_skip_m").as_double();
        fp_buffer_m_        = get_parameter("footprint_buffer_m").as_double();

        map_frame_  = get_parameter("map_frame").as_string();
        base_frame_ = get_parameter("base_frame").as_string();
    }

    // ── tick ──────────────────────────────────────────────────────────

    void tick() {
        auto now = this->now();
        double now_sec = now.seconds();
        if (start_time_ < 0) start_time_ = now_sec;
        double t_since_start = now_sec - start_time_;

        // Reset the geometric-block reason at the start of every tick.
        // Guards (imminent_clip, pivot_clips, repeated no_plan) set this
        // when they decide to refuse motion; publish_cmd uses it as the
        // nav_status `state` field at the end of the tick.
        blocked_reason_.clear();

        // Startup freeze (before ramp)
        if (t_since_start < startup_delay_) {
            publish_cmd(0.0, 0.0, "warming_up");
            return;
        }
        if (!has_goal_) {
            publish_cmd(0.0, 0.0, "idle:no_goal");
            return;
        }

        // Robot pose in map frame via TF (falls back to odom if TF lags)
        double rx = robot_x_, ry = robot_y_, ryaw = robot_yaw_;
        try {
            auto tf = tf_buffer_->lookupTransform(
                map_frame_, base_frame_, tf2::TimePointZero,
                tf2::durationFromSec(0.1));
            rx = tf.transform.translation.x;
            ry = tf.transform.translation.y;
            tf2::Quaternion q;
            tf2::fromMsg(tf.transform.rotation, q);
            double roll, pitch, yaw;
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
            ryaw = yaw;
            has_map_tf_ = true;
        } catch (tf2::TransformException &ex) {
            if (!has_map_tf_) {
                publish_cmd(0.0, 0.0, "warming_up:no_tf");
                return;
            }
            // use previous rx,ry,ryaw (from odom callback)
            (void)ex;
        }

        // ── brake-hold (post-unreachable inertia bleed) ──
        // After any goal-relative block fires, publish_cmd extends
        // brake_until_sec_ by 1.5 s. While the deadline is in the future,
        // tick() refuses to act on any new path: zero cmd_vel until inertia
        // settles and CFPA2 has had time to reassign deliberately rather
        // than reflexively. nav_status emits state="brake_hold" — neutral
        // (not "unreachable"), so CFPA2 won't fast-blacklist on it; the
        // assigned goal stays put for the duration.
        if (now_sec < brake_until_sec_) {
            publish_cmd(0.0, 0.0, "brake_hold");
            return;
        }

        double dist_to_goal = std::hypot(goal_x_ - rx, goal_y_ - ry);

        // ── goal reached ──
        if (dist_to_goal < goal_tol_) {
            publish_cmd(0.0, 0.0, "goal_reached");
            if (last_replan_time_ < 0 || (now_sec - last_replan_time_) >= goal_cooldown_) {
                replan_pub_->publish(std_msgs::msg::Empty());
                last_replan_time_ = now_sec;
            }
            return;
        }

        // ── A* global replan ──
        // Three triggers:
        //   (a) first tick / no plan yet  →  must plan
        //   (b) DYNAMIC: the next N path points now land in occupied cells
        //       of the latest OccupancyGrid — happens when LiDAR reveals
        //       new walls in what used to be unknown territory. Without
        //       this, the robot follows a stale plan into freshly-seen
        //       obstacles.
        //   (c) goal changed (handled at /way_point callback by clearing
        //       resampled_path_ + last_global_plan_time_=-1 → falls to (a))
        //
        // We DELIBERATELY do NOT replan periodically just because
        // global_replan_sec elapsed. With the cost landscape close to
        // flat between two routes to the same goal (e.g. obstacle
        // bypassable to W or E with similar lengths), each periodic
        // replan can flip A*'s tie-break, producing alternating paths
        // that start in opposite directions. The robot then turns one
        // way, immediately gets a path the other way, oscillates.
        // demo3_mixed 2026-04-25: B locked in narrow corridor, no
        // motion all session because plan flipped W/E every 1.5 s.
        // Genuine path improvements from new map data still arrive
        // via (b); committing to a chosen direction is strictly
        // safer than oscillating.
        bool need_replan = last_global_plan_time_ < 0
            || resampled_path_.size() < 2;

        // Floor so dynamic replans don't hammer A* at full 20 Hz.
        // Even a perfectly-invalid plan only needs to replan once per 200 ms.
        bool min_interval_ok = (last_global_plan_time_ < 0)
            || ((now_sec - last_global_plan_time_) >= 0.2);

        if (!need_replan && min_interval_ok && last_map_ && !resampled_path_.empty()) {
            // Check the next ~2 m of path (20 samples at 0.1 m) against
            // the latest map. If ANY point sits in an occupied cell,
            // replan immediately. Also run an oriented footprint check
            // at a coarser stride (every 4th sample) so that freshly
            // revealed walls that clip the robot's rectangle — but not
            // its centre line — also trigger replan.
            const auto &mi = last_map_->info;
            size_t start = std::min(last_pursuit_idx_, resampled_path_.size() - 1);
            size_t end = std::min(resampled_path_.size(), start + 20);
            for (size_t i = start; i < end; i++) {
                double px = resampled_path_[i].first;
                double py = resampled_path_[i].second;
                int mx = static_cast<int>((px - mi.origin.position.x) / mi.resolution);
                int my = static_cast<int>((py - mi.origin.position.y) / mi.resolution);
                if (mx < 0 || mx >= static_cast<int>(mi.width) ||
                    my < 0 || my >= static_cast<int>(mi.height)) {
                    need_replan = true;
                    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                        "A*: dynamic replan — path exits known map at idx=%zu", i);
                    break;
                }
                int8_t val = last_map_->data[my * mi.width + mx];
                // Any newly-blocked cell OR any cell that became
                // unknown-again (octomap decay edge case) invalidates
                // the stale plan. Treat both as replan triggers.
                if (val >= map_occupied_thresh_ || val < 0) {
                    need_replan = true;
                    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                        "A*: dynamic replan — path idx=%zu at (%.2f,%.2f) is now %s (val=%d)",
                        i, px, py, (val < 0 ? "unknown" : "blocked"), (int)val);
                    break;
                }

                // Oriented footprint check (every 4th sample for cost).
                // Always validates; only the response branches by distance:
                //   far  → push disk + replan
                //   near + ahead → trigger replan but DON'T push disk
                //                  (would land on/near self → poison map)
                //   near + behind → ignore (passed it)
                if ((i - start) % 4 == 0) {
                    double theta_i;
                    if (i + 1 < resampled_path_.size()) {
                        double dx = resampled_path_[i+1].first  - px;
                        double dy = resampled_path_[i+1].second - py;
                        theta_i = std::atan2(dy, dx);
                    } else if (i > 0) {
                        double dx = px - resampled_path_[i-1].first;
                        double dy = py - resampled_path_[i-1].second;
                        theta_i = std::atan2(dy, dx);
                    } else {
                        theta_i = ryaw;
                    }
                    if (footprint_clips(*last_map_, px, py, theta_i,
                                        fp_length_, fp_width_,
                                        map_occupied_thresh_,
                                        false /*don't block on unknown*/,
                                        fp_stride_m_, fp_buffer_m_)) {
                        double dx = px - rx, dy = py - ry;
                        double clip_dist = std::hypot(dx, dy);
                        bool ahead = (dx * std::cos(ryaw)
                                       + dy * std::sin(ryaw)) > 0.0;
                        // Self-pose immunity: see plan-time validation
                        // for rationale. Body_clip handler does the
                        // escape; planner-side replan-spam doesn't help
                        // and starves the body_clip detection of cycles.
                        if (i == 0 || clip_dist < fp_self_pose_skip_) {
                            continue;
                        }
                        if (clip_dist >= fp_skip_near_robot_) {
                            // Far — disk-push behavior.
                            need_replan = true;
                            persistent_nogo_disks_.push_back(
                                {px, py, fp_retry_radius_,
                                 now_sec + fp_nogo_ttl_sec_});
                            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                "A*: dynamic replan — footprint clips at "
                                "idx=%zu (%.2f,%.2f θ=%.2f) dist=%.2f m",
                                i, px, py, theta_i, clip_dist);
                            break;
                        } else if (ahead) {
                            // Near + ahead — imminent collision. Force a
                            // replan but DON'T poison the map. Main tick
                            // will fail to plan and emit zero cmd until
                            // CFPA2 picks a new goal.
                            need_replan = true;
                            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                "A*: imminent footprint clip ahead at "
                                "%.2f m (idx=%zu) — forcing replan, "
                                "not pushing disk on self", clip_dist, i);
                            break;
                        }
                        // else (near + behind/at-self) → ignore, keep scanning
                    }
                }
            }
        }

        if (need_replan && last_map_) {
            // Drop expired persistent no-go disks before building retry list.
            persistent_nogo_disks_.erase(
                std::remove_if(persistent_nogo_disks_.begin(),
                               persistent_nogo_disks_.end(),
                               [now_sec](const PersistentDisc &d) {
                                   return d.expire_sec < now_sec;
                               }),
                persistent_nogo_disks_.end());

            // ── Option B: A* + oriented footprint validation loop ──
            // Try up to fp_max_retries_ + 1 times. Each failed plan
            // injects a no-go disk at the offending pose so the next
            // A* re-routes around it. Disks persist (with TTL) so the
            // planner remembers the bad region across ticks.
            std::vector<NogoDisc> retry_disks;
            // seed with the persistent store
            retry_disks.reserve(persistent_nogo_disks_.size() + fp_max_retries_);
            for (const auto &pd : persistent_nogo_disks_) {
                retry_disks.push_back({pd.cx, pd.cy, pd.r});
            }

            AStarResult plan;
            std::vector<std::pair<double,double>> cand_resampled;
            bool plan_ok = false;

            for (int attempt = 0; attempt <= fp_max_retries_; attempt++) {
                plan = plan_astar(
                    *last_map_, rx, ry, goal_x_, goal_y_,
                    global_downsample_, global_inflation_m_,
                    map_occupied_thresh_, global_wp_spacing_,
                    true /*unknown_is_obstacle*/,
                    retry_disks,
                    astar_clr_target_cells_, astar_clr_weight_);

                if (!plan.valid || plan.waypoints.empty()) break;

                // Build a candidate resampled path and check each pose.
                std::vector<std::pair<double,double>> raw;
                raw.reserve(plan.waypoints.size() + 1);
                raw.emplace_back(rx, ry);
                for (auto &p : plan.waypoints) raw.push_back(p);
                cand_resampled = resample_uniform(raw, resample_step_);

                // ── Footprint validation: ALWAYS validate every path
                //    point. Distance-from-robot decides only the
                //    *response*, not whether to look. (Pre-2026-04-25 we
                //    skipped validation entirely within fp_skip_near_robot_m
                //    of the robot to avoid pushing disks onto self; that
                //    coupled "do we observe?" with "how do we react?" and
                //    blinded the planner to walls in the last 0.5 m of
                //    approach. demo3_mixed sw_h_2 (15 cm thick) tipped A
                //    that way: front wheel pinned the wall, rear legs kept
                //    walking, body levered up to 73° tilt.)
                //
                //    Response branches by clip distance from CURRENT robot
                //    pose:
                //      far  (≥ fp_skip_near_robot_m):
                //          push nogo disk + retry plan (existing behaviour)
                //      near (<  fp_skip_near_robot_m) AND ahead-of-heading:
                //          imminent_clip → request frontier_replan, do NOT
                //          push disk on self, fail this plan attempt
                //      near AND behind/at-current-pose:
                //          ignore — robot has already passed it
                size_t bad_idx = 0;
                bool bad_found = false;
                bool imminent_clip = false;   // near + ahead → emergency abort
                double imm_dist = 1e9;
                double cos_yaw = std::cos(ryaw);
                double sin_yaw = std::sin(ryaw);
                for (size_t i = 0; i < cand_resampled.size(); i++) {
                    double px = cand_resampled[i].first;
                    double py = cand_resampled[i].second;

                    double theta_i;
                    if (i + 1 < cand_resampled.size()) {
                        double dxp = cand_resampled[i+1].first  - px;
                        double dyp = cand_resampled[i+1].second - py;
                        theta_i = std::atan2(dyp, dxp);
                    } else if (i > 0) {
                        double dxp = px - cand_resampled[i-1].first;
                        double dyp = py - cand_resampled[i-1].second;
                        theta_i = std::atan2(dyp, dxp);
                    } else {
                        theta_i = ryaw;
                    }

                    if (!footprint_clips(*last_map_, px, py, theta_i,
                                         fp_length_, fp_width_,
                                         map_occupied_thresh_,
                                         false /*don't block on unknown*/,
                                         fp_stride_m_, fp_buffer_m_))
                        continue;

                    double dx = px - rx, dy = py - ry;
                    double clip_dist = std::hypot(dx, dy);
                    bool ahead = (dx * cos_yaw + dy * sin_yaw) > 0.0;

                    // ── self-pose immunity ─────────────────────────
                    // Samples within fp_self_pose_skip_ of the robot are
                    // physically AT the robot's pose. If they clip, the
                    // body itself overlaps an inflation/wall cell — that's
                    // body_clip territory, handled later by tick's lateral-
                    // escape primitive (CHAMP strafe). Treating it as
                    // imminent_clip refuses every plan the moment the body
                    // grazes a halo, freezing the robot on the wall edge
                    // forever. This is the spawn-disk principle extended
                    // to every tick: planner-side belief about inflation
                    // can't override the physics fact that the robot is
                    // already at this pose. demo3_mixed 2026-04-25 zigzag_3:
                    // A drifted 5–10 cm into halo, every plan rejected,
                    // brake_hold + imminent_clip looped for 100+ s.
                    if (i == 0 || clip_dist < fp_self_pose_skip_) {
                        continue;
                    }
                    if (clip_dist < fp_skip_near_robot_) {
                        if (ahead && clip_dist < imm_dist) {
                            imminent_clip = true;
                            imm_dist = clip_dist;
                        }
                        // near + behind/current → ignore; near + ahead is
                        // recorded but we keep scanning so a far clip on
                        // the same path can still get the disk treatment
                        // for next attempt.
                        continue;
                    }

                    // Far clip — first one wins as the disk site.
                    bad_idx = i;
                    bad_found = true;
                    break;
                }

                if (imminent_clip) {
                    // Don't push a disk (would land within the body or
                    // just ahead of it, poisoning all future plans). Bail
                    // this attempt; nav_status reports state="unreachable"
                    // with reason="imminent_clip", which is what CFPA2's
                    // fast-blacklist consumes — single source of truth.
                    // (We deliberately do NOT publish frontier_replan here
                    // anymore: doing so on every imminent_clip tick fired
                    // CFPA2 reassignment ~1 Hz, faster than the robot
                    // could decelerate, producing the 13 s / 12-goal flap
                    // that drove A into zigzag_3 north wall on
                    // 2026-04-25. Brake-hold + nav_status now do the
                    // job with hysteresis baked in.)
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                        "A*: imminent footprint clip ahead at %.2f m "
                        "— aborting plan (state=unreachable, brake-hold "
                        "engaged; near=%.2f m of robot)",
                        imm_dist, fp_skip_near_robot_);
                    blocked_reason_ = "blocked:imminent_clip";
                    break;  // exit retry loop; plan_ok stays false
                }

                if (!bad_found) { plan_ok = true; break; }

                // Far clip path — existing disk-push retry behaviour.
                double bx = cand_resampled[bad_idx].first;
                double by = cand_resampled[bad_idx].second;
                retry_disks.push_back({bx, by, fp_retry_radius_});
                persistent_nogo_disks_.push_back(
                    {bx, by, fp_retry_radius_, now_sec + fp_nogo_ttl_sec_});
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                    "A*: footprint clips at idx=%zu (%.2f,%.2f) — "
                    "attempt=%d/%d, injecting nogo disk r=%.2f",
                    bad_idx, bx, by, attempt + 1, fp_max_retries_ + 1,
                    fp_retry_radius_);
            }

            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                "A*: rx=%.2f,%.2f goal=%.2f,%.2f dist=%.2f valid=%d npts=%zu "
                "map=%dx%d res=%.2f origin=(%.2f,%.2f) fp_ok=%d disks=%zu",
                rx, ry, goal_x_, goal_y_, dist_to_goal,
                plan.valid ? 1 : 0, plan.waypoints.size(),
                (int)last_map_->info.width, (int)last_map_->info.height,
                last_map_->info.resolution,
                last_map_->info.origin.position.x, last_map_->info.origin.position.y,
                plan_ok ? 1 : 0, persistent_nogo_disks_.size());

            if (plan_ok && !cand_resampled.empty()) {
                resampled_path_ = std::move(cand_resampled);
                global_waypoints_ = plan.waypoints;
                last_pursuit_idx_ = 0;
                last_global_plan_time_ = now_sec;
                consecutive_plan_failures_ = 0;
                publish_global_path();
            } else {
                last_global_plan_time_ = now_sec;
                consecutive_plan_failures_++;
                // Option B exhausted retries. Persistent disks may have
                // poisoned every route to the goal; sticking with the
                // stale path while disks live for fp_nogo_ttl_sec wedges
                // pure-pursuit on an anchored-to-old-pose plan.
                // After 2+ consecutive failures, dump the disks AND the
                // stale path so the next tick replans fresh from the
                // current pose. Robot stops ("no_plan") in the meantime
                // — far safer than tracking a stale tangent into a wall.
                if (consecutive_plan_failures_ >= 2) {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                        "A*: %d consecutive plan failures — dropping %zu "
                        "persistent nogo disks and stale path; will replan "
                        "fresh from current pose next tick",
                        consecutive_plan_failures_,
                        persistent_nogo_disks_.size());
                    persistent_nogo_disks_.clear();
                    resampled_path_.clear();
                    global_waypoints_.clear();
                }
            }

            publish_nogo_markers();
        } else if (need_replan && !last_map_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "A*: replan requested but no /map received yet");
        }

        if (resampled_path_.size() < 2) {
            // Persistent no-plan → escalate the nav_status string so
            // CFPA2's stall blacklist (which keys off the string) treats
            // it as "this goal is unreachable from here, please blacklist
            // and reassign". Threshold of 5 ticks (= 250 ms at 20 Hz)
            // filters out transient single-tick A* misses while still
            // catching the wedged-against-obstacle case quickly.
            if (consecutive_plan_failures_ >= 5) {
                blocked_reason_ = "blocked:no_plan_repeated";
                publish_cmd(0.0, 0.0, "no_plan_repeated");
            } else {
                publish_cmd(0.0, 0.0, "no_plan");
            }
            return;
        }

        // ── scan safety ──
        ScanMetrics sm;
        bool have_scan = static_cast<bool>(last_scan_);
        if (have_scan)
            sm = analyze_scan(*last_scan_, front_half_, side_half_, obs_slow_);

        // ── pure pursuit + cross-track feedback ──
        size_t near = nearest_index(resampled_path_, rx, ry, last_pursuit_idx_);
        last_pursuit_idx_ = near;
        double Ld = std::clamp(Ld_min_ + Ld_gain_ * last_v_cmd_, Ld_min_, Ld_max_);
        size_t la = lookahead_index(resampled_path_, near, Ld);

        double tx = resampled_path_[la].first;
        double ty = resampled_path_[la].second;
        double dx = tx - rx, dy = ty - ry;
        // transform to robot frame
        double cy_ = std::cos(ryaw), sy_ = std::sin(ryaw);
        double lx =  cy_ * dx + sy_ * dy;
        double ly = -sy_ * dx + cy_ * dy;
        double Ld_actual = std::hypot(lx, ly);
        double alpha = std::atan2(ly, lx);           // angle to lookahead
        double kappa_cmd = (Ld_actual > 1e-3)
            ? 2.0 * std::sin(alpha) / Ld_actual : 0.0;

        // ── cross-track error (signed perpendicular distance to path) ──
        // Take the path segment near `near` as local path direction, then
        // compute how far the robot is from that line. Positive = robot
        // is to the LEFT of the path (needs right steer).
        double cross_track_err = 0.0;
        double path_yaw = ryaw;
        {
            size_t i0 = near;
            size_t i1 = std::min(near + 1, resampled_path_.size() - 1);
            if (i1 > i0) {
                double px0 = resampled_path_[i0].first;
                double py0 = resampled_path_[i0].second;
                double px1 = resampled_path_[i1].first;
                double py1 = resampled_path_[i1].second;
                double sx = px1 - px0, sy = py1 - py0;
                double seg = std::hypot(sx, sy);
                if (seg > 1e-6) {
                    // Signed perpendicular: cross product / |seg|.
                    cross_track_err = ((rx - px0) * sy - (ry - py0) * sx) / seg;
                    path_yaw = std::atan2(sy, sx);
                }
            }
        }
        double heading_err = wrap_angle(path_yaw - ryaw);

        // ── curvature ahead (from path, not cmd) ──
        double kappa_path = curvature_ahead(resampled_path_, near, curv_window_);

        // ── speed shaping ──
        double v_target = max_v_;
        // Body-frame lateral velocity target. Default 0; non-zero only
        // when pivot-relief or body_clip-escape primitives commandeer
        // motion to slide laterally to a recoverable pose. Rate-limited
        // alongside v_target → produces v_y_cmd for publish_cmd.
        double v_y_target = 0.0;

        // 1) smooth startup ramp (0 → 1 over startup_ramp_sec)
        double tr = t_since_start - startup_delay_;
        double f_start = (startup_ramp_ > 1e-3) ? clamp01(tr / startup_ramp_) : 1.0;
        v_target *= f_start;

        // 2) goal approach braking: inside goal_slow_r_, ramp down to goal_slow_floor_
        if (dist_to_goal < goal_slow_r_) {
            double s = clamp01(dist_to_goal / std::max(goal_slow_r_, 1e-3));
            v_target = std::max(goal_slow_floor_, v_target * s);
        }

        // 3) curvature-aware slow in corners
        double f_curv = ramp(kappa_path, curv_low_, curv_high_, 1.0, curv_floor_);
        v_target *= f_curv;

        // 4) scan-based slow
        if (have_scan && sm.min_front < obs_slow_) {
            double rng_val = obs_slow_ - obs_stop_;
            double f_scan = (rng_val > 1e-6)
                ? clamp01((sm.min_front - obs_stop_) / rng_val) : 0.0;
            v_target *= f_scan;
        }

        // angular command = pure-pursuit feed-forward + Stanley-style
        // cross-track correction. Pure pursuit alone treats path as a
        // stream of lookahead waypoints and will cut corners / drift off
        // if wheel slip or rate limits push the robot laterally; adding
        // cross-track makes the controller actively pull back onto the
        // path centerline. heading_err corrects orientation drift.
        double w_pp = v_target * kappa_cmd;  // pure-pursuit geometry
        // Stanley-ish: steer = heading_err + atan(k_ct * e_ct / v_ref)
        double v_ref = std::max(v_target, 0.10);
        double w_stanley = cross_track_gain_ * heading_err
                           + std::atan2(cross_track_gain_ * cross_track_err, v_ref);
        // Combine — pure-pursuit for smooth path-following + stanley for
        // error-correcting pull-back. Weight by how far off path we are.
        double ct_abs = std::abs(cross_track_err);
        double blend = clamp01(ct_abs / 0.30);  // 0=on path → PP; 1=30 cm off → Stanley
        double w_target = (1.0 - blend) * w_pp + blend * w_stanley;

        // ── turn-mode dispatch (Plan B: legged-while-walking) ──
        // Decide wheel vs legged based on HEADING ERROR vs planned path
        // tangent, not just curvature. Small drift → wheels keep cruising;
        // drift grows past the threshold → CHAMP takes over for the turn.
        //
        //   |heading_err| > leg_heading_thresh  OR  κ_path >= prefer_legs_curv
        //     → LEGGED mode, v capped to legs_mode_v_cap (walking speed).
        //       KEY CHANGE: v is NOT forced to 0 — CHAMP walks while turning.
        //       This avoids the old stop-rotate-go cycle and lets the robot
        //       round corners without losing momentum.
        //   else
        //     → WHEEL mode, v floored above wheel_lin_thresh so the router
        //       picks wheel drive.
        bool want_legs = (std::abs(heading_err) > leg_heading_thresh_)
                         || (kappa_path >= prefer_legs_curv_);

        if (!want_legs && relief_active_) {
            // No longer in pivot-mode at all — pivot must have resolved
            // (heading aligned with path). Clear the relief tracker.
            relief_active_ = false;
            relief_distance_m_ = 0.0;
        }
        if (want_legs) {
            // LEGGED-turn speed policy (hard-gated):
            //   |ψ_err| ≤ legs_pivot_above  → walk @ legs_mode_v_cap
            //   |ψ_err| >  legs_pivot_above → v = 0, pure in-place pivot
            //
            // Rationale: CHAMP walks in body X. If the path goes the
            // opposite way (|ψ_err| large), walking forward at 0.15 m/s
            // drives AWAY from the path for several seconds until the
            // turn completes — the exact "robot ignores path, walks
            // into wall" failure mode. Above the threshold we pivot
            // in place first, then resume walking once heading is close
            // enough to the path tangent.
            double abs_he = std::abs(heading_err);
            if (abs_he > legs_pivot_above_) {
                // Pure-pivot mode. CHAMP's gait library treats
                // (lin.x = 0, ω ≠ 0) as a stand command and never engages
                // the leg trot — so the angular setpoint is silently
                // ignored on legged-only robots. legs_pivot_v_floor_ is a
                // tiny forward creep (e.g. 0.05 m/s) that keeps gait
                // engaged so ω actually executes. 0.0 = legacy hard pivot.
                v_target = legs_pivot_v_floor_;
            } else {
                v_target = std::min(v_target, legs_mode_v_cap_);
            }

            // Ensure ω is above the wheel-angular threshold in the correct
            // direction so router commits to legged (otherwise a small
            // heading correction could read as wheel mode). For a big
            // heading error, always use sign(heading_err) — not w_pp /
            // w_stanley, which can point the wrong way near α = ±π where
            // sin(α) ≈ 0.
            double sign_w;
            if (std::abs(heading_err) > 1e-3) sign_w = (heading_err > 0) ? 1.0 : -1.0;
            else sign_w = (w_target >= 0) ? 1.0 : -1.0;
            double min_mag = wheel_ang_thresh_ + 0.05;  // just above threshold
            if (std::abs(w_target) < min_mag) w_target = sign_w * min_mag;
            if (abs_he > legs_pivot_above_) {
                w_target = sign_w * std::min(std::abs(w_target), legs_mode_w_cap_);
            }
            // Cap |ω| to avoid tip risk (earlier flips came from high ω).
            w_target = std::clamp(w_target, -legs_mode_w_cap_, legs_mode_w_cap_);

            // ── Pure-pivot footprint check ────────────────────────────
            // In pure-pivot mode, the body rectangle sweeps through
            // [ryaw, ryaw + heading_err] while the position stays at
            // (rx, ry). Path-sample validation doesn't catch this
            // because no path point covers a rotation about a fixed
            // centre. Without the check, A's hip pinned on a corner +
            // commanded ω = 0.35 → cantilever flip in 0.25 s
            // (demo3_mixed t=199.7s, FL_hip × zigzag_1 → tilt 59°→154°).
            //
            // Walk N intermediate yaw samples between current and target
            // and run the same footprint_clips() against the raw map.
            // If any clip → kill the pivot (v=ω=0) and request a fresh
            // goal that doesn't demand this rotation.
            if (abs_he > legs_pivot_above_ && last_map_) {
                // Sweep [ryaw, ryaw + heading_err] in N+1 samples.
                constexpr int kPivotSweepN = 5;      // 5 samples ≈ 6° resolution at 30°
                bool pivot_clips = false;
                double clip_at_deg = 0.0;
                for (int k = 0; k <= kPivotSweepN; ++k) {
                    double t = static_cast<double>(k) / kPivotSweepN;
                    double th = ryaw + t * heading_err;
                    if (footprint_clips(*last_map_, rx, ry, th,
                                        fp_length_, fp_width_,
                                        map_occupied_thresh_,
                                        false /*don't block on unknown*/,
                                        fp_stride_m_, fp_buffer_m_)) {
                        pivot_clips = true;
                        clip_at_deg = (t * heading_err) * 180.0 / M_PI;
                        break;
                    }
                }
                if (pivot_clips) {
                    // ── PIVOT-RELIEF primitive ────────────────────────
                    // Old behavior: refuse pivot (v=ω=0), let CFPA2
                    // reassign. But the underlying geometry doesn't
                    // change — same robot, same nearby walls. CFPA2
                    // keeps trying angles, none are pivot-safe at
                    // the current pose, so the robot is stuck OR (if
                    // the corridor was barely passable) drifts and
                    // ends up CLIMBING something while waiting → tip.
                    //
                    // New behavior: the geometry is wrong only at
                    // CURRENT pose. Search for a nearby pose where
                    // the SAME pivot through the SAME yaw range IS
                    // safe. If found: command translation toward it
                    // (no rotation, hold heading), reach the
                    // pivot-safe pose, then resume normal pivot —
                    // sweep will pass at that pose.
                    //
                    // 8 body-frame directions at kReliefDist, in
                    // priority order (forward = continue along the
                    // path; lateral = sidestep into open space;
                    // backward = retreat). Each candidate must pass
                    // BOTH (a) translation footprint clear along the
                    // line, AND (b) full pivot-sweep clear at the
                    // candidate pose.
                    constexpr double kReliefDist     = 0.30;
                    constexpr double kReliefSpeed    = 0.10;
                    constexpr int    kReliefNTrans   = 4;
                    constexpr int    kReliefNPivot   = 5;
                    struct ReliefDir { double bx, by; const char *name; };
                    static const ReliefDir kReliefDirs[] = {
                        { 1.0,    0.0,    "forward"},
                        { 0.707, -0.707,  "front-right"},
                        { 0.707,  0.707,  "front-left"},
                        { 0.0,   -1.0,    "right"},
                        { 0.0,    1.0,    "left"},
                        {-0.707, -0.707,  "back-right"},
                        {-0.707,  0.707,  "back-left"},
                        {-1.0,    0.0,    "backward"},
                    };
                    const double cos_y = std::cos(ryaw);
                    const double sin_y = std::sin(ryaw);
                    const char  *relief_name = nullptr;
                    double relief_bx = 0.0, relief_by = 0.0;
                    for (const auto &dir : kReliefDirs) {
                        // (a) translation footprint clear along the line
                        bool t_clear = true;
                        for (int k = 1; k <= kReliefNTrans && t_clear; ++k) {
                            const double t = static_cast<double>(k) / kReliefNTrans;
                            const double dx_b = dir.bx * kReliefDist * t;
                            const double dy_b = dir.by * kReliefDist * t;
                            const double sx = rx + cos_y * dx_b - sin_y * dy_b;
                            const double sy = ry + sin_y * dx_b + cos_y * dy_b;
                            if (footprint_clips(*last_map_, sx, sy, ryaw,
                                                fp_length_, fp_width_,
                                                map_occupied_thresh_,
                                                false, fp_stride_m_,
                                                fp_buffer_m_)) {
                                t_clear = false;
                            }
                        }
                        if (!t_clear) continue;

                        // (b) full pivot through heading_err clear at candidate
                        const double cand_x = rx + cos_y * dir.bx * kReliefDist
                                                  - sin_y * dir.by * kReliefDist;
                        const double cand_y = ry + sin_y * dir.bx * kReliefDist
                                                  + cos_y * dir.by * kReliefDist;
                        bool p_clear = true;
                        for (int k = 0; k <= kReliefNPivot && p_clear; ++k) {
                            const double t = static_cast<double>(k) / kReliefNPivot;
                            const double th = ryaw + t * heading_err;
                            if (footprint_clips(*last_map_, cand_x, cand_y, th,
                                                fp_length_, fp_width_,
                                                map_occupied_thresh_,
                                                false, fp_stride_m_,
                                                fp_buffer_m_)) {
                                p_clear = false;
                            }
                        }
                        if (!p_clear) continue;

                        relief_bx = dir.bx;
                        relief_by = dir.by;
                        relief_name = dir.name;
                        break;
                    }

                    if (relief_name) {
                        // Cumulative-distance budget: track how far we've
                        // moved while in relief mode. Without a cap,
                        // 38+ consecutive 0.3 m commands wandered A
                        // 1.3 m straight into zigzag_1 — each individual
                        // step footprint-clear, cumulative path crossed
                        // the wall's inflation halo. After
                        // kReliefMaxCumulM the search is clearly leading
                        // the robot deeper into clutter, not out of it.
                        constexpr double kReliefMaxCumulM = 0.50;
                        if (relief_active_) {
                            relief_distance_m_ +=
                                std::hypot(rx - relief_last_x_,
                                           ry - relief_last_y_);
                        } else {
                            relief_distance_m_ = 0.0;
                            relief_active_ = true;
                        }
                        relief_last_x_ = rx;
                        relief_last_y_ = ry;

                        if (relief_distance_m_ > kReliefMaxCumulM) {
                            // Budget exhausted — abort relief, freeze.
                            v_target   = 0.0;
                            v_y_target = 0.0;
                            w_target   = 0.0;
                            relief_active_ = false;
                            relief_distance_m_ = 0.0;
                            blocked_reason_ = "blocked:pivot_unsafe";
                            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                "A*: pivot relief abandoned after %.2f m of "
                                "cumulative motion without finding a pivot-"
                                "safe pose. Wedged in clutter; holding still.",
                                kReliefMaxCumulM);
                        } else {
                            // Translate toward pivot-safe pose at low speed,
                            // hold heading. Once we arrive, next tick's
                            // pivot_sweep will pass and normal pivot resumes.
                            v_target   = relief_bx * kReliefSpeed;
                            v_y_target = relief_by * kReliefSpeed;
                            w_target   = 0.0;
                            blocked_reason_ = std::string("blocked:pivot_relief:") +
                                              relief_name;
                            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                "A*: pivot would clip at Δyaw=%.1f° "
                                "(heading_err=%.1f°) — relief: translating %s "
                                "(v=%.2f,%.2f), cumul=%.2f m / %.2f m budget.",
                                clip_at_deg, heading_err * 180.0 / M_PI,
                                relief_name, v_target, v_y_target,
                                relief_distance_m_, kReliefMaxCumulM);
                        }
                    } else {
                        // No relief direction works — really stuck.
                        v_target   = 0.0;
                        v_y_target = 0.0;
                        w_target   = 0.0;
                        blocked_reason_ = "blocked:pivot_unsafe";
                        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                            "A*: pivot blocked at Δyaw=%.1f° AND no relief "
                            "direction within %.2f m — robot wedged. "
                            "Holding still.",
                            clip_at_deg, kReliefDist);
                    }
                }
            }
        } else if (dist_to_goal > goal_slow_r_ * 0.5) {
            // WHEEL: keep v above wheel_lin_thresh so router picks wheel.
            double v_floor = wheel_lin_thresh_ + 0.02;
            if (v_target < v_floor) v_target = v_floor;
        }

        // ── reverse-for-space maneuver (DISABLED) ──
        // Originally: if |α| > π/4 AND scan_front < 0.55 m, back up at
        // 0.12 m/s steering opposite the desired turn, release at 0.80 m
        // clearance. Idea was to give tight turns more room. In practice
        // the reverse+opposite-steer combo was triggering too often near
        // walls and occasionally flipping the robot when the router
        // switched to legged mid-backup. Keeping the code in place but
        // gated off — re-enable by flipping kReverseEnabled to true if
        // we later want to experiment.
        constexpr bool kReverseEnabled = false;
        if (kReverseEnabled) {
            static constexpr double kReverseTriggerDist  = 0.55; // m
            static constexpr double kReverseReleaseDist  = 0.80; // m
            static constexpr double kReverseAlphaMin     = M_PI / 4.0; // 45°
            static constexpr double kReverseSpeed        = 0.12; // m/s (back)
            static constexpr double kReverseAngMag       = 0.25; // rad/s

            bool front_tight = have_scan && sm.min_front < kReverseTriggerDist;
            bool need_space  = std::abs(alpha) > kReverseAlphaMin
                               || kappa_path > prefer_legs_curv_;

            if (reversing_) {
                if (have_scan && sm.min_front >= kReverseReleaseDist) {
                    reversing_ = false;
                }
            } else if (front_tight && need_space && dist_to_goal > goal_slow_r_) {
                reversing_ = true;
            }

            if (reversing_) {
                v_target = -kReverseSpeed;
                double sign_w = (alpha > 0) ? -1.0 : 1.0;
                w_target = sign_w * kReverseAngMag;
            }
        } else {
            reversing_ = false;  // keep state clean while feature is off
        }

        // ── emergency stop ──
        if (have_scan) {
            bool blocked = sm.min_front < obs_stop_;
            if (blocked && !reversing_) {
                // Only stop if we're NOT already handling it with reverse.
                v_target = 0.0;
                if (turn_in_place_ && external_stop_ == 0) {
                    if (std::abs(w_target) < 0.1)
                        w_target = max_w_ * 0.5
                                   * ((sm.right_push > sm.left_push) ? 1.0 : -1.0);
                } else {
                    w_target = 0.0;
                }
                // Don't publish frontier_replan here: emergency-stop is a
                // transient scan event; nav_status will already show the
                // stalled v=0 / ω≠0 state and CFPA2's stall blacklist
                // takes care of persistent cases. Reflex replan-pub here
                // contributed to the goal-flap loop.
            } else if (external_stop_ != 0) {
                v_target = 0.0;
                w_target = 0.0;
            }
        }

        // ── hard saturate ──
        // Allow negative v during the reverse-for-space maneuver. Forward
        // cap stays at max_v_; reverse speed is inherently small (set by
        // kReverseSpeed).
        double v_lo = reversing_ ? -std::abs(max_v_) * 0.5 : 0.0;
        v_target = std::clamp(v_target, v_lo, max_v_);
        w_target = std::clamp(w_target, -max_w_, max_w_);

        // ── rate limit (finite accel) ──
        // Asymmetric: decel cap can be larger than accel cap. Tipping is
        // caused by high v·ω, not high |dv/dt|, so a hard brake is safer
        // than a slow one — especially when the path suddenly flips 180°
        // and the controller needs to stop before pivoting.
        double dt = 1.0 / control_rate_;
        double dv_pos = acc_v_ * dt;   // limit on speeding up / reversing acceleration
        double dv_neg = dec_v_ * dt;   // limit on slowing down (magnitude)
        double dw_max = acc_w_ * dt;
        double dv_want = v_target - last_v_cmd_;
        bool is_decel = (dv_want < 0 && last_v_cmd_ > 0)
                        || (dv_want > 0 && last_v_cmd_ < 0);
        double dv_cap = is_decel ? dv_neg : dv_pos;
        double v_cmd = std::clamp(dv_want, -dv_cap, dv_cap) + last_v_cmd_;
        double w_cmd = std::clamp(w_target - last_w_cmd_, -dw_max, dw_max) + last_w_cmd_;
        // Lateral velocity tracks v_y_target with the same accel cap as v
        // (sym­metric — both come from the same translation primitive).
        // last_v_y_cmd_ defaults to 0 in normal nav so dv_want collapses
        // to v_y_target.
        double dv_y_want = v_y_target - last_v_y_cmd_;
        bool is_y_decel = (dv_y_want < 0 && last_v_y_cmd_ > 0)
                          || (dv_y_want > 0 && last_v_y_cmd_ < 0);
        double dv_y_cap = is_y_decel ? dv_neg : dv_pos;
        double v_y_cmd = std::clamp(dv_y_want, -dv_y_cap, dv_y_cap) + last_v_y_cmd_;

        // Below min_v, keep sending min_v with whatever ω — prevents CHAMP
        // from getting literally zero cmd_vel while we intend to creep.
        if (v_target > 1e-3 && v_cmd < min_v_) v_cmd = min_v_;

        // ── Execute-time footprint guard (the dual of plan-time check) ──
        // Plan-time validation (in plan_astar's retry loop and the dynamic
        // re-check at the top of tick()) tests footprint_clips at PATH
        // SAMPLES, asking "is the path I intend to follow safe?" — assumes
        // the robot tracks the centerline perfectly. Reality: pure-pursuit
        // + Stanley + leg-gait swing introduce 10-20 cm of lateral drift,
        // so the body can be off-centerline at execution time. With
        // inflation 0.25 m and body half-width 0.225 m, the on-paper
        // body-to-wall margin is only 25 mm — drift of that magnitude
        // pushes the body across the wall surface. Pre-2026-04-25, A
        // racked up 4500+ contact ticks against ne_div_h_east this way:
        // path was provably safe, drift was not modelled.
        //
        // Same function, different input: validate the rectangle at the
        // ROBOT'S ACTUAL CURRENT POSE (rx, ry, ryaw). If it clips, refuse
        // any motion this tick and tell CFPA2 the goal can't be safely
        // held — fast-blacklist via state="unreachable" / reason=body_clip.
        //
        // Skip during the first `startup_ramp_sec` so an octomap not-yet-
        // settled-around-spawn doesn't false-fire.
        bool body_clip = false;
        if (last_map_ && t_since_start > (startup_delay_ + startup_ramp_)) {
            body_clip = footprint_clips(*last_map_, rx, ry, ryaw,
                                        fp_length_, fp_width_,
                                        map_occupied_thresh_,
                                        false /*don't block on unknown*/,
                                        fp_stride_m_, fp_buffer_m_);
        }
        if (body_clip) {
            // Escape primitive: instead of just freezing (v=ω=0), try to
            // slide OUT of the wedged pose. The robot is in body-clip
            // exactly when its rectangle overlaps an occupied cell — but
            // a small translation in some BODY-frame direction may be
            // fully clear (think: head against wall A, tail against wall
            // B in a narrow corridor — forward and backward both hit
            // something, but body's left/right is open).
            //
            // We try 8 body-frame directions in priority order. For each
            // candidate, validate the body rectangle along the straight
            // line out to kEscDist. First fully-clear direction wins;
            // we command motion in that direction (in body frame, which
            // becomes cmd_vel.linear.x for forward/back and linear.y
            // for strafe). CHAMP supports omnidirectional gait → this
            // works as a crab-walk in legged mode. Heading is held so
            // the body translates without rotating (no pivot_unsafe risk).
            constexpr double kEscDist     = 0.30;
            constexpr double kEscSpeed    = 0.10;
            constexpr int    kEscNSamples = 4;
            struct EscapeDir { double bx, by; const char *name; };
            static const EscapeDir kEscapeDirs[] = {
                {-1.0,    0.0,    "backward"},
                {-0.707,  0.707,  "back-left"},
                {-0.707, -0.707,  "back-right"},
                { 0.0,    1.0,    "left"},
                { 0.0,   -1.0,    "right"},
                { 0.707,  0.707,  "front-left"},
                { 0.707, -0.707,  "front-right"},
                { 1.0,    0.0,    "forward"},
            };
            const double cy_w = std::cos(ryaw), sy_w = std::sin(ryaw);
            const char *escape_name = nullptr;
            double esc_bx = 0.0, esc_by = 0.0;
            for (const auto &dir : kEscapeDirs) {
                bool clear = true;
                for (int k = 1; k <= kEscNSamples && clear; ++k) {
                    const double t = static_cast<double>(k) / kEscNSamples;
                    const double dx_b = dir.bx * kEscDist * t;
                    const double dy_b = dir.by * kEscDist * t;
                    const double sx = rx + cy_w * dx_b - sy_w * dy_b;
                    const double sy = ry + sy_w * dx_b + cy_w * dy_b;
                    if (footprint_clips(*last_map_, sx, sy, ryaw,
                                        fp_length_, fp_width_,
                                        map_occupied_thresh_,
                                        false /*don't block on unknown*/,
                                        fp_stride_m_, fp_buffer_m_)) {
                        clear = false;
                    }
                }
                if (clear) {
                    esc_bx = dir.bx;
                    esc_by = dir.by;
                    escape_name = dir.name;
                    break;
                }
            }

            if (escape_name) {
                v_cmd   = esc_bx * kEscSpeed;        // body-frame x → linear.x
                v_y_cmd = esc_by * kEscSpeed;        // body-frame y → linear.y
                w_cmd   = 0.0;                       // hold heading
                blocked_reason_ = std::string("blocked:body_clip_escape:") +
                                  escape_name;
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                    "A*: body_clip at (%.2f,%.2f yaw=%.2f) — escaping %s "
                    "(v=%.2f,%.2f). CHAMP strafe gait should slide body "
                    "out of clipping pose; replan resumes when clear.",
                    rx, ry, ryaw, escape_name, v_cmd, v_y_cmd);
            } else {
                v_cmd   = 0.0;
                v_y_cmd = 0.0;
                w_cmd   = 0.0;
                blocked_reason_ = "blocked:body_clip";
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                    "A*: body_clip at (%.2f,%.2f yaw=%.2f) AND no escape "
                    "direction found within %.2f m — robot fully wedged. "
                    "Manual intervention required.",
                    rx, ry, ryaw, kEscDist);
            }
            // NOTE: deliberately NOT publishing frontier_replan here.
            // body_clip is POSE-relative ("the place I'm standing has a
            // body-overlap"), not GOAL-relative ("the goal is bad").
            // Asking for a new goal poisons GOAL space (CFPA2 fast-
            // blacklists each goal that astar reports unreachable from
            // a stuck pose; with the body still in contact, every new
            // goal also fails → CFPA2 cycles through the entire reachable
            // frontier set and ends in [hold/cfpa2_blacklisted_stop]).
            // 2026-04-25 demo3_mixed: B sat 525 s producing 134 NEW
            // GOAL events, all fast-blacklisted — exactly that cascade.
            //
            // Instead, publish_cmd routes "blocked:body_clip*" to a
            // distinct nav_status state ("stuck") that CFPA2 does NOT
            // fast-blacklist on. CFPA2 keeps trying goals; whenever it
            // happens to pick a goal whose first path step moves AWAY
            // from the contact direction, astar's path can succeed and
            // the robot escapes. No goal-space poisoning.
        }

        last_v_cmd_ = v_cmd;
        last_w_cmd_ = w_cmd;
        last_v_y_cmd_ = v_y_cmd;

        // If a geometric guard tripped this tick (and consequently zeroed
        // v_cmd / w_cmd), report the specific reason. CFPA2's stall
        // blacklist consumes the nav_status string and only kicks in
        // after `local_nav_stall_blacklist_sec` of stalled state — so
        // until we surface "blocked:*" here, the upstream allocator
        // thought the robot was still navigating and never reassigned.
        std::string mode_hint;
        if (!blocked_reason_.empty()) {
            mode_hint = blocked_reason_;
        } else {
            mode_hint = (kappa_path > prefer_legs_curv_)
                ? "navigating:legs_bias"
                : ((v_cmd >= wheel_lin_thresh_ && std::abs(w_cmd) <= wheel_ang_thresh_)
                      ? "navigating:wheel_pref" : "navigating:legged_pref");
        }
        publish_cmd(v_cmd, w_cmd, mode_hint, v_y_cmd);

        // visualisation — trajectory grows per distance moved (not per
        // tick) so RViz can show the full trip since boot without
        // blowing up on a 20 Hz Path publish. Only appends when the
        // robot has moved > traj_min_step_ m since the last sample.
        publish_local_path(near, la);
        {
            bool append = trajectory_.empty();
            if (!append) {
                auto &last_pt = trajectory_.back();
                append = std::hypot(rx - last_pt.first, ry - last_pt.second)
                         > traj_min_step_;
            }
            if (append) {
                trajectory_.emplace_back(rx, ry);
                if ((int)trajectory_.size() > traj_max_points_)
                    trajectory_.erase(trajectory_.begin());
            }
        }
        publish_trajectory();
        publish_goal_marker();
        publish_pose_marker(rx, ry, ryaw);
    }

    // ── publishers ────────────────────────────────────────────────────

    void publish_cmd(double lin, double ang, const std::string &state,
                     double lin_y = 0.0) {
        geometry_msgs::msg::TwistStamped msg;
        msg.header.stamp = this->now();
        msg.twist.linear.x  = lin;
        // Lateral velocity (body-frame y, i.e. strafe) is non-zero ONLY
        // during the body_clip escape primitive, which uses CHAMP's
        // crab-walk gait to slide laterally out of a wedged-in-corridor
        // pose. The hybrid_cmd_router (Go2W) forwards the full Twist
        // when in legged mode, so .y reaches CHAMP unchanged; Go2 talks
        // to CHAMP directly. lin_y also nudges the router's mode-select
        // toward legged (lateral motion can't go through wheels), so we
        // don't have to fight the router about routing during escape.
        msg.twist.linear.y  = lin_y;
        msg.twist.angular.z = ang;
        cmd_pub_->publish(msg);

        // Build nav_status/v1 JSON. CFPA2 has TWO consumption paths off
        // this string:
        //   1. Fast-blacklist (~200 ms): triggers iff state=="unreachable"
        //      OR state=="failed", AND the JSON's `goal` field matches
        //      what CFPA2 last assigned. Blacklists the goal for
        //      cfpa2_fast_unreachable_blacklist_sec, reassigns next tick.
        //   2. Legacy stall (~45 s): heuristic over status freshness.
        //
        // When a geometric guard tripped this tick (blocked_reason_ set),
        // we route through path 1 by emitting state="unreachable" with
        // `reason` = the specific guard. Otherwise the regular
        // "navigating:..." / "no_plan" / "warming_up" / etc. strings
        // pass through unchanged (path 2 alone applies).
        //
        // `goal` is always included when we have one — CFPA2 uses it
        // to match against last_goal so stale status from a previous
        // assignment doesn't blacklist the wrong frontier.
        std::string out_state = state;
        std::string reason;
        if (!blocked_reason_.empty()) {
            // blocked_reason_ format: "blocked:<tag>" — strip the prefix
            // for the `reason` field. State routing depends on the tag:
            //
            //   imminent_clip / pivot_unsafe / no_plan_repeated  →
            //     state="unreachable" — these are GOAL-relative failures
            //     (the path forward / required rotation / anywhere-from-
            //     here-to-goal is blocked). CFPA2's fast-blacklist
            //     (~200 ms) is the right reaction; the goal IS bad from
            //     this pose, get a new one.
            //
            //   body_clip  →  state="stuck" — POSE-relative failure (the
            //     robot's current pose has body-vs-obstacle overlap).
            //     The goal isn't bad; the robot just can't move from
            //     here. fast-blacklisting goals on this signal cascades
            //     until every reachable goal is poisoned and CFPA2
            //     enters [hold/cfpa2_blacklisted_stop]. CFPA2 does NOT
            //     fast-blacklist on "stuck", so goal space stays clean
            //     and CFPA2 can keep trying directions until one works.
            const std::string prefix = "blocked:";
            const std::string tag =
                (blocked_reason_.rfind(prefix, 0) == 0)
                    ? blocked_reason_.substr(prefix.size())
                    : blocked_reason_;
            reason = tag;
            // Pose-relative blocks (don't poison goal space — route to
            // "stuck" so CFPA2 doesn't fast-blacklist the assigned goal):
            //   body_clip / body_clip_escape:* — body in clipping pose
            //   pivot_relief:*                 — actively translating
            //                                    to a pivot-safe pose
            // Goal-relative blocks route to "unreachable" (CFPA2 fast-
            // blacklists current goal, picks a different one):
            //   imminent_clip / pivot_unsafe / no_plan_repeated
            const bool is_pose_relative =
                (tag == "body_clip") ||
                (tag.rfind("body_clip_escape", 0) == 0) ||
                (tag.rfind("pivot_relief", 0) == 0);
            out_state = is_pose_relative ? "stuck" : "unreachable";

            // Goal-relative block → arm 1.5 s brake. Next tick(s) will
            // short-circuit to zero cmd_vel until the deadline, regardless
            // of whether CFPA2 has reassigned in the meantime. This is the
            // anti-flap that prevents inertia + champ residual from
            // carrying the robot into walls when goal-space rapidly
            // toggles direction (the 2026-04-25 zigzag_3 impact mode).
            if (out_state == "unreachable") {
                static constexpr double kBrakeHoldSec = 1.5;
                double now_s = this->now().seconds();
                double new_until = now_s + kBrakeHoldSec;
                if (new_until > brake_until_sec_) brake_until_sec_ = new_until;
            }
        }

        std::ostringstream oss;
        oss << "{\"schema\":\"nav_status/v1\",\"source\":\"astar_nav\","
            << "\"state\":\"" << out_state << "\","
            << "\"v\":" << lin << ",\"w\":" << ang;
        if (has_goal_) {
            oss << ",\"goal\":[" << goal_x_ << "," << goal_y_ << "]";
        }
        if (!reason.empty()) {
            oss << ",\"reason\":\"" << reason << "\"";
        }
        oss << "}";

        std_msgs::msg::String s;
        s.data = oss.str();
        status_pub_->publish(s);
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
            ps.pose.orientation.w = 1.0;
            msg.poses.push_back(ps);
        }
        global_path_pub_->publish(msg);
    }

    // Publish the FULL resampled path from the current nearest-waypoint
    // to the end — not just the pure-pursuit lookahead window. (`from`
    // and `to` kept for future partial-display use.)  RViz's default
    // /robot/planned_path display now shows the whole route to goal,
    // which is what operators usually want to see.
    void publish_local_path(size_t from, size_t /*to*/) {
        nav_msgs::msg::Path msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = map_frame_;
        size_t start = std::min(from, resampled_path_.empty()
                                      ? size_t{0}
                                      : resampled_path_.size() - 1);
        for (size_t i = start; i < resampled_path_.size(); i++) {
            geometry_msgs::msg::PoseStamped ps;
            ps.header = msg.header;
            ps.pose.position.x = resampled_path_[i].first;
            ps.pose.position.y = resampled_path_[i].second;
            ps.pose.orientation.w = 1.0;
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
            ps.pose.orientation.w = 1.0;
            msg.poses.push_back(ps);
        }
        traj_pub_->publish(msg);
    }

    void publish_goal_marker() {
        visualization_msgs::msg::Marker m;
        m.header.stamp = this->now();
        m.header.frame_id = map_frame_;
        m.ns = "astar_nav"; m.id = 0;
        m.type = visualization_msgs::msg::Marker::SPHERE;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.position.x = goal_x_;
        m.pose.position.y = goal_y_;
        m.pose.position.z = 0.15;
        m.pose.orientation.w = 1.0;
        m.scale.x = m.scale.y = m.scale.z = 0.25;
        m.color.r = 1.0; m.color.g = 0.2; m.color.b = 0.2; m.color.a = 0.9;
        goal_marker_pub_->publish(m);
    }

    void publish_nogo_markers() {
        // Single SPHERE_LIST marker carrying all active persistent nogo
        // disks. Cheap on RViz. Empty list → DELETE action (clear).
        visualization_msgs::msg::Marker m;
        m.header.stamp = this->now();
        m.header.frame_id = map_frame_;
        m.ns = "astar_nogo"; m.id = 0;
        m.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        if (persistent_nogo_disks_.empty()) {
            m.action = visualization_msgs::msg::Marker::DELETE;
            nogo_marker_pub_->publish(m);
            return;
        }
        m.action = visualization_msgs::msg::Marker::ADD;
        // Uniform scale — render every disk as a 2·r sphere.
        // (If radii differ across disks we'd need one marker each; for
        // Option B all disks are the same size so this is fine.)
        double r = fp_retry_radius_;
        m.scale.x = 2.0 * r; m.scale.y = 2.0 * r; m.scale.z = 0.05;
        m.color.r = 1.0; m.color.g = 0.1; m.color.b = 0.1; m.color.a = 0.55;
        m.pose.orientation.w = 1.0;
        for (const auto &d : persistent_nogo_disks_) {
            geometry_msgs::msg::Point pt;
            pt.x = d.cx; pt.y = d.cy; pt.z = 0.05;
            m.points.push_back(pt);
        }
        nogo_marker_pub_->publish(m);
    }

    void publish_pose_marker(double rx, double ry, double ryaw) {
        visualization_msgs::msg::Marker m;
        m.header.stamp = this->now();
        m.header.frame_id = map_frame_;
        m.ns = "astar_nav"; m.id = 1;
        m.type = visualization_msgs::msg::Marker::ARROW;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.position.x = rx;
        m.pose.position.y = ry;
        m.pose.position.z = 0.15;
        tf2::Quaternion q; q.setRPY(0, 0, ryaw);
        m.pose.orientation = tf2::toMsg(q);
        m.scale.x = 0.4; m.scale.y = 0.06; m.scale.z = 0.06;
        m.color.r = 0.2; m.color.g = 0.8; m.color.b = 0.2; m.color.a = 0.9;
        pose_marker_pub_->publish(m);
    }

    // ── members ───────────────────────────────────────────────────────

    // params
    double max_v_, max_w_, min_v_, acc_v_, dec_v_, acc_w_;
    double control_rate_, startup_delay_, startup_ramp_;
    double goal_tol_, goal_slow_r_, goal_slow_floor_, goal_cooldown_;
    int    curv_window_;
    double curv_low_, curv_high_, curv_floor_;
    double Ld_min_, Ld_max_, Ld_gain_;
    double cross_track_gain_;
    double obs_slow_, obs_stop_, front_half_, side_half_;
    bool   turn_in_place_;
    int    map_occupied_thresh_, global_downsample_;
    double global_inflation_m_, global_wp_spacing_, global_replan_sec_, resample_step_;
    int   astar_clr_target_cells_ = 0;
    float astar_clr_weight_       = 0.0f;
    double wheel_lin_thresh_, wheel_ang_thresh_, prefer_legs_curv_;
    double legs_bias_lin_max_, legs_bias_ang_min_;
    double leg_heading_thresh_;      // rad (loaded as deg → rad)
    double legs_mode_v_cap_;         // m/s
    double legs_mode_w_cap_;         // rad/s
    double legs_pivot_above_;        // rad — pivot-in-place above this
    double legs_pivot_v_floor_;      // m/s — creep during pivot (CHAMP gait engagement)
    double traj_min_step_;           // m — trajectory downsample step
    int    traj_max_points_;

    // Option B footprint validation params
    double fp_length_, fp_width_;
    double fp_stride_m_;
    double fp_retry_radius_;
    int    fp_max_retries_;
    double fp_nogo_ttl_sec_;
    double fp_skip_near_robot_;
    double fp_self_pose_skip_ = 0.10;
    double fp_buffer_m_;

    std::string map_frame_;
    std::string base_frame_;

    // state
    double robot_x_ = 0, robot_y_ = 0, robot_yaw_ = 0;
    double goal_x_ = 0, goal_y_ = 0;
    bool   has_goal_ = false;
    bool   has_map_tf_ = false;
    double start_time_ = -1.0;
    double last_global_plan_time_ = -1.0;
    double last_replan_time_ = -1.0;
    double last_v_cmd_ = 0.0, last_w_cmd_ = 0.0;
    // Lateral velocity (body-frame y, used by pivot-relief and
    // body_clip-escape primitives; 0 in normal navigation).
    double last_v_y_cmd_ = 0.0;

    // Cumulative distance in pivot-relief mode without exiting (i.e.
    // pivot_sweep at current pose still fails). Reset whenever pivot
    // is no longer needed or normal nav succeeds. If this exceeds a
    // budget, relief is aborted — the robot has been wandering for
    // too long without finding a pivot-safe pose, indicating the
    // search is leading it deeper into clutter rather than out of it.
    // Without this cap, 38+ consecutive 0.3 m relief commands led A
    // through 1.3 m of motion straight into zigzag_1 (each individual
    // 0.3 m line clear, cumulative path crossed the wall's inflation
    // halo).
    double relief_distance_m_ = 0.0;
    double relief_last_x_ = 0.0;
    double relief_last_y_ = 0.0;
    bool   relief_active_ = false;
    int8_t external_stop_ = 0;
    bool reversing_ = false;  // latched during reverse-for-space maneuver
    int  consecutive_plan_failures_ = 0;  // Option B exhaust counter

    // Brake-hold deadline (unix sec). Set whenever a goal-relative block
    // (imminent_clip / pivot_unsafe / no_plan_repeated) routes nav_status
    // to "unreachable". While brake is active, tick() short-circuits to
    // publish_cmd(0,0,0) — bleeds off champ-gait + body inertia before any
    // new path can drive the robot. Without this, CFPA2's fast-blacklist
    // (~200 ms) reassigns mid-momentum: astar starts tracking a path 90°
    // away from current heading while the robot is still coasting in the
    // old direction. Demo3_mixed 2026-04-25: 13 s window with 12 goal
    // switches across NE/S/SW/W; A coasted 5 m southward into zigzag_3.
    double brake_until_sec_ = -1.0;

    // When the planner refuses to issue motion because a geometric guard
    // tripped (imminent footprint clip ahead, pure-pivot would clip,
    // repeated A* failures), this string holds a SHORT reason tag like
    // "blocked:imminent_clip". `publish_cmd` uses it as the nav_status
    // `state` field instead of the usual "navigating:..." mode hint, so
    // CFPA2 (which already runs a stall blacklist on the nav_status
    // string after `local_nav_stall_blacklist_sec` of stalled state) can
    // tell the difference between "robot still trying" and "robot has
    // given up; please assign a different goal". Reset to empty at the
    // start of every tick; written by the guard branches; read by
    // publish_cmd at the end. Without this, B sat 525 s on an
    // unreachable goal in 2026-04-25 demo3_mixed because every
    // pivot_blocked tick still reported nav_status="navigating:leg".
    std::string blocked_reason_;

    std::vector<std::pair<double,double>> global_waypoints_;
    std::vector<std::pair<double,double>> resampled_path_;
    size_t last_pursuit_idx_ = 0;
    std::vector<std::pair<double,double>> trajectory_;

    // Persistent no-go disks (Option B). Each entry is (cx, cy, r, expire_sec).
    // Pushed when footprint validation fails at a pose; decays after
    // fp_nogo_ttl_sec so transient map artifacts don't permanently
    // poison the route. Passed into every plan_astar call so that A*
    // learns from recent execution-layer failures.
    struct PersistentDisc { double cx, cy, r, expire_sec; };
    std::vector<PersistentDisc> persistent_nogo_disks_;

    sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
    nav_msgs::msg::OccupancyGrid::SharedPtr last_map_;

    // ROS
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_, global_path_pub_, traj_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_marker_pub_, pose_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr nogo_marker_pub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr replan_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr wp_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr stop_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AStarNavNode>());
    rclcpp::shutdown();
    return 0;
}
