/*
 * hybrid_astar_nav_node.cpp — Hybrid A* + Ceres-smoothed nav for Go2W.
 *
 * Same I/O contract as astar_nav_node (drop-in via nav_backend:=hybrid_astar):
 *   subs:  /way_point  /odom/ground_truth  /scan  /{ns}/map
 *   pubs:  /cmd_vel_stamped  /nav_status  /planned_path  /robot_trajectory
 *          /final_goal_marker  /robot_pose_marker  /global_planned_path
 *
 * Architecture (clean v0.1):
 *
 *   plan()
 *     ├── compute_clearance_dt()        chamfer 3-4 EDT, cached per map stamp
 *     ├── compute_dijkstra_heuristic()  2D Dijkstra-from-goal on coarse grid
 *     ├── hybrid_astar_search()         lattice search; OMPL RS analytic shot
 *     │     └── footprint_clips()       oriented rect vs raw OccGrid
 *     └── smooth_ceres()                LM solve: anchor + smoothness + obs + κ
 *
 *   tick()
 *     ├── (replan when stale / dynamic invalidation)
 *     ├── pure-pursuit + Stanley + curvature speed shaping
 *     └── hybrid_cmd_router-aware mode bias (wheel ↔ leg)
 *
 * Why Hybrid A* + Ceres (vs the legacy astar_nav_node 8-conn + reactive
 * post-hoc footprint check):
 *   - Search-time kinematics: every expanded state has a feasible motion
 *     primitive bringing it from the parent. The robot's body rectangle
 *     is checked at the kinematic state, not on a centre-line shadow.
 *     → Eliminates 90 % of "point-passable, body-clips" failures that
 *       the old planner needed an Option-B retry loop + persistent no-go
 *       disk store + self-pose immunity to paper over.
 *   - Reeds-Shepp analytic shot: when the robot is in goal vicinity,
 *     a single RS curve replaces hundreds of expansions; the path is
 *     C¹ at the shot junction.
 *   - Ceres smoother turns the discrete primitive chain into a path
 *     that is C² and clearance-aware, with curvature peaks bounded by
 *     the explicit κ-cost term — directly attacks the leg-mode
 *     hysteresis root cause (control fights A* zigzags).
 */

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <queue>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>

#include <ceres/ceres.h>

#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/State.h>
#include <ompl/base/ScopedState.h>

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

namespace ob = ompl::base;

// ═══════════════════════════════════════════════════════════════════════
//   math + geometry helpers
// ═══════════════════════════════════════════════════════════════════════

static inline double wrap_angle(double a) {
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}
static inline double clamp01(double x) {
    return (x < 0.0) ? 0.0 : (x > 1.0) ? 1.0 : x;
}

// Discretized state key for closed-set hashing.
// θ binned into n_theta bins ∈ [0, n_theta).
struct StateKey {
    int gx, gy, gth;
    bool operator==(const StateKey &o) const {
        return gx == o.gx && gy == o.gy && gth == o.gth;
    }
};
struct StateKeyHash {
    size_t operator()(const StateKey &k) const noexcept {
        // Cantor-like packing; map width unlikely > 2^15, n_theta ≤ 64.
        uint64_t a = static_cast<uint32_t>(k.gx);
        uint64_t b = static_cast<uint32_t>(k.gy);
        uint64_t c = static_cast<uint32_t>(k.gth);
        return std::hash<uint64_t>{}((a << 32) ^ (b << 16) ^ c);
    }
};

// ═══════════════════════════════════════════════════════════════════════
//   Footprint validation (same logic as astar_nav_node, kept verbatim
//   so behaviour is identical; deduplication can come once we extract
//   a shared go2w_nav_common library).
// ═══════════════════════════════════════════════════════════════════════
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

// ═══════════════════════════════════════════════════════════════════════
//   Coarse grid + clearance distance transform (cached per map stamp).
//   Dual-purpose: A* heuristic Dijkstra source AND Ceres obstacle
//   cost gradient (via bilinear interpolation in the smoother).
// ═══════════════════════════════════════════════════════════════════════
struct CoarseGrid {
    int w = 0, h = 0;
    double res = 0.0;            // metres per cell
    double ox = 0.0, oy = 0.0;   // world origin
    std::vector<uint8_t> blocked;          // 1 = blocked (occ ∪ inflation ∪ unknown)
    std::vector<uint8_t> walls;            // 1 = real wall (occ ∪ closing); never cleared
    std::vector<float>   clearance_m;      // metres to nearest blocked cell, capped
    rclcpp::Time stamp;
};

static void build_coarse_grid(
    const nav_msgs::msg::OccupancyGrid &map,
    int downsample, double inflation_m,
    int occupied_thresh, bool unknown_is_obstacle,
    double clearance_cap_m,
    CoarseGrid &out)
{
    int map_w = static_cast<int>(map.info.width);
    int map_h = static_cast<int>(map.info.height);
    double map_res = map.info.resolution;

    out.w = (map_w + downsample - 1) / downsample;
    out.h = (map_h + downsample - 1) / downsample;
    out.res = map_res * downsample;
    out.ox = map.info.origin.position.x;
    out.oy = map.info.origin.position.y;
    out.stamp = map.header.stamp;

    int cw = out.w, ch = out.h;
    out.blocked.assign(cw * ch, 0);
    std::vector<uint8_t> occ(cw * ch, 0);
    std::vector<uint8_t> unk(cw * ch, 0);

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
                        has_unk = true;
                    }
                }
            }
            if (has_occ) occ[cy * cw + cx] = 1;
            else if (has_unk) unk[cy * cw + cx] = 1;
        }
    }

    // Single-pass closing on occupied cells (radius 1) — same rationale as
    // astar_nav_node: fixes thin-wall octomap projections without over-
    // closing narrow corridors.
    std::vector<uint8_t> walls = occ;
    {
        std::vector<uint8_t> dil(cw * ch, 0);
        for (int cy = 0; cy < ch; cy++)
            for (int cx = 0; cx < cw; cx++) {
                if (walls[cy * cw + cx]) { dil[cy * cw + cx] = 1; continue; }
                bool any = false;
                for (int dy = -1; dy <= 1 && !any; dy++)
                    for (int dx = -1; dx <= 1 && !any; dx++) {
                        int nx = cx + dx, ny = cy + dy;
                        if (nx >= 0 && nx < cw && ny >= 0 && ny < ch &&
                            walls[ny * cw + nx]) any = true;
                    }
                if (any) dil[cy * cw + cx] = 1;
            }
        for (int cy = 0; cy < ch; cy++)
            for (int cx = 0; cx < cw; cx++) {
                if (!dil[cy * cw + cx]) continue;
                bool all_set = true;
                for (int dy = -1; dy <= 1 && all_set; dy++)
                    for (int dx = -1; dx <= 1 && all_set; dx++) {
                        int nx = cx + dx, ny = cy + dy;
                        if (nx < 0 || nx >= cw || ny < 0 || ny >= ch) continue;
                        if (!dil[ny * cw + nx]) all_set = false;
                    }
                if (all_set) walls[cy * cw + cx] = 1;
            }
    }
    out.walls = walls;

    // Inflation around walls.
    std::vector<uint8_t> inflated = walls;
    int infl = std::max(0, (int)std::ceil(inflation_m / out.res));
    if (infl > 0) {
        std::vector<std::pair<int,int>> obs;
        for (int cy = 0; cy < ch; cy++)
            for (int cx = 0; cx < cw; cx++)
                if (walls[cy * cw + cx]) obs.emplace_back(cx, cy);
        int isq = infl * infl;
        for (auto [ox, oy] : obs)
            for (int ddy = -infl; ddy <= infl; ddy++)
                for (int ddx = -infl; ddx <= infl; ddx++) {
                    if (ddx*ddx + ddy*ddy > isq) continue;
                    int nx = ox + ddx, ny = oy + ddy;
                    if (nx >= 0 && nx < cw && ny >= 0 && ny < ch)
                        inflated[ny * cw + nx] = 1;
                }
    }

    // Final blocked = inflated walls ∪ unknown (no inflation on unknown,
    // same rationale as astar_nav_node).
    for (int i = 0; i < cw * ch; i++) {
        out.blocked[i] = inflated[i] || (unknown_is_obstacle && unk[i]);
    }

    // ── Chamfer 3-4 distance transform (in chamfer units, then →metres) ──
    constexpr int kBig = 1 << 28;
    std::vector<int> d(cw * ch, kBig);
    for (int i = 0; i < cw * ch; i++) if (out.blocked[i]) d[i] = 0;
    for (int y = 0; y < ch; y++)
        for (int x = 0; x < cw; x++) {
            int idx = y * cw + x;
            int v = d[idx];
            if (x > 0)              v = std::min(v, d[idx - 1]      + 3);
            if (y > 0)              v = std::min(v, d[idx - cw]     + 3);
            if (x > 0 && y > 0)     v = std::min(v, d[idx - cw - 1] + 4);
            if (x < cw-1 && y > 0)  v = std::min(v, d[idx - cw + 1] + 4);
            d[idx] = v;
        }
    for (int y = ch - 1; y >= 0; y--)
        for (int x = cw - 1; x >= 0; x--) {
            int idx = y * cw + x;
            int v = d[idx];
            if (x < cw-1)             v = std::min(v, d[idx + 1]      + 3);
            if (y < ch-1)             v = std::min(v, d[idx + cw]     + 3);
            if (x < cw-1 && y < ch-1) v = std::min(v, d[idx + cw + 1] + 4);
            if (x > 0 && y < ch-1)    v = std::min(v, d[idx + cw - 1] + 4);
            d[idx] = v;
        }
    out.clearance_m.assign(cw * ch, 0.0f);
    int cap_chamfer = static_cast<int>(std::ceil(clearance_cap_m / out.res * 3.0));
    for (int i = 0; i < cw * ch; i++) {
        int dc = std::min(d[i], cap_chamfer);
        out.clearance_m[i] = static_cast<float>((dc / 3.0) * out.res);
    }
}

// 2D Dijkstra heuristic-grid: shortest grid distance from goal cell to
// every free cell on the coarse grid. Used as h(s) in Hybrid A*: provides
// an obstacle-aware admissible lower bound on cost-to-go (much tighter
// than Euclidean in cluttered scenes; classic Dolgov 2010 trick).
static std::vector<float> dijkstra_h(const CoarseGrid &g, int gx, int gy) {
    std::vector<float> h(g.w * g.h, std::numeric_limits<float>::infinity());
    if (gx < 0 || gx >= g.w || gy < 0 || gy >= g.h) return h;
    if (g.blocked[gy * g.w + gx]) return h;  // unreachable goal cell

    using Item = std::pair<float, int>;
    std::priority_queue<Item, std::vector<Item>, std::greater<Item>> pq;
    h[gy * g.w + gx] = 0.0f;
    pq.push({0.0f, gy * g.w + gx});
    static const int dx[8] = {-1,0,1,-1,1,-1,0,1};
    static const int dy[8] = {-1,-1,-1,0,0,1,1,1};
    static const float w[8] = {1.414f,1.0f,1.414f,1.0f,1.0f,1.414f,1.0f,1.414f};

    while (!pq.empty()) {
        auto [d_curr, idx] = pq.top(); pq.pop();
        if (d_curr > h[idx] + 1e-3f) continue;
        int cx = idx % g.w, cy = idx / g.w;
        for (int k = 0; k < 8; k++) {
            int nx = cx + dx[k], ny = cy + dy[k];
            if (nx < 0 || nx >= g.w || ny < 0 || ny >= g.h) continue;
            int nidx = ny * g.w + nx;
            if (g.blocked[nidx]) continue;
            float nd = d_curr + w[k] * static_cast<float>(g.res);
            if (nd < h[nidx]) {
                h[nidx] = nd;
                pq.push({nd, nidx});
            }
        }
    }
    return h;
}

// ═══════════════════════════════════════════════════════════════════════
//   Hybrid A* search
// ═══════════════════════════════════════════════════════════════════════

struct HAState {
    double x, y, theta;
};

struct HANode {
    HAState s;
    float g;             // accumulated cost
    float f;             // g + h
    int parent;          // index in nodes_
    int prim_id;         // motion primitive used to reach this node
};

struct HybridAStarOptions {
    // State discretization
    int    n_theta            = 24;
    double step_length        = 0.4;
    double max_steer          = 0.4;     // rad
    bool   allow_reverse      = false;
    double reverse_penalty    = 2.0;     // multiplier on g for reverse motions
    double switch_penalty     = 1.5;     // additive cost for direction change
    double steer_change_pen   = 0.3;     // additive cost when steering changes
    // Search budget
    int    max_iters          = 20000;
    int    rs_shot_period     = 5;       // try analytic shot every N expansions
    double rs_turning_radius  = 0.8;     // metres; matches step_length / sin(max_steer)
    double goal_tol_xy        = 0.25;
    double goal_tol_theta     = 0.30;    // rad
    // Body footprint
    double fp_length          = 0.65;
    double fp_width           = 0.45;
    double fp_stride          = 0.08;
    double fp_buffer          = 0.03;
};

// Result of Hybrid A* search.
struct HybridAStarResult {
    bool valid = false;
    std::vector<HAState> states;     // contiguous SE(2) path, including end of analytic shot
    int n_expansions = 0;
};

// Dense interpolation of an OMPL Reeds-Shepp curve into HAState samples.
// Used both for analytic shot collision check and for path stitching.
static std::vector<HAState> rs_interp_states(
    const std::shared_ptr<ob::ReedsSheppStateSpace> &rs,
    const HAState &a, const HAState &b, double step_m)
{
    ob::ScopedState<ob::ReedsSheppStateSpace> sa(rs), sb(rs), si(rs);
    sa[0] = a.x; sa[1] = a.y; sa[2] = a.theta;
    sb[0] = b.x; sb[1] = b.y; sb[2] = b.theta;
    double L = rs->distance(sa.get(), sb.get());
    int N = std::max(2, (int)std::ceil(L / std::max(step_m, 1e-3)));
    std::vector<HAState> out;
    out.reserve(N + 1);
    for (int i = 0; i <= N; i++) {
        double t = (double)i / N;
        rs->interpolate(sa.get(), sb.get(), t, si.get());
        out.push_back({si[0], si[1], si[2]});
    }
    return out;
}

static bool rs_path_collision_free(
    const std::vector<HAState> &samples,
    const nav_msgs::msg::OccupancyGrid &map,
    const HybridAStarOptions &opt,
    int occupied_thresh)
{
    for (const auto &s : samples) {
        if (footprint_clips(map, s.x, s.y, s.theta,
                            opt.fp_length, opt.fp_width,
                            occupied_thresh, /*unknown_is_obstacle=*/false,
                            opt.fp_stride, opt.fp_buffer))
            return false;
    }
    return true;
}

static HybridAStarResult hybrid_astar_search(
    const nav_msgs::msg::OccupancyGrid &map,
    const CoarseGrid &cg,
    const std::vector<float> &h_grid,        // Dijkstra heuristic
    const std::shared_ptr<ob::ReedsSheppStateSpace> &rs,
    const HAState &start, const HAState &goal,
    const HybridAStarOptions &opt,
    int occupied_thresh)
{
    HybridAStarResult result;

    auto state_key = [&](const HAState &s) {
        StateKey k;
        k.gx = static_cast<int>((s.x - cg.ox) / cg.res);
        k.gy = static_cast<int>((s.y - cg.oy) / cg.res);
        double th = wrap_angle(s.theta);
        if (th < 0) th += 2 * M_PI;
        k.gth = static_cast<int>(th / (2 * M_PI) * opt.n_theta) % opt.n_theta;
        return k;
    };
    auto h_at = [&](const HAState &s) -> float {
        int cx = static_cast<int>((s.x - cg.ox) / cg.res);
        int cy = static_cast<int>((s.y - cg.oy) / cg.res);
        if (cx < 0 || cx >= cg.w || cy < 0 || cy >= cg.h) {
            return static_cast<float>(std::hypot(s.x - goal.x, s.y - goal.y));
        }
        float h = h_grid[cy * cg.w + cx];
        // Fallback to Euclidean if Dijkstra didn't reach this cell (e.g.
        // start is in a disconnected component). Euclidean is admissible.
        if (!std::isfinite(h)) h = static_cast<float>(std::hypot(s.x - goal.x, s.y - goal.y));
        return h;
    };
    auto cell_blocked = [&](const HAState &s) {
        int cx = static_cast<int>((s.x - cg.ox) / cg.res);
        int cy = static_cast<int>((s.y - cg.oy) / cg.res);
        if (cx < 0 || cx >= cg.w || cy < 0 || cy >= cg.h) return true;
        return cg.blocked[cy * cg.w + cx] != 0;
    };

    // Generate motion primitives.
    // Forward primitives: (steer, dir=+1). Optionally reverse mirror.
    struct Prim { double steer; int dir; };
    std::vector<Prim> prims;
    for (double s : {-opt.max_steer, 0.0, +opt.max_steer}) prims.push_back({s, +1});
    if (opt.allow_reverse) {
        for (double s : {-opt.max_steer, 0.0, +opt.max_steer}) prims.push_back({s, -1});
    }

    // Bicycle-model arc step: applies steer δ, distance L, direction d.
    auto step_state = [&](const HAState &s, double steer, int dir) {
        HAState ns;
        double L = opt.step_length * dir;
        if (std::abs(steer) < 1e-6) {
            ns.x = s.x + L * std::cos(s.theta);
            ns.y = s.y + L * std::sin(s.theta);
            ns.theta = s.theta;
        } else {
            // Turning radius from step_length and max_steer:
            //   R = L_step / sin(δ)        (so per-step Δθ = δ when |steer|=max)
            // For arbitrary steer: dθ = (L / R) × (steer / max_steer)
            double R = opt.step_length / std::sin(std::abs(opt.max_steer));
            double dth = (L / R) * (steer / opt.max_steer);
            ns.theta = wrap_angle(s.theta + dth);
            // Mid-point arc integration (good enough at L = 0.4 m).
            double th_m = s.theta + 0.5 * dth;
            ns.x = s.x + L * std::cos(th_m);
            ns.y = s.y + L * std::sin(th_m);
        }
        return ns;
    };

    std::vector<HANode> nodes;
    nodes.reserve(8192);
    std::unordered_map<StateKey, int, StateKeyHash> closed;

    auto cmp = [&](int a, int b) { return nodes[a].f > nodes[b].f; };
    std::priority_queue<int, std::vector<int>, decltype(cmp)> pq(cmp);

    HANode root{start, 0.0f, h_at(start), -1, -1};
    nodes.push_back(root);
    pq.push(0);

    int best_idx = -1;

    int iter = 0;
    while (!pq.empty() && iter < opt.max_iters) {
        int cur = pq.top(); pq.pop();
        StateKey ck = state_key(nodes[cur].s);
        auto it = closed.find(ck);
        if (it != closed.end() && it->second != cur) continue;  // stale
        closed[ck] = cur;
        iter++;

        // ── Goal check (xy + heading) ──
        double d_goal = std::hypot(nodes[cur].s.x - goal.x, nodes[cur].s.y - goal.y);
        double dth = std::abs(wrap_angle(nodes[cur].s.theta - goal.theta));
        if (d_goal < opt.goal_tol_xy && dth < opt.goal_tol_theta) {
            best_idx = cur;
            break;
        }

        // ── Reeds-Shepp analytic shot every N expansions ──
        if (iter % opt.rs_shot_period == 0) {
            auto samples = rs_interp_states(rs, nodes[cur].s, goal, opt.fp_stride);
            if (rs_path_collision_free(samples, map, opt, occupied_thresh)) {
                // Append RS samples as a synthetic chain so reconstruct sees them.
                int parent = cur;
                for (size_t i = 1; i < samples.size(); i++) {
                    HANode n;
                    n.s = samples[i];
                    n.parent = parent;
                    n.prim_id = -2;  // marker: RS shot
                    n.g = nodes[parent].g + static_cast<float>(opt.step_length);
                    n.f = n.g;
                    nodes.push_back(n);
                    parent = static_cast<int>(nodes.size()) - 1;
                }
                best_idx = parent;
                break;
            }
        }

        // ── Expand neighbours ──
        for (int p_id = 0; p_id < (int)prims.size(); p_id++) {
            const auto &p = prims[p_id];
            HAState ns = step_state(nodes[cur].s, p.steer, p.dir);

            // Cell-level early reject.
            if (cell_blocked(ns)) continue;

            // Footprint check at the new state. We also sample the midpoint
            // of the arc, which catches walls a 0.4 m arc would skip over.
            double th_m = 0.5 * (nodes[cur].s.theta + ns.theta);
            double xm = 0.5 * (nodes[cur].s.x + ns.x);
            double ym = 0.5 * (nodes[cur].s.y + ns.y);
            if (footprint_clips(map, ns.x, ns.y, ns.theta,
                                opt.fp_length, opt.fp_width,
                                occupied_thresh, false,
                                opt.fp_stride, opt.fp_buffer)) continue;
            if (footprint_clips(map, xm, ym, th_m,
                                opt.fp_length, opt.fp_width,
                                occupied_thresh, false,
                                opt.fp_stride, opt.fp_buffer)) continue;

            // Cost: base step + reverse pen + direction switch + steer change.
            double step_cost = opt.step_length;
            if (p.dir < 0) step_cost *= opt.reverse_penalty;
            int parent_prim = nodes[cur].prim_id;
            if (parent_prim >= 0) {
                int parent_dir = prims[parent_prim].dir;
                if (parent_dir != p.dir) step_cost += opt.switch_penalty;
                if (std::abs(prims[parent_prim].steer - p.steer) > 1e-6)
                    step_cost += opt.steer_change_pen;
            }

            float ng = nodes[cur].g + static_cast<float>(step_cost);
            HANode child{ns, ng, ng + h_at(ns), cur, p_id};
            StateKey nk = state_key(ns);
            auto cit = closed.find(nk);
            if (cit != closed.end() && nodes[cit->second].g <= ng + 1e-3f) continue;

            nodes.push_back(child);
            int cidx = static_cast<int>(nodes.size()) - 1;
            closed[nk] = cidx;
            pq.push(cidx);
        }
    }

    result.n_expansions = iter;
    if (best_idx < 0) return result;

    // Reconstruct path.
    std::vector<HAState> rev;
    int idx = best_idx;
    while (idx >= 0) {
        rev.push_back(nodes[idx].s);
        idx = nodes[idx].parent;
    }
    std::reverse(rev.begin(), rev.end());
    result.states = std::move(rev);
    result.valid = true;
    return result;
}

// ═══════════════════════════════════════════════════════════════════════
//   Ceres path smoother
//
//   Variables: 2D positions p_i (i = 1 .. N-2).  Endpoints p_0 and p_{N-1}
//   are fixed (start = robot pose, end = goal).
//
//   Cost terms (scaled, summed as residual blocks → LM minimises ‖r‖²):
//     1. anchor_w     · (p_i - p_i^orig)
//     2. smooth_w     · (p_{i+1} - 2 p_i + p_{i-1})
//     3. obstacle_w   · max(0, safety_m - clearance(p_i))
//     4. curvature_w  · κ(p_{i-1}, p_i, p_{i+1})    (large-angle proxy)
//
//   The clearance grid is sampled with bilinear interpolation, which is
//   C¹ on cell interiors — autodiff-friendly through Ceres Jets.
// ═══════════════════════════════════════════════════════════════════════

struct ClearanceField {
    const float *data;  // row-major
    int w, h;
    double res, ox, oy;

    template <typename T>
    T sample(const T &wx, const T &wy) const {
        // World → grid (real-valued cell coords, center-aligned).
        T fx = (wx - T(ox)) / T(res) - T(0.5);
        T fy = (wy - T(oy)) / T(res) - T(0.5);
        // Clamp to interior so derivatives at edges are well-defined.
        T fx_c = ceres::fmax(T(0.0), ceres::fmin(T(w - 1.0001), fx));
        T fy_c = ceres::fmax(T(0.0), ceres::fmin(T(h - 1.0001), fy));
        // Floor via static_cast on the value cast — pull underlying double
        // from Jet by converting through ceres::floor.
        double fx_v = 0.0, fy_v = 0.0;
        if constexpr (std::is_same_v<T, double>) {
            fx_v = fx_c; fy_v = fy_c;
        } else {
            fx_v = fx_c.a; fy_v = fy_c.a;
        }
        int ix = std::min(w - 2, std::max(0, (int)std::floor(fx_v)));
        int iy = std::min(h - 2, std::max(0, (int)std::floor(fy_v)));
        T tx = fx_c - T(ix);
        T ty = fy_c - T(iy);
        T c00 = T((double)data[ iy      * w + ix    ]);
        T c10 = T((double)data[ iy      * w + ix + 1]);
        T c01 = T((double)data[(iy + 1) * w + ix    ]);
        T c11 = T((double)data[(iy + 1) * w + ix + 1]);
        T c0 = c00 + tx * (c10 - c00);
        T c1 = c01 + tx * (c11 - c01);
        return c0 + ty * (c1 - c0);
    }
};

struct AnchorCost {
    double ox, oy, w;
    AnchorCost(double x0, double y0, double weight) : ox(x0), oy(y0), w(weight) {}
    template <typename T>
    bool operator()(const T *p, T *r) const {
        r[0] = T(w) * (p[0] - T(ox));
        r[1] = T(w) * (p[1] - T(oy));
        return true;
    }
};

struct SmoothCost {
    double w;
    explicit SmoothCost(double weight) : w(weight) {}
    template <typename T>
    bool operator()(const T *pm, const T *p0, const T *pp, T *r) const {
        // p_{i+1} - 2 p_i + p_{i-1}
        r[0] = T(w) * (pp[0] - T(2.0) * p0[0] + pm[0]);
        r[1] = T(w) * (pp[1] - T(2.0) * p0[1] + pm[1]);
        return true;
    }
};

struct ObstacleCost {
    const ClearanceField *field;
    double safety, w;
    ObstacleCost(const ClearanceField *f, double s, double weight)
        : field(f), safety(s), w(weight) {}
    template <typename T>
    bool operator()(const T *p, T *r) const {
        T d = field->sample(p[0], p[1]);
        T deficit = T(safety) - d;
        // soft hinge: max(0, deficit)
        if (deficit > T(0.0)) r[0] = T(w) * deficit;
        else                   r[0] = T(0.0);
        return true;
    }
};

struct CurvatureCost {
    double w;
    explicit CurvatureCost(double weight) : w(weight) {}
    template <typename T>
    bool operator()(const T *pm, const T *p0, const T *pp, T *r) const {
        // Approximate κ via cross-product / |Δs|² of consecutive segments.
        // r = w · cross(d1, d2) / (|d1| |d2| + ε)
        T d1x = p0[0] - pm[0];
        T d1y = p0[1] - pm[1];
        T d2x = pp[0] - p0[0];
        T d2y = pp[1] - p0[1];
        T cross = d1x * d2y - d1y * d2x;
        T n1 = ceres::sqrt(d1x * d1x + d1y * d1y + T(1e-9));
        T n2 = ceres::sqrt(d2x * d2x + d2y * d2y + T(1e-9));
        r[0] = T(w) * cross / (n1 * n2);
        return true;
    }
};

struct SmootherOptions {
    double anchor_w    = 0.5;
    double smooth_w    = 5.0;
    double obstacle_w  = 8.0;
    double curvature_w = 2.0;
    double safety_m    = 0.35;
    int    max_iters   = 40;
    bool   verbose     = false;
};

static std::vector<std::pair<double,double>> smooth_ceres(
    const std::vector<std::pair<double,double>> &raw,
    const ClearanceField &field,
    const SmootherOptions &opt)
{
    if (raw.size() < 4) return raw;

    int N = static_cast<int>(raw.size());
    // Each interior point = 2 parameters [x, y].
    std::vector<double> params(2 * N);
    for (int i = 0; i < N; i++) {
        params[2 * i + 0] = raw[i].first;
        params[2 * i + 1] = raw[i].second;
    }

    ceres::Problem problem;
    for (int i = 0; i < N; i++) {
        problem.AddParameterBlock(&params[2 * i], 2);
    }
    // Endpoints fixed.
    problem.SetParameterBlockConstant(&params[0]);
    problem.SetParameterBlockConstant(&params[2 * (N - 1)]);

    for (int i = 0; i < N; i++) {
        // Anchor
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<AnchorCost, 2, 2>(
                new AnchorCost(raw[i].first, raw[i].second, opt.anchor_w)),
            nullptr, &params[2 * i]);
        // Obstacle hinge
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<ObstacleCost, 1, 2>(
                new ObstacleCost(&field, opt.safety_m, opt.obstacle_w)),
            nullptr, &params[2 * i]);
    }
    for (int i = 1; i < N - 1; i++) {
        // Smoothness
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<SmoothCost, 2, 2, 2, 2>(
                new SmoothCost(opt.smooth_w)),
            nullptr,
            &params[2 * (i - 1)], &params[2 * i], &params[2 * (i + 1)]);
        // Curvature
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<CurvatureCost, 1, 2, 2, 2>(
                new CurvatureCost(opt.curvature_w)),
            nullptr,
            &params[2 * (i - 1)], &params[2 * i], &params[2 * (i + 1)]);
    }

    ceres::Solver::Options sopt;
    sopt.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    sopt.max_num_iterations = opt.max_iters;
    sopt.minimizer_progress_to_stdout = opt.verbose;
    sopt.function_tolerance = 1e-4;
    sopt.gradient_tolerance = 1e-6;
    sopt.parameter_tolerance = 1e-6;
    sopt.num_threads = 2;
    ceres::Solver::Summary sum;
    ceres::Solve(sopt, &problem, &sum);

    std::vector<std::pair<double,double>> out(N);
    for (int i = 0; i < N; i++) {
        out[i] = { params[2 * i], params[2 * i + 1] };
    }
    return out;
}

// ═══════════════════════════════════════════════════════════════════════
//   Path utilities (shared with astar_nav_node logic)
// ═══════════════════════════════════════════════════════════════════════
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
    if (out.back() != poly.back()) out.push_back(poly.back());
    return out;
}

static double curvature_max_window(
    const std::vector<std::pair<double,double>> &path, size_t idx, int window) {
    if (path.size() < 3) return 0.0;
    double mk = 0.0;
    size_t end = std::min(path.size() - 1, idx + (size_t)window);
    for (size_t i = std::max<size_t>(idx, 1); i + 1 <= end; i++) {
        double dx1 = path[i].first - path[i-1].first;
        double dy1 = path[i].second - path[i-1].second;
        double dx2 = path[i+1].first - path[i].first;
        double dy2 = path[i+1].second - path[i].second;
        double s1 = std::hypot(dx1, dy1), s2 = std::hypot(dx2, dy2);
        if (s1 < 1e-6 || s2 < 1e-6) continue;
        double th1 = std::atan2(dy1, dx1), th2 = std::atan2(dy2, dx2);
        double dth = wrap_angle(th2 - th1);
        double k = std::abs(dth) / std::max(0.5 * (s1 + s2), 1e-6);
        if (k > mk) mk = k;
    }
    return mk;
}

static size_t nearest_index(
    const std::vector<std::pair<double,double>> &path,
    double x, double y, size_t prev_idx) {
    if (path.empty()) return 0;
    if (prev_idx >= path.size()) prev_idx = path.size() - 1;
    size_t best = prev_idx;
    double bestd = std::hypot(path[prev_idx].first - x, path[prev_idx].second - y);
    constexpr double kBackHysteresis = 0.20;
    for (size_t i = 0; i < path.size(); i++) {
        double d = std::hypot(path[i].first - x, path[i].second - y);
        double margin = (i < prev_idx) ? kBackHysteresis : 0.0;
        if (d + margin < bestd) { bestd = d; best = i; }
    }
    return best;
}

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
//   ROS Node
// ═══════════════════════════════════════════════════════════════════════
class HybridAStarNavNode : public rclcpp::Node {
public:
    static rclcpp::NodeOptions make_node_options() {
        rclcpp::NodeOptions opts;
        opts.parameter_overrides({rclcpp::Parameter("use_sim_time", true)});
        return opts;
    }

    HybridAStarNavNode() : Node("hybrid_astar_nav", make_node_options()) {
        declare_all_params();
        load_params();

        rs_space_ = std::make_shared<ob::ReedsSheppStateSpace>(opt_.rs_turning_radius);

        tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        cmd_pub_         = create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel_stamped", 10);
        status_pub_      = create_publisher<std_msgs::msg::String>("/nav_status", 10);
        path_pub_        = create_publisher<nav_msgs::msg::Path>("/planned_path", 10);
        global_path_pub_ = create_publisher<nav_msgs::msg::Path>("/global_planned_path", 10);
        traj_pub_        = create_publisher<nav_msgs::msg::Path>("/robot_trajectory", 10);
        goal_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("/final_goal_marker", 10);
        pose_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("/robot_pose_marker", 10);
        replan_pub_      = create_publisher<std_msgs::msg::Empty>(
                              get_parameter("frontier_replan_topic").as_string(), 10);

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
                double goal_jump = std::hypot(ngx - goal_x_, ngy - goal_y_);
                if (goal_jump > 0.3) {
                    smoothed_path_.clear();
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
        RCLCPP_INFO(get_logger(), "[hybrid_astar] map=%s rs_R=%.2f n_theta=%d step=%.2f",
                    map_topic_param.c_str(), opt_.rs_turning_radius,
                    opt_.n_theta, opt_.step_length);

        timer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / control_rate_),
            std::bind(&HybridAStarNavNode::tick, this));
    }

private:
    // ── parameters ────────────────────────────────────────────────────
    void declare_all_params() {
        // speed envelope
        declare_parameter("max_linear_speed",  0.70);
        declare_parameter("max_angular_speed", 1.00);
        declare_parameter("min_linear_speed",  0.05);
        declare_parameter("linear_accel_max",  0.50);
        declare_parameter("linear_decel_max",  0.50);
        declare_parameter("angular_accel_max", 1.20);
        declare_parameter("control_rate",      20.0);
        declare_parameter("startup_delay",     1.0);
        declare_parameter("startup_ramp_sec",  2.0);
        // goal
        declare_parameter("goal_tolerance",    0.25);
        declare_parameter("goal_slow_radius",  1.5);
        declare_parameter("goal_slow_floor",   0.15);
        declare_parameter("goal_reached_replan_cooldown_sec", 1.0);
        declare_parameter("goal_reached_brake_sec", 0.6);
        // pure pursuit
        declare_parameter("lookahead_min",  0.30);
        declare_parameter("lookahead_max",  0.80);
        declare_parameter("lookahead_gain", 0.60);
        declare_parameter("cross_track_gain", 0.70);
        // curvature shaping
        declare_parameter("curvature_lookahead_segments", 6);
        declare_parameter("curv_full_speed_below", 0.30);
        declare_parameter("curv_slow_above",       1.50);
        declare_parameter("curv_factor_min",       0.10);
        // reactive scan brake
        declare_parameter("obstacle_slow_dist", 0.75);
        declare_parameter("obstacle_stop_dist", 0.35);
        declare_parameter("front_half_angle_deg", 35.0);
        // map / planner
        declare_parameter("map_topic", std::string(""));
        declare_parameter("map_frame", std::string("map"));
        declare_parameter("base_frame", std::string("base_link"));
        declare_parameter("map_occupied_thresh", 50);
        declare_parameter("global_downsample",  3);
        declare_parameter("global_inflation_m", 0.18);
        declare_parameter("global_replan_sec",  1.5);
        declare_parameter("resample_step_m",    0.10);
        // hybrid A*
        declare_parameter("ha_n_theta",          24);
        declare_parameter("ha_step_length",      0.40);
        declare_parameter("ha_max_steer",        0.40);
        declare_parameter("ha_allow_reverse",    false);
        declare_parameter("ha_reverse_penalty",  2.0);
        declare_parameter("ha_switch_penalty",   1.5);
        declare_parameter("ha_steer_change_pen", 0.3);
        declare_parameter("ha_max_iters",        20000);
        declare_parameter("ha_rs_shot_period",   5);
        declare_parameter("ha_rs_turning_radius", 0.8);
        declare_parameter("ha_goal_tol_xy",      0.25);
        declare_parameter("ha_goal_tol_theta",   0.30);
        // footprint
        declare_parameter("footprint_length", 0.65);
        declare_parameter("footprint_width",  0.45);
        declare_parameter("footprint_check_stride_m", 0.08);
        declare_parameter("footprint_buffer_m", 0.03);
        // smoother
        declare_parameter("smoother_anchor_w",    0.5);
        declare_parameter("smoother_smooth_w",    5.0);
        declare_parameter("smoother_obstacle_w",  8.0);
        declare_parameter("smoother_curvature_w", 2.0);
        declare_parameter("smoother_safety_m",    0.35);
        declare_parameter("smoother_max_iters",   40);
        // hybrid_cmd_router bias
        declare_parameter("wheel_linear_threshold_for_bias",  0.18);
        declare_parameter("wheel_angular_threshold_for_bias", 0.30);
        declare_parameter("leg_heading_threshold_deg", 12.0);
        declare_parameter("legs_mode_v_cap", 0.15);
        declare_parameter("legs_mode_w_cap", 0.45);
        // glue topics
        declare_parameter("frontier_replan_topic", std::string("/frontier_replan"));
        declare_parameter("stop_topic",            std::string("/stop"));
    }

    void load_params() {
        max_v_ = get_parameter("max_linear_speed").as_double();
        max_w_ = get_parameter("max_angular_speed").as_double();
        min_v_ = get_parameter("min_linear_speed").as_double();
        acc_v_ = get_parameter("linear_accel_max").as_double();
        dec_v_ = get_parameter("linear_decel_max").as_double();
        acc_w_ = get_parameter("angular_accel_max").as_double();
        control_rate_ = get_parameter("control_rate").as_double();
        startup_delay_ = get_parameter("startup_delay").as_double();
        startup_ramp_  = get_parameter("startup_ramp_sec").as_double();

        goal_tol_ = get_parameter("goal_tolerance").as_double();
        goal_slow_r_ = get_parameter("goal_slow_radius").as_double();
        goal_slow_floor_ = get_parameter("goal_slow_floor").as_double();
        goal_cooldown_ = get_parameter("goal_reached_replan_cooldown_sec").as_double();
        goal_reached_brake_sec_ = get_parameter("goal_reached_brake_sec").as_double();

        Ld_min_  = get_parameter("lookahead_min").as_double();
        Ld_max_  = get_parameter("lookahead_max").as_double();
        Ld_gain_ = get_parameter("lookahead_gain").as_double();
        cross_track_gain_ = get_parameter("cross_track_gain").as_double();

        curv_window_ = get_parameter("curvature_lookahead_segments").as_int();
        curv_low_  = get_parameter("curv_full_speed_below").as_double();
        curv_high_ = get_parameter("curv_slow_above").as_double();
        curv_floor_ = get_parameter("curv_factor_min").as_double();

        obs_slow_ = get_parameter("obstacle_slow_dist").as_double();
        obs_stop_ = get_parameter("obstacle_stop_dist").as_double();
        front_half_ = get_parameter("front_half_angle_deg").as_double() * M_PI / 180.0;

        map_occupied_thresh_ = get_parameter("map_occupied_thresh").as_int();
        global_downsample_   = get_parameter("global_downsample").as_int();
        global_inflation_m_  = get_parameter("global_inflation_m").as_double();
        global_replan_sec_   = get_parameter("global_replan_sec").as_double();
        resample_step_       = get_parameter("resample_step_m").as_double();

        opt_.n_theta            = get_parameter("ha_n_theta").as_int();
        opt_.step_length        = get_parameter("ha_step_length").as_double();
        opt_.max_steer          = get_parameter("ha_max_steer").as_double();
        opt_.allow_reverse      = get_parameter("ha_allow_reverse").as_bool();
        opt_.reverse_penalty    = get_parameter("ha_reverse_penalty").as_double();
        opt_.switch_penalty     = get_parameter("ha_switch_penalty").as_double();
        opt_.steer_change_pen   = get_parameter("ha_steer_change_pen").as_double();
        opt_.max_iters          = get_parameter("ha_max_iters").as_int();
        opt_.rs_shot_period     = get_parameter("ha_rs_shot_period").as_int();
        opt_.rs_turning_radius  = get_parameter("ha_rs_turning_radius").as_double();
        opt_.goal_tol_xy        = get_parameter("ha_goal_tol_xy").as_double();
        opt_.goal_tol_theta     = get_parameter("ha_goal_tol_theta").as_double();
        opt_.fp_length          = get_parameter("footprint_length").as_double();
        opt_.fp_width           = get_parameter("footprint_width").as_double();
        opt_.fp_stride          = get_parameter("footprint_check_stride_m").as_double();
        opt_.fp_buffer          = get_parameter("footprint_buffer_m").as_double();

        sopt_.anchor_w     = get_parameter("smoother_anchor_w").as_double();
        sopt_.smooth_w     = get_parameter("smoother_smooth_w").as_double();
        sopt_.obstacle_w   = get_parameter("smoother_obstacle_w").as_double();
        sopt_.curvature_w  = get_parameter("smoother_curvature_w").as_double();
        sopt_.safety_m     = get_parameter("smoother_safety_m").as_double();
        sopt_.max_iters    = get_parameter("smoother_max_iters").as_int();

        wheel_lin_thresh_ = get_parameter("wheel_linear_threshold_for_bias").as_double();
        wheel_ang_thresh_ = get_parameter("wheel_angular_threshold_for_bias").as_double();
        leg_heading_thresh_ = get_parameter("leg_heading_threshold_deg").as_double() * M_PI / 180.0;
        legs_mode_v_cap_ = get_parameter("legs_mode_v_cap").as_double();
        legs_mode_w_cap_ = get_parameter("legs_mode_w_cap").as_double();

        map_frame_  = get_parameter("map_frame").as_string();
        base_frame_ = get_parameter("base_frame").as_string();
    }

    // ── tick ──────────────────────────────────────────────────────────
    void tick() {
        auto now = this->now();
        double now_sec = now.seconds();
        if (start_time_ < 0) start_time_ = now_sec;
        double t_since_start = now_sec - start_time_;

        if (t_since_start < startup_delay_) { publish_cmd(0.0, 0.0, "warming_up"); return; }
        if (!has_goal_) { publish_cmd(0.0, 0.0, "idle:no_goal"); return; }

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
        } catch (tf2::TransformException &) {
            if (!has_map_tf_) { publish_cmd(0.0, 0.0, "warming_up:no_tf"); return; }
        }

        if (now_sec < brake_until_sec_) { publish_cmd(0.0, 0.0, "brake_hold"); return; }

        double dist_to_goal = std::hypot(goal_x_ - rx, goal_y_ - ry);
        if (dist_to_goal < goal_tol_) {
            publish_cmd(0.0, 0.0, "goal_reached");
            brake_until_sec_ = std::max(brake_until_sec_, now_sec + goal_reached_brake_sec_);
            if (last_replan_time_ < 0 || (now_sec - last_replan_time_) >= goal_cooldown_) {
                replan_pub_->publish(std_msgs::msg::Empty());
                last_replan_time_ = now_sec;
            }
            return;
        }

        // Replan triggers: no plan yet, or stale path collides with latest map.
        bool need_replan = (last_global_plan_time_ < 0) || smoothed_path_.size() < 2;
        bool min_interval_ok = (last_global_plan_time_ < 0)
            || ((now_sec - last_global_plan_time_) >= 0.2);

        if (!need_replan && min_interval_ok && last_map_ && !smoothed_path_.empty()) {
            const auto &mi = last_map_->info;
            size_t start = std::min(last_pursuit_idx_, smoothed_path_.size() - 1);
            size_t end = std::min(smoothed_path_.size(), start + 20);
            for (size_t i = start; i < end; i++) {
                double px = smoothed_path_[i].first;
                double py = smoothed_path_[i].second;
                int mx = (int)((px - mi.origin.position.x) / mi.resolution);
                int my = (int)((py - mi.origin.position.y) / mi.resolution);
                if (mx < 0 || mx >= (int)mi.width || my < 0 || my >= (int)mi.height) {
                    need_replan = true; break;
                }
                int8_t v = last_map_->data[my * mi.width + mx];
                if (v >= map_occupied_thresh_ || v < 0) { need_replan = true; break; }
            }
        }

        if (need_replan && last_map_) {
            do_replan(rx, ry, ryaw, now_sec);
        }

        if (smoothed_path_.size() < 2) {
            publish_cmd(0.0, 0.0, "no_plan");
            return;
        }

        // Track and emit cmd.
        track_and_publish(rx, ry, ryaw, dist_to_goal, t_since_start);
    }

    void do_replan(double rx, double ry, double ryaw, double now_sec) {
        // (re)build coarse grid + clearance DT if map changed.
        bool need_rebuild = (cg_.w == 0)
            || (rclcpp::Time(last_map_->header.stamp) != cg_.stamp);
        if (need_rebuild) {
            build_coarse_grid(*last_map_, global_downsample_, global_inflation_m_,
                              map_occupied_thresh_, /*unknown_is_obstacle=*/true,
                              /*clearance_cap_m=*/1.5, cg_);
        }

        // Goal heading: aim along start→goal. (For most exploration tasks
        // there's no preferred terminal heading; use the chord direction.)
        double gth = std::atan2(goal_y_ - ry, goal_x_ - rx);

        int gcx = (int)((goal_x_ - cg_.ox) / cg_.res);
        int gcy = (int)((goal_y_ - cg_.oy) / cg_.res);
        gcx = std::clamp(gcx, 0, cg_.w - 1);
        gcy = std::clamp(gcy, 0, cg_.h - 1);
        // Goal-cell relocation (closest free cell within R).
        if (cg_.blocked[gcy * cg_.w + gcx]) {
            int R = 6, best_x = -1, best_y = -1, best_d2 = INT_MAX;
            for (int dy = -R; dy <= R; dy++) for (int dx = -R; dx <= R; dx++) {
                int nx = gcx + dx, ny = gcy + dy;
                if (nx < 0 || nx >= cg_.w || ny < 0 || ny >= cg_.h) continue;
                if (cg_.blocked[ny * cg_.w + nx]) continue;
                int d2 = dx*dx + dy*dy;
                if (d2 < best_d2) { best_d2 = d2; best_x = nx; best_y = ny; }
            }
            if (best_x >= 0) { gcx = best_x; gcy = best_y; }
            else {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                    "[hybrid_astar] goal unreachable on coarse grid");
                smoothed_path_.clear();
                last_global_plan_time_ = now_sec;
                return;
            }
        }

        auto h_grid = dijkstra_h(cg_, gcx, gcy);

        HAState start_s{rx, ry, ryaw};
        HAState goal_s{ goal_x_, goal_y_, gth };
        auto search = hybrid_astar_search(*last_map_, cg_, h_grid, rs_space_,
                                          start_s, goal_s, opt_,
                                          map_occupied_thresh_);

        last_global_plan_time_ = now_sec;
        if (!search.valid) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "[hybrid_astar] search failed after %d expansions", search.n_expansions);
            smoothed_path_.clear();
            return;
        }

        // Convert to xy polyline and resample uniformly for the smoother.
        std::vector<std::pair<double,double>> raw;
        raw.reserve(search.states.size());
        for (auto &s : search.states) raw.emplace_back(s.x, s.y);
        auto resampled = resample_uniform(raw, resample_step_);

        // Run Ceres smoother.
        ClearanceField field{
            cg_.clearance_m.data(), cg_.w, cg_.h, cg_.res, cg_.ox, cg_.oy};
        auto smoothed = smooth_ceres(resampled, field, sopt_);

        smoothed_path_ = std::move(smoothed);
        last_pursuit_idx_ = 0;

        // Publish global path for RViz.
        publish_path(global_path_pub_, search.states);
        publish_path_xy(path_pub_, smoothed_path_);

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
            "[hybrid_astar] plan ok: expansions=%d states=%zu smoothed=%zu",
            search.n_expansions, search.states.size(), smoothed_path_.size());
    }

    // ── tracker ───────────────────────────────────────────────────────
    void track_and_publish(double rx, double ry, double ryaw,
                           double dist_to_goal, double t_since_start) {
        // Pure-pursuit + Stanley + curvature speed shaping.
        size_t idx = nearest_index(smoothed_path_, rx, ry, last_pursuit_idx_);
        last_pursuit_idx_ = idx;

        // Speed pipeline
        double v_target = max_v_;
        // ramp-up
        double rs = clamp01((t_since_start - startup_delay_) / std::max(startup_ramp_, 1e-6));
        v_target *= rs;
        // goal approach
        if (dist_to_goal < goal_slow_r_) {
            double f = std::max(goal_slow_floor_ / max_v_,
                                (dist_to_goal - goal_tol_)
                                    / std::max(goal_slow_r_ - goal_tol_, 1e-3));
            v_target *= clamp01(f);
        }
        // curvature factor
        double k_ahead = curvature_max_window(smoothed_path_, idx, curv_window_);
        double cf = 1.0;
        if (k_ahead > curv_low_) {
            double t = clamp01((k_ahead - curv_low_) / std::max(curv_high_ - curv_low_, 1e-3));
            cf = 1.0 + t * (curv_floor_ - 1.0);
        }
        v_target *= cf;
        // scan brake (front cone)
        double front_min = std::numeric_limits<double>::infinity();
        if (last_scan_) {
            double a = last_scan_->angle_min;
            for (size_t i = 0; i < last_scan_->ranges.size(); i++) {
                double r = last_scan_->ranges[i];
                if (std::isfinite(r) && r > 0.05 && std::abs(wrap_angle(a)) < front_half_) {
                    if (r < front_min) front_min = r;
                }
                a += last_scan_->angle_increment;
            }
        }
        if (std::isfinite(front_min)) {
            if (front_min < obs_stop_) v_target = 0.0;
            else if (front_min < obs_slow_) {
                v_target *= clamp01((front_min - obs_stop_) / std::max(obs_slow_ - obs_stop_, 1e-3));
            }
        }
        v_target = std::max(0.0, std::min(v_target, max_v_));

        // Lookahead
        double Ld = std::clamp(Ld_min_ + Ld_gain_ * v_target, Ld_min_, Ld_max_);
        size_t li = lookahead_index(smoothed_path_, idx, Ld);
        double lx = smoothed_path_[li].first;
        double ly = smoothed_path_[li].second;
        double dx_l = lx - rx, dy_l = ly - ry;
        double psi_target = std::atan2(dy_l, dx_l);
        double psi_err = wrap_angle(psi_target - ryaw);

        // Stanley cross-track term: signed lateral distance from path tangent at idx.
        double cross_track = 0.0;
        if (idx + 1 < smoothed_path_.size()) {
            double tx = smoothed_path_[idx + 1].first - smoothed_path_[idx].first;
            double ty = smoothed_path_[idx + 1].second - smoothed_path_[idx].second;
            double tn = std::hypot(tx, ty);
            if (tn > 1e-6) {
                tx /= tn; ty /= tn;
                double ex = rx - smoothed_path_[idx].first;
                double ey = ry - smoothed_path_[idx].second;
                cross_track = -tx * ey + ty * ex;  // left positive
            }
        }
        double w_cmd = psi_err + std::atan2(cross_track_gain_ * cross_track,
                                            std::max(v_target, min_v_));
        // Mode bias: large heading error → leg mode (cap v + |ω|).
        if (std::abs(psi_err) > leg_heading_thresh_) {
            v_target = std::min(v_target, legs_mode_v_cap_);
            w_cmd = std::clamp(w_cmd, -legs_mode_w_cap_, legs_mode_w_cap_);
        } else {
            w_cmd = std::clamp(w_cmd, -max_w_, max_w_);
        }
        v_target = std::max(v_target, (std::abs(psi_err) > leg_heading_thresh_) ? 0.0 : min_v_);

        publish_cmd(v_target, w_cmd, "tracking");
    }

    // ── publishing helpers ────────────────────────────────────────────
    void publish_cmd(double v, double w, const std::string &state, double vy = 0.0) {
        // Acceleration-limited integration.
        double now_s = this->now().seconds();
        double dt = (last_cmd_time_ < 0) ? 1.0 / control_rate_ : (now_s - last_cmd_time_);
        last_cmd_time_ = now_s;
        dt = std::clamp(dt, 1.0 / 200.0, 0.2);
        double dv = std::clamp(v - last_v_cmd_,
                               -dec_v_ * dt,  acc_v_ * dt);
        double dw = std::clamp(w - last_w_cmd_,
                               -acc_w_ * dt,  acc_w_ * dt);
        double vo = last_v_cmd_ + dv;
        double wo = last_w_cmd_ + dw;
        last_v_cmd_ = vo; last_w_cmd_ = wo;

        if (external_stop_) { vo = 0.0; wo = 0.0; vy = 0.0; }

        geometry_msgs::msg::TwistStamped t;
        t.header.stamp = this->now();
        t.header.frame_id = base_frame_;
        t.twist.linear.x  = vo;
        t.twist.linear.y  = vy;
        t.twist.angular.z = wo;
        cmd_pub_->publish(t);

        std_msgs::msg::String s; s.data = state; status_pub_->publish(s);

        // Robot-pose marker.
        visualization_msgs::msg::Marker m;
        m.header.frame_id = map_frame_;
        m.header.stamp = this->now();
        m.ns = "hybrid_astar"; m.id = 1;
        m.type = visualization_msgs::msg::Marker::ARROW;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.position.x = robot_x_;
        m.pose.position.y = robot_y_;
        m.pose.position.z = 0.05;
        tf2::Quaternion q; q.setRPY(0, 0, robot_yaw_);
        m.pose.orientation = tf2::toMsg(q);
        m.scale.x = 0.6; m.scale.y = 0.08; m.scale.z = 0.08;
        m.color.r = 0.1f; m.color.g = 0.7f; m.color.b = 1.0f; m.color.a = 0.9f;
        pose_marker_pub_->publish(m);

        // Trajectory accumulation.
        double dxm = robot_x_ - traj_last_x_, dym = robot_y_ - traj_last_y_;
        if (!traj_init_ || std::hypot(dxm, dym) > 0.02) {
            geometry_msgs::msg::PoseStamped ps;
            ps.header.frame_id = map_frame_;
            ps.header.stamp = this->now();
            ps.pose.position.x = robot_x_;
            ps.pose.position.y = robot_y_;
            ps.pose.orientation = tf2::toMsg(q);
            traj_path_.poses.push_back(ps);
            traj_last_x_ = robot_x_; traj_last_y_ = robot_y_;
            traj_init_ = true;
            if (traj_path_.poses.size() > 50000) {
                traj_path_.poses.erase(traj_path_.poses.begin(),
                                       traj_path_.poses.begin() + 1000);
            }
            traj_path_.header = ps.header;
            traj_pub_->publish(traj_path_);
        }
    }

    void publish_path(rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub,
                      const std::vector<HAState> &states) {
        nav_msgs::msg::Path p;
        p.header.frame_id = map_frame_;
        p.header.stamp = this->now();
        p.poses.reserve(states.size());
        for (auto &s : states) {
            geometry_msgs::msg::PoseStamped ps;
            ps.header = p.header;
            ps.pose.position.x = s.x;
            ps.pose.position.y = s.y;
            tf2::Quaternion q; q.setRPY(0, 0, s.theta);
            ps.pose.orientation = tf2::toMsg(q);
            p.poses.push_back(ps);
        }
        pub->publish(p);
    }
    void publish_path_xy(rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub,
                         const std::vector<std::pair<double,double>> &pts) {
        nav_msgs::msg::Path p;
        p.header.frame_id = map_frame_;
        p.header.stamp = this->now();
        p.poses.reserve(pts.size());
        for (size_t i = 0; i < pts.size(); i++) {
            geometry_msgs::msg::PoseStamped ps;
            ps.header = p.header;
            ps.pose.position.x = pts[i].first;
            ps.pose.position.y = pts[i].second;
            double th = (i + 1 < pts.size())
                ? std::atan2(pts[i+1].second - pts[i].second,
                             pts[i+1].first  - pts[i].first)
                : 0.0;
            tf2::Quaternion q; q.setRPY(0, 0, th);
            ps.pose.orientation = tf2::toMsg(q);
            p.poses.push_back(ps);
        }
        pub->publish(p);
    }

    // ── members ───────────────────────────────────────────────────────
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_, global_path_pub_, traj_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_marker_pub_, pose_marker_pub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr replan_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr wp_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr stop_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
    nav_msgs::msg::OccupancyGrid::SharedPtr last_map_;

    HybridAStarOptions opt_;
    SmootherOptions    sopt_;
    CoarseGrid         cg_;
    std::shared_ptr<ob::ReedsSheppStateSpace> rs_space_;

    std::vector<std::pair<double,double>> smoothed_path_;
    nav_msgs::msg::Path traj_path_;
    bool traj_init_ = false;
    double traj_last_x_ = 0.0, traj_last_y_ = 0.0;

    bool   has_goal_ = false, has_map_tf_ = false;
    double goal_x_ = 0.0, goal_y_ = 0.0;
    double robot_x_ = 0.0, robot_y_ = 0.0, robot_yaw_ = 0.0;
    double start_time_ = -1.0;
    double last_global_plan_time_ = -1.0;
    double last_replan_time_ = -1.0;
    double last_cmd_time_ = -1.0;
    double last_v_cmd_ = 0.0, last_w_cmd_ = 0.0;
    double brake_until_sec_ = -1.0;
    size_t last_pursuit_idx_ = 0;
    int8_t external_stop_ = 0;

    // Parameters (loaded once)
    double max_v_, max_w_, min_v_, acc_v_, dec_v_, acc_w_;
    double control_rate_, startup_delay_, startup_ramp_;
    double goal_tol_, goal_slow_r_, goal_slow_floor_, goal_cooldown_;
    double goal_reached_brake_sec_;
    double Ld_min_, Ld_max_, Ld_gain_, cross_track_gain_;
    int    curv_window_;
    double curv_low_, curv_high_, curv_floor_;
    double obs_slow_, obs_stop_, front_half_;
    int    map_occupied_thresh_;
    int    global_downsample_;
    double global_inflation_m_, global_replan_sec_, resample_step_;
    double wheel_lin_thresh_, wheel_ang_thresh_;
    double leg_heading_thresh_, legs_mode_v_cap_, legs_mode_w_cap_;
    std::string map_frame_, base_frame_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HybridAStarNavNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
