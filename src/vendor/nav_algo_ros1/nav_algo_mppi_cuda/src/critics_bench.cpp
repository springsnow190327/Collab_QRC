// Copyright 2026 Collab_QRC
// SPDX-License-Identifier: Apache-2.0
//
// CPU-vs-GPU bench for the MPPI critic kernels. For each critic, runs the
// CPU reference (plain C++ loop matching xtensor semantics from Nav2's
// score() body) and the GPU kernel on random inputs of the Nav2 canonical
// shape (B=2000, T=56), then diffs costs[B] elementwise.
//
// PASS criterion: max abs diff < 1e-4 (fp32 reduction-ordering tolerance).

#include "nav_algo_mppi_cuda/critics.cuh"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <vector>

#include <cuda_runtime.h>

namespace
{

// ── GoalCritic CPU reference ─────────────────────────────────────────────
// Port of mppi::critics::GoalCritic::score (goal_critic.cpp:37-58):
//   dists[b,t] = sqrt((traj_x - goal_x)² + (traj_y - goal_y)²)
//   costs[b] += pow(mean_t(dists[b,:]) * weight, power)
void cpuGoalCritic(
  unsigned int B, unsigned int T,
  const std::vector<float> & traj_x,
  const std::vector<float> & traj_y,
  float goal_x, float goal_y,
  float weight, int power,
  std::vector<float> & costs)
{
  for (unsigned int b = 0; b < B; ++b) {
    double sum = 0.0;  // sum in double to keep CPU reference high-precision
    for (unsigned int t = 0; t < T; ++t) {
      const float dx = traj_x[b * T + t] - goal_x;
      const float dy = traj_y[b * T + t] - goal_y;
      sum += std::sqrt(dx * dx + dy * dy);
    }
    const double mean = sum / static_cast<double>(T);
    double cost = mean * weight;
    if (power != 1) cost = std::pow(cost, static_cast<double>(power));
    costs[b] += static_cast<float>(cost);
  }
}

// ── GoalAngleCritic CPU reference ────────────────────────────────────────
inline float normalize_angle_cpu(float a)
{
  while (a >  static_cast<float>(M_PI))  a -= 2.0f * static_cast<float>(M_PI);
  while (a <= -static_cast<float>(M_PI)) a += 2.0f * static_cast<float>(M_PI);
  return a;
}

void cpuGoalAngleCritic(
  unsigned int B, unsigned int T,
  const std::vector<float> & traj_yaws,
  float goal_yaw, bool symmetric,
  float weight, int power,
  std::vector<float> & costs)
{
  const float goal_yaw_sym = normalize_angle_cpu(goal_yaw + static_cast<float>(M_PI));
  for (unsigned int b = 0; b < B; ++b) {
    double sum = 0.0;
    for (unsigned int t = 0; t < T; ++t) {
      const float y = traj_yaws[b * T + t];
      float ad = std::fabs(normalize_angle_cpu(goal_yaw - y));
      if (symmetric) {
        float ad_sym = std::fabs(normalize_angle_cpu(goal_yaw_sym - y));
        ad = std::fmin(ad, ad_sym);
      }
      sum += ad;
    }
    const double mean = sum / static_cast<double>(T);
    double cost = mean * weight;
    if (power != 1) cost = std::pow(cost, static_cast<double>(power));
    costs[b] += static_cast<float>(cost);
  }
}

// ── PreferForwardCritic CPU reference ────────────────────────────────────
void cpuPreferForwardCritic(
  unsigned int B, unsigned int T,
  const std::vector<float> & state_vx,
  float model_dt, float weight, int power,
  std::vector<float> & costs)
{
  for (unsigned int b = 0; b < B; ++b) {
    double sum = 0.0;
    for (unsigned int t = 0; t < T; ++t) {
      sum += std::fmax(-state_vx[b * T + t], 0.0f) * model_dt;
    }
    double cost = sum * weight;
    if (power != 1) cost = std::pow(cost, static_cast<double>(power));
    costs[b] += static_cast<float>(cost);
  }
}

// ── ConstraintCritic CPU reference (DiffDrive only) ──────────────────────
void cpuConstraintCritic(
  unsigned int B, unsigned int T,
  const std::vector<float> & state_vx,
  const std::vector<float> & state_vy,
  float min_vel, float max_vel, float model_dt,
  float weight, int power,
  std::vector<float> & costs)
{
  for (unsigned int b = 0; b < B; ++b) {
    double sum = 0.0;
    for (unsigned int t = 0; t < T; ++t) {
      const float vx = state_vx[b * T + t];
      const float vy = state_vy.empty() ? 0.0f : state_vy[b * T + t];
      const float sgn = (vx > 0.0f) ? 1.0f : -1.0f;
      const float vel_total = sgn * std::sqrt(vx * vx + vy * vy);
      const float out_max = std::fmax(vel_total - max_vel, 0.0f);
      const float out_min = std::fmax(min_vel - vel_total, 0.0f);
      sum += (out_max + out_min) * model_dt;
    }
    double cost = sum * weight;
    if (power != 1) cost = std::pow(cost, static_cast<double>(power));
    costs[b] += static_cast<float>(cost);
  }
}

// ── PathFollowCritic CPU reference ───────────────────────────────────────
void cpuPathFollowCritic(
  unsigned int B, unsigned int T,
  const std::vector<float> & traj_x,
  const std::vector<float> & traj_y,
  float path_x, float path_y,
  float weight, int power,
  std::vector<float> & costs)
{
  for (unsigned int b = 0; b < B; ++b) {
    const float lx = traj_x[b * T + (T - 1)];
    const float ly = traj_y[b * T + (T - 1)];
    const float dx = lx - path_x;
    const float dy = ly - path_y;
    double cost = static_cast<double>(weight) * std::sqrt(dx * dx + dy * dy);
    if (power != 1) cost = std::pow(cost, static_cast<double>(power));
    costs[b] += static_cast<float>(cost);
  }
}

// ── PathAngleCritic CPU reference (default-yaml branch) ──────────────────
void cpuPathAngleCritic(
  unsigned int B, unsigned int T,
  const std::vector<float> & traj_x,
  const std::vector<float> & traj_y,
  const std::vector<float> & traj_yaws,
  float path_x, float path_y,
  float weight, int power,
  std::vector<float> & costs)
{
  for (unsigned int b = 0; b < B; ++b) {
    double sum = 0.0;
    for (unsigned int t = 0; t < T; ++t) {
      const size_t i = b * T + t;
      const float yaw_between =
        std::atan2(path_y - traj_y[i], path_x - traj_x[i]);
      sum += std::fabs(normalize_angle_cpu(yaw_between - traj_yaws[i]));
    }
    const double mean = sum / static_cast<double>(T);
    double cost = mean * weight;
    if (power != 1) cost = std::pow(cost, static_cast<double>(power));
    costs[b] += static_cast<float>(cost);
  }
}

// findClosestPathPt CPU reference (matches utils.hpp).
inline int find_closest_path_pt_cpu(
  const std::vector<float> & dists, float target, int init = 0)
{
  auto begin = dists.begin() + init;
  auto end   = dists.end();
  auto iter  = std::lower_bound(begin, end, target);
  if (iter == begin) return 0;
  if (iter == end) return static_cast<int>(dists.size() - 1);
  if (target - *(iter - 1) < *iter - target) {
    return static_cast<int>((iter - 1) - dists.begin());
  }
  return static_cast<int>(iter - dists.begin());
}

// ── PathAlignCritic CPU reference ────────────────────────────────────────
// Mirrors path_align_critic.cpp inner loop (the score()). All gates
// (within_position_goal_tolerance, offset_from_furthest, invalid_ctr
// over max_path_occupancy_ratio) are host-side; this body only runs the
// per-trajectory residual sum.
void cpuPathAlignCritic(
  unsigned int B, unsigned int T,
  unsigned int path_segments_count,
  unsigned int stride,
  bool use_path_orientations,
  const std::vector<float> & traj_x,
  const std::vector<float> & traj_y,
  const std::vector<float> & traj_yaws,
  const std::vector<float> & P_x,
  const std::vector<float> & P_y,
  const std::vector<float> & P_yaw,
  const std::vector<float> & path_int_dist,
  const std::vector<uint8_t> & path_pts_valid,
  float weight, int power,
  std::vector<float> & costs)
{
  for (unsigned int b = 0; b < B; ++b) {
    float traj_integrated_distance = 0.0f;
    float summed_path_dist = 0.0f;
    int num_samples = 0;
    int path_pt = 0;
    for (unsigned int p = stride; p < T; p += stride) {
      const float dx_t = traj_x[b * T + p] - traj_x[b * T + (p - stride)];
      const float dy_t = traj_y[b * T + p] - traj_y[b * T + (p - stride)];
      traj_integrated_distance += std::sqrt(dx_t * dx_t + dy_t * dy_t);
      // CPU passes prev path_pt as init for monotonic walk; for our test
      // dists are strictly increasing so init=0 gives the same answer.
      path_pt = find_closest_path_pt_cpu(path_int_dist, traj_integrated_distance, 0);
      if (static_cast<unsigned int>(path_pt) >= path_segments_count) {
        path_pt = static_cast<int>(path_segments_count) - 1;
      }
      if (path_pts_valid[path_pt] != 0) {
        const float dx = P_x[path_pt] - traj_x[b * T + p];
        const float dy = P_y[path_pt] - traj_y[b * T + p];
        float r2 = dx * dx + dy * dy;
        if (use_path_orientations) {
          const float dyaw = normalize_angle_cpu(P_yaw[path_pt] - traj_yaws[b * T + p]);
          r2 += dyaw * dyaw;
        }
        summed_path_dist += std::sqrt(r2);
        num_samples += 1;
      }
    }
    float mean = 0.0f;
    if (num_samples > 0) {
      mean = summed_path_dist / static_cast<float>(num_samples);
    }
    double cost = mean * weight;
    if (power != 1) cost = std::pow(cost, static_cast<double>(power));
    costs[b] += static_cast<float>(cost);
  }
}

// ── ObstaclesCritic CPU reference (no-footprint mode) ───────────────────
bool world_to_map_cpu(
  const nav_algo_mppi_cuda::CostmapInfo & cm, float wx, float wy,
  unsigned int & mx, unsigned int & my)
{
  if (wx < cm.origin_x || wy < cm.origin_y) return false;
  mx = static_cast<unsigned int>((wx - cm.origin_x) / cm.resolution);
  my = static_cast<unsigned int>((wy - cm.origin_y) / cm.resolution);
  return mx < cm.size_x && my < cm.size_y;
}

float distance_to_obstacle_cpu(
  float pose_cost, float scale, float circ_r, bool using_footprint)
{
  float d = (scale * circ_r - std::log(pose_cost) + std::log(253.0f)) / scale;
  if (!using_footprint) d -= circ_r;
  return d;
}

bool in_collision_cpu(float cost, bool track_unknown)
{
  if (cost >= 254.0f && cost <= 254.5f) return true;
  if (cost >= 254.5f && track_unknown) return true;
  return false;
}

float point_cost_cpu(
  const std::vector<uint8_t> & cm, const nav_algo_mppi_cuda::CostmapInfo & info,
  int x, int y)
{
  if (x < 0 || y < 0 || x >= static_cast<int>(info.size_x) ||
      y >= static_cast<int>(info.size_y))
  {
    return 254.0f;
  }
  return static_cast<float>(cm[y * info.size_x + x]);
}

float line_cost_cpu(
  const std::vector<uint8_t> & cm, const nav_algo_mppi_cuda::CostmapInfo & info,
  int x0, int y0, int x1, int y1)
{
  int dx  =  std::abs(x1 - x0);
  int dy  = -std::abs(y1 - y0);
  int sx  = x0 < x1 ? 1 : -1;
  int sy  = y0 < y1 ? 1 : -1;
  int err = dx + dy;
  float line_cost = 0.0f;
  while (true) {
    const float pc = point_cost_cpu(cm, info, x0, y0);
    if (pc == 254.0f) return pc;
    if (pc > line_cost) line_cost = pc;
    if (x0 == x1 && y0 == y1) break;
    int e2 = 2 * err;
    if (e2 >= dy) { err += dy; x0 += sx; }
    if (e2 <= dx) { err += dx; y0 += sy; }
  }
  return line_cost;
}

float footprint_cost_at_pose_cpu(
  const std::vector<float> & fp_x, const std::vector<float> & fp_y,
  float pose_x, float pose_y, float pose_theta,
  const std::vector<uint8_t> & cm, const nav_algo_mppi_cuda::CostmapInfo & info)
{
  const size_t n = fp_x.size();
  if (n < 2) return 0.0f;
  const float c = std::cos(pose_theta);
  const float s = std::sin(pose_theta);

  unsigned int x_first = 0, y_first = 0;
  {
    const float wx = pose_x + fp_x[0] * c - fp_y[0] * s;
    const float wy = pose_y + fp_x[0] * s + fp_y[0] * c;
    if (!world_to_map_cpu(info, wx, wy, x_first, y_first)) return 254.0f;
  }
  float fp_cost = 0.0f;
  unsigned int x_prev = x_first, y_prev = y_first;
  for (size_t i = 1; i < n; ++i) {
    const float wx = pose_x + fp_x[i] * c - fp_y[i] * s;
    const float wy = pose_y + fp_x[i] * s + fp_y[i] * c;
    unsigned int x_curr, y_curr;
    if (!world_to_map_cpu(info, wx, wy, x_curr, y_curr)) return 254.0f;
    const float lc = line_cost_cpu(cm, info,
      static_cast<int>(x_prev), static_cast<int>(y_prev),
      static_cast<int>(x_curr), static_cast<int>(y_curr));
    if (lc == 254.0f) return lc;
    if (lc > fp_cost) fp_cost = lc;
    x_prev = x_curr; y_prev = y_curr;
  }
  const float closing = line_cost_cpu(cm, info,
    static_cast<int>(x_prev),  static_cast<int>(y_prev),
    static_cast<int>(x_first), static_cast<int>(y_first));
  return std::fmax(fp_cost, closing);
}

void cpuObstaclesCritic(
  unsigned int B, unsigned int T,
  const nav_algo_mppi_cuda::ObstaclesConfig & cfg,
  const std::vector<float> & traj_x,
  const std::vector<float> & traj_y,
  const std::vector<float> & traj_yaws,
  const std::vector<float> & fp_x,
  const std::vector<float> & fp_y,
  const std::vector<uint8_t> & costmap,
  const nav_algo_mppi_cuda::CostmapInfo & cm_info,
  std::vector<float> & costs)
{
  for (unsigned int b = 0; b < B; ++b) {
    float traj_cost = 0.0f;
    float repulsive = 0.0f;
    bool collide = false;
    for (unsigned int t = 0; t < T; ++t) {
      const float x   = traj_x[b * T + t];
      const float y   = traj_y[b * T + t];
      const float yaw = traj_yaws[b * T + t];
      unsigned int mx, my;
      float pose_cost = 255.0f;
      if (world_to_map_cpu(cm_info, x, y, mx, my)) {
        pose_cost = static_cast<float>(costmap[my * cm_info.size_x + mx]);
      }
      bool using_footprint = false;
      if (cfg.consider_footprint && fp_x.size() >= 2 &&
          (pose_cost >= cfg.possibly_inscribed_cost ||
           cfg.possibly_inscribed_cost < 1.0f))
      {
        pose_cost = footprint_cost_at_pose_cpu(fp_x, fp_y, x, y, yaw, costmap, cm_info);
        using_footprint = true;
      }
      if (pose_cost < 1.0f) continue;
      if (in_collision_cpu(pose_cost, cfg.tracking_unknown)) {
        collide = true;
        break;
      }
      if (cfg.inflation_radius == 0.0f || cfg.inflation_scale_factor == 0.0f) continue;
      const float dist = distance_to_obstacle_cpu(
        pose_cost, cfg.inflation_scale_factor, cfg.circumscribed_radius,
        using_footprint);
      if (dist < cfg.collision_margin_distance) {
        traj_cost += cfg.collision_margin_distance - dist;
      } else if (!cfg.near_goal) {
        repulsive += cfg.inflation_radius - dist;
      }
    }
    const float raw = collide ? cfg.collision_cost : traj_cost;
    double cost = static_cast<double>(cfg.critical_weight) * raw
                + static_cast<double>(cfg.repulsion_weight)
                  * (repulsive / static_cast<double>(T));
    if (cfg.power != 1) cost = std::pow(cost, static_cast<double>(cfg.power));
    costs[b] += static_cast<float>(cost);
  }
}

void checkCuda(cudaError_t e, const char * msg)
{
  if (e != cudaSuccess) {
    std::fprintf(stderr, "CUDA error %s: %s\n", msg, cudaGetErrorString(e));
    std::exit(2);
  }
}

float maxAbsDiff(const std::vector<float> & a, const std::vector<float> & b)
{
  float m = 0.0f;
  for (size_t i = 0; i < a.size(); ++i) {
    m = std::fmax(m, std::fabs(a[i] - b[i]));
  }
  return m;
}

}  // namespace

int main()
{
  const unsigned int B = 2000;
  const unsigned int T = 56;
  const size_t N = B * T;

  // Synthetic inputs — uniform within Nav2's working envelope on Go2W.
  std::mt19937 rng(42);
  std::uniform_real_distribution<float> x_dist(-3.0f, 6.0f);
  std::uniform_real_distribution<float> y_dist(-3.0f, 3.0f);
  std::uniform_real_distribution<float> yaw_dist(-M_PI, M_PI);
  std::uniform_real_distribution<float> vx_dist(-0.10f, 0.30f);  // vx_min, vx_max
  std::uniform_real_distribution<float> vy_dist(-0.05f, 0.05f);
  std::vector<float> traj_x(N), traj_y(N), traj_yaws(N);
  std::vector<float> state_vx(N), state_vy(N);
  for (size_t i = 0; i < N; ++i) {
    traj_x[i]    = x_dist(rng);
    traj_y[i]    = y_dist(rng);
    traj_yaws[i] = yaw_dist(rng);
    state_vx[i]  = vx_dist(rng);
    state_vy[i]  = vy_dist(rng);
  }

  // Upload to GPU once; reused across critics.
  float *d_traj_x, *d_traj_y, *d_traj_yaws, *d_state_vx, *d_state_vy, *d_costs;
  size_t bytes_traj = N * sizeof(float);
  size_t bytes_costs = B * sizeof(float);
  checkCuda(cudaMalloc(&d_traj_x,    bytes_traj),  "alloc traj_x");
  checkCuda(cudaMalloc(&d_traj_y,    bytes_traj),  "alloc traj_y");
  checkCuda(cudaMalloc(&d_traj_yaws, bytes_traj),  "alloc traj_yaws");
  checkCuda(cudaMalloc(&d_state_vx,  bytes_traj),  "alloc state_vx");
  checkCuda(cudaMalloc(&d_state_vy,  bytes_traj),  "alloc state_vy");
  checkCuda(cudaMalloc(&d_costs,     bytes_costs), "alloc costs");
  checkCuda(cudaMemcpy(d_traj_x,    traj_x.data(),    bytes_traj, cudaMemcpyHostToDevice), "H2D traj_x");
  checkCuda(cudaMemcpy(d_traj_y,    traj_y.data(),    bytes_traj, cudaMemcpyHostToDevice), "H2D traj_y");
  checkCuda(cudaMemcpy(d_traj_yaws, traj_yaws.data(), bytes_traj, cudaMemcpyHostToDevice), "H2D traj_yaws");
  checkCuda(cudaMemcpy(d_state_vx,  state_vx.data(),  bytes_traj, cudaMemcpyHostToDevice), "H2D state_vx");
  checkCuda(cudaMemcpy(d_state_vy,  state_vy.data(),  bytes_traj, cudaMemcpyHostToDevice), "H2D state_vy");

  int total_fail = 0;

  // ── GoalCritic ────────────────────────────────────────────────────────
  {
    const float goal_x = 3.0f, goal_y = 0.0f;
    const float weight = 5.0f;
    const int power = 1;

    // CPU
    std::vector<float> cpu_costs(B, 0.0f);
    auto t0 = std::chrono::steady_clock::now();
    cpuGoalCritic(B, T, traj_x, traj_y, goal_x, goal_y, weight, power, cpu_costs);
    auto t1 = std::chrono::steady_clock::now();
    double cpu_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    // GPU
    checkCuda(cudaMemset(d_costs, 0, bytes_costs), "zero costs");
    nav_algo_mppi_cuda::CriticConfig cfg{B, T, power, weight};
    std::vector<double> ms_runs;
    for (int run = 0; run < 6; ++run) {
      checkCuda(cudaMemset(d_costs, 0, bytes_costs), "rezero");
      auto g0 = std::chrono::steady_clock::now();
      int rc = nav_algo_mppi_cuda::launchGoalCritic(
        cfg, d_traj_x, d_traj_y, goal_x, goal_y, d_costs);
      auto g1 = std::chrono::steady_clock::now();
      if (rc != 0) {
        std::fprintf(stderr, "GoalCritic launch failed: %d\n", rc);
        return 3;
      }
      if (run > 0) {
        ms_runs.push_back(std::chrono::duration<double, std::milli>(g1 - g0).count());
      }
    }
    std::sort(ms_runs.begin(), ms_runs.end());
    double gpu_ms = ms_runs[ms_runs.size() / 2];

    std::vector<float> gpu_costs(B);
    checkCuda(cudaMemcpy(gpu_costs.data(), d_costs, bytes_costs, cudaMemcpyDeviceToHost),
              "D2H costs");

    float d = maxAbsDiff(cpu_costs, gpu_costs);
    bool pass = d < 1e-4f;
    std::printf("GoalCritic            CPU %.3f ms | GPU %.3f ms | %.1f× | max|Δ| %.3e | %s\n",
                cpu_ms, gpu_ms, cpu_ms / gpu_ms, d, pass ? "PASS" : "FAIL");
    if (!pass) total_fail++;
  }

  // Helper lambda to run one critic case (CPU ref + GPU + diff).
  auto runCase = [&](const char * name,
                     auto && cpu_fn, auto && gpu_fn,
                     float weight, int power)
  {
    std::vector<float> cpu_costs(B, 0.0f);
    auto t0 = std::chrono::steady_clock::now();
    cpu_fn(cpu_costs);
    auto t1 = std::chrono::steady_clock::now();
    double cpu_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    nav_algo_mppi_cuda::CriticConfig cfg{B, T, power, weight};
    std::vector<double> ms_runs;
    for (int run = 0; run < 6; ++run) {
      checkCuda(cudaMemset(d_costs, 0, bytes_costs), "rezero");
      auto g0 = std::chrono::steady_clock::now();
      int rc = gpu_fn(cfg);
      auto g1 = std::chrono::steady_clock::now();
      if (rc != 0) {
        std::fprintf(stderr, "%s launch failed: %d\n", name, rc);
        return false;
      }
      if (run > 0) {
        ms_runs.push_back(std::chrono::duration<double, std::milli>(g1 - g0).count());
      }
    }
    std::sort(ms_runs.begin(), ms_runs.end());
    double gpu_ms = ms_runs[ms_runs.size() / 2];

    std::vector<float> gpu_costs(B);
    checkCuda(cudaMemcpy(gpu_costs.data(), d_costs, bytes_costs, cudaMemcpyDeviceToHost),
              "D2H costs");
    float d = maxAbsDiff(cpu_costs, gpu_costs);
    bool pass = d < 1e-4f;
    std::printf("%-22s CPU %.3f ms | GPU %.3f ms | %5.1f× | max|Δ| %.3e | %s\n",
                name, cpu_ms, gpu_ms, cpu_ms / gpu_ms, d, pass ? "PASS" : "FAIL");
    return pass;
  };

  // ── GoalAngleCritic ─────────────────────────────────────────────────────
  {
    const float goal_yaw = 0.4f;
    const float weight = 3.0f;
    const int power = 1;
    if (!runCase("GoalAngleCritic",
        [&](auto & out) { cpuGoalAngleCritic(B, T, traj_yaws, goal_yaw, false, weight, power, out); },
        [&](const auto & cfg) {
          return nav_algo_mppi_cuda::launchGoalAngleCritic(
            cfg, d_traj_yaws, goal_yaw, /*symmetric=*/false, d_costs);
        },
        weight, power)) total_fail++;
  }

  // ── PreferForwardCritic ─────────────────────────────────────────────────
  {
    const float weight = 5.0f;
    const int power = 1;
    const float model_dt = 0.05f;
    if (!runCase("PreferForwardCritic",
        [&](auto & out) { cpuPreferForwardCritic(B, T, state_vx, model_dt, weight, power, out); },
        [&](const auto & cfg) {
          return nav_algo_mppi_cuda::launchPreferForwardCritic(
            cfg, d_state_vx, model_dt, d_costs);
        },
        weight, power)) total_fail++;
  }

  // ── ConstraintCritic ────────────────────────────────────────────────────
  {
    const float vx_max = 0.30f, vy_max = 0.0f, vx_min = -0.10f;
    const float max_vel = std::sqrt(vx_max * vx_max + vy_max * vy_max);
    const float min_sgn = vx_min > 0.0f ? 1.0f : -1.0f;
    const float min_vel = min_sgn * std::sqrt(vx_min * vx_min + vy_max * vy_max);
    const float weight = 4.0f;
    const int power = 1;
    const float model_dt = 0.05f;
    // Pass empty vy vector to CPU (matches DiffDrive's holonomic=false path
    // — vy contribution is 0 since vy_max=0; ConstraintCritic still uses
    // state.vy in the magnitude. For our yaml vy_max=0 so vy is sampled
    // around 0 and the magnitude term degrades to |vx|.)
    if (!runCase("ConstraintCritic",
        [&](auto & out) { cpuConstraintCritic(B, T, state_vx, state_vy,
                                              min_vel, max_vel, model_dt, weight, power, out); },
        [&](const auto & cfg) {
          return nav_algo_mppi_cuda::launchConstraintCritic(
            cfg, d_state_vx, d_state_vy, min_vel, max_vel, model_dt, d_costs);
        },
        weight, power)) total_fail++;
  }

  // ── PathFollowCritic ────────────────────────────────────────────────────
  {
    const float path_x = 4.0f, path_y = 1.5f;
    const float weight = 5.0f;
    const int power = 1;
    if (!runCase("PathFollowCritic",
        [&](auto & out) { cpuPathFollowCritic(B, T, traj_x, traj_y, path_x, path_y,
                                              weight, power, out); },
        [&](const auto & cfg) {
          return nav_algo_mppi_cuda::launchPathFollowCritic(
            cfg, d_traj_x, d_traj_y, path_x, path_y, d_costs);
        },
        weight, power)) total_fail++;
  }

  // ── PathAngleCritic ─────────────────────────────────────────────────────
  {
    const float path_x = 4.0f, path_y = 1.5f;
    const float weight = 2.0f;
    const int power = 1;
    if (!runCase("PathAngleCritic",
        [&](auto & out) { cpuPathAngleCritic(B, T, traj_x, traj_y, traj_yaws,
                                             path_x, path_y, weight, power, out); },
        [&](const auto & cfg) {
          return nav_algo_mppi_cuda::launchPathAngleCritic(
            cfg, d_traj_x, d_traj_y, d_traj_yaws, path_x, path_y, d_costs);
        },
        weight, power)) total_fail++;
  }

  // ── PathAlignCritic ─────────────────────────────────────────────────────
  {
    // Synthetic path: 50 points on a straight line at y=1.5, x=0..4.9 (step
    // 0.1). All points valid. Integrated distances = 0, 0.1, 0.2, ...
    const unsigned int P = 50;
    std::vector<float> P_x(P), P_y(P), P_yaw(P), path_int_dist(P);
    std::vector<uint8_t> path_pts_valid(P, 1);
    for (unsigned int i = 0; i < P; ++i) {
      P_x[i] = static_cast<float>(i) * 0.1f;
      P_y[i] = 1.5f;
      P_yaw[i] = 0.0f;
      path_int_dist[i] = static_cast<float>(i) * 0.1f;
    }
    const unsigned int path_segments_count = P;
    const unsigned int stride = 4;
    const float weight = 14.0f;
    const int power = 1;
    const bool use_path_orientations = false;

    float *d_Px, *d_Py, *d_Pyaw, *d_pid;
    uint8_t *d_pv;
    checkCuda(cudaMalloc(&d_Px,  P * sizeof(float)),   "alloc Px");
    checkCuda(cudaMalloc(&d_Py,  P * sizeof(float)),   "alloc Py");
    checkCuda(cudaMalloc(&d_Pyaw, P * sizeof(float)),  "alloc Pyaw");
    checkCuda(cudaMalloc(&d_pid, P * sizeof(float)),   "alloc pid");
    checkCuda(cudaMalloc(&d_pv,  P * sizeof(uint8_t)), "alloc pv");
    checkCuda(cudaMemcpy(d_Px,  P_x.data(),   P * sizeof(float),   cudaMemcpyHostToDevice), "H2D Px");
    checkCuda(cudaMemcpy(d_Py,  P_y.data(),   P * sizeof(float),   cudaMemcpyHostToDevice), "H2D Py");
    checkCuda(cudaMemcpy(d_Pyaw, P_yaw.data(), P * sizeof(float),  cudaMemcpyHostToDevice), "H2D Pyaw");
    checkCuda(cudaMemcpy(d_pid, path_int_dist.data(), P * sizeof(float), cudaMemcpyHostToDevice), "H2D pid");
    checkCuda(cudaMemcpy(d_pv,  path_pts_valid.data(), P * sizeof(uint8_t), cudaMemcpyHostToDevice), "H2D pv");

    std::vector<float> cpu_costs(B, 0.0f);
    auto t0 = std::chrono::steady_clock::now();
    cpuPathAlignCritic(B, T, path_segments_count, stride, use_path_orientations,
                       traj_x, traj_y, traj_yaws, P_x, P_y, P_yaw,
                       path_int_dist, path_pts_valid, weight, power, cpu_costs);
    auto t1 = std::chrono::steady_clock::now();
    double cpu_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    nav_algo_mppi_cuda::PathAlignConfig cfg{
      B, T, path_segments_count, stride, power, weight, use_path_orientations};
    std::vector<double> ms_runs;
    for (int run = 0; run < 6; ++run) {
      checkCuda(cudaMemset(d_costs, 0, bytes_costs), "rezero");
      auto g0 = std::chrono::steady_clock::now();
      int rc = nav_algo_mppi_cuda::launchPathAlignCritic(
        cfg, d_traj_x, d_traj_y, d_traj_yaws,
        d_Px, d_Py, d_Pyaw, d_pid, d_pv, d_costs);
      auto g1 = std::chrono::steady_clock::now();
      if (rc != 0) { std::fprintf(stderr, "PathAlign launch failed: %d\n", rc); total_fail++; break; }
      if (run > 0) ms_runs.push_back(std::chrono::duration<double, std::milli>(g1 - g0).count());
    }
    std::sort(ms_runs.begin(), ms_runs.end());
    double gpu_ms = ms_runs.empty() ? 1.0 : ms_runs[ms_runs.size() / 2];

    std::vector<float> gpu_costs(B);
    checkCuda(cudaMemcpy(gpu_costs.data(), d_costs, bytes_costs, cudaMemcpyDeviceToHost),
              "D2H costs");
    float d = maxAbsDiff(cpu_costs, gpu_costs);
    bool pass = d < 1e-4f;
    std::printf("PathAlignCritic        CPU %.3f ms | GPU %.3f ms | %5.1f× | max|Δ| %.3e | %s\n",
                cpu_ms, gpu_ms, cpu_ms / gpu_ms, d, pass ? "PASS" : "FAIL");
    if (!pass) total_fail++;

    cudaFree(d_Px); cudaFree(d_Py); cudaFree(d_Pyaw); cudaFree(d_pid); cudaFree(d_pv);
  }

  // ── ObstaclesCritic ─────────────────────────────────────────────────────
  // Synthetic 100×100 costmap @ 0.10 m / cell (10 m × 10 m, origin at -5,-5).
  // Mostly free; sprinkle a few LETHAL cells + inflation ramp around them.
  {
    const unsigned int W = 100, H = 100;
    nav_algo_mppi_cuda::CostmapInfo cm_info{W, H, -5.0f, -5.0f, 0.10f};
    std::vector<uint8_t> costmap(W * H, 0);
    auto seed_lethal = [&](unsigned cx, unsigned cy) {
      for (int dy = -8; dy <= 8; ++dy) {
        for (int dx = -8; dx <= 8; ++dx) {
          int x = static_cast<int>(cx) + dx;
          int y = static_cast<int>(cy) + dy;
          if (x < 0 || y < 0 || x >= static_cast<int>(W) || y >= static_cast<int>(H)) continue;
          const float r = std::sqrt(static_cast<float>(dx * dx + dy * dy));
          uint8_t v = 0;
          if (r < 0.5f)      v = 254;
          else if (r < 3.0f) v = static_cast<uint8_t>(253.0f * std::exp(-0.3f * r));
          costmap[y * W + x] = std::max(costmap[y * W + x], v);
        }
      }
    };
    seed_lethal(50, 50);
    seed_lethal(60, 55);
    seed_lethal(40, 45);

    // Go2W footprint matching the bringup yaml: 0.64×0.36 m rectangle
    // centered on base_link. Vertices in robot frame.
    std::vector<float> fp_x = { 0.32f,  0.32f, -0.32f, -0.32f};
    std::vector<float> fp_y = { 0.18f, -0.18f, -0.18f,  0.18f};

    uint8_t * d_costmap;
    float *d_fp_x, *d_fp_y;
    checkCuda(cudaMalloc(&d_costmap, W * H * sizeof(uint8_t)), "alloc costmap");
    checkCuda(cudaMemcpy(d_costmap, costmap.data(), W * H * sizeof(uint8_t),
                         cudaMemcpyHostToDevice), "H2D costmap");
    checkCuda(cudaMalloc(&d_fp_x, fp_x.size() * sizeof(float)), "alloc fp_x");
    checkCuda(cudaMalloc(&d_fp_y, fp_y.size() * sizeof(float)), "alloc fp_y");
    checkCuda(cudaMemcpy(d_fp_x, fp_x.data(), fp_x.size() * sizeof(float),
                         cudaMemcpyHostToDevice), "H2D fp_x");
    checkCuda(cudaMemcpy(d_fp_y, fp_y.data(), fp_y.size() * sizeof(float),
                         cudaMemcpyHostToDevice), "H2D fp_y");

    auto run_obstacles_case = [&](const char * name, bool consider_footprint) {
      nav_algo_mppi_cuda::ObstaclesConfig cfg{};
      cfg.batch_size               = B;
      cfg.time_steps               = T;
      cfg.power                    = 1;
      cfg.critical_weight          = 20.0f;
      cfg.repulsion_weight         = 3.0f;
      cfg.collision_cost           = 10000.0f;
      cfg.collision_margin_distance = 0.05f;
      cfg.inflation_radius         = 0.20f;
      cfg.inflation_scale_factor   = 5.0f;
      cfg.circumscribed_radius     = 0.20f;
      cfg.possibly_inscribed_cost  = 100.0f;
      cfg.tracking_unknown         = true;
      cfg.near_goal                = false;
      cfg.consider_footprint       = consider_footprint;

      std::vector<float> cpu_costs(B, 0.0f);
      auto t0 = std::chrono::steady_clock::now();
      cpuObstaclesCritic(B, T, cfg, traj_x, traj_y, traj_yaws, fp_x, fp_y,
                         costmap, cm_info, cpu_costs);
      auto t1 = std::chrono::steady_clock::now();
      double cpu_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

      std::vector<double> ms_runs;
      for (int run = 0; run < 6; ++run) {
        checkCuda(cudaMemset(d_costs, 0, bytes_costs), "rezero");
        auto g0 = std::chrono::steady_clock::now();
        int rc = nav_algo_mppi_cuda::launchObstaclesCritic(
          cfg, d_traj_x, d_traj_y, d_traj_yaws, d_costmap, cm_info,
          d_fp_x, d_fp_y, static_cast<unsigned int>(fp_x.size()),
          d_costs);
        auto g1 = std::chrono::steady_clock::now();
        if (rc != 0) { std::fprintf(stderr, "%s launch failed: %d\n", name, rc); return false; }
        if (run > 0) ms_runs.push_back(std::chrono::duration<double, std::milli>(g1 - g0).count());
      }
      std::sort(ms_runs.begin(), ms_runs.end());
      double gpu_ms = ms_runs.empty() ? 1.0 : ms_runs[ms_runs.size() / 2];

      std::vector<float> gpu_costs(B);
      checkCuda(cudaMemcpy(gpu_costs.data(), d_costs, bytes_costs, cudaMemcpyDeviceToHost),
                "D2H costs");
      float d = maxAbsDiff(cpu_costs, gpu_costs);
      const float TOL = 1e-2f;
      bool pass = d < TOL;
      std::printf("%-22s CPU %.3f ms | GPU %.3f ms | %5.1f× | max|Δ| %.3e | %s\n",
                  name, cpu_ms, gpu_ms, cpu_ms / gpu_ms, d, pass ? "PASS" : "FAIL");
      return pass;
    };

    if (!run_obstacles_case("ObstaclesCritic(noFP)", false)) total_fail++;
    if (!run_obstacles_case("ObstaclesCritic(FP)",   true))  total_fail++;

    cudaFree(d_costmap); cudaFree(d_fp_x); cudaFree(d_fp_y);
  }

  cudaFree(d_traj_x); cudaFree(d_traj_y); cudaFree(d_traj_yaws);
  cudaFree(d_state_vx); cudaFree(d_state_vy); cudaFree(d_costs);
  return total_fail;
}
