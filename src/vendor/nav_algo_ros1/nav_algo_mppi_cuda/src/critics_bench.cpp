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

  cudaFree(d_traj_x); cudaFree(d_traj_y); cudaFree(d_traj_yaws);
  cudaFree(d_state_vx); cudaFree(d_state_vy); cudaFree(d_costs);
  return total_fail;
}
