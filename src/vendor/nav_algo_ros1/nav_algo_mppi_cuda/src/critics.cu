// Copyright 2026 Collab_QRC
// SPDX-License-Identifier: Apache-2.0
//
// CUDA kernels for MPPI critics. See critics.cuh for the math contract.

#include "nav_algo_mppi_cuda/critics.cuh"
#include <cub/block/block_reduce.cuh>
#include <cuda_runtime.h>

namespace nav_algo_mppi_cuda
{
namespace
{

// Time-step block size. 64 covers Nav2's canonical 56-step horizon plus a
// little slack; threads with t >= time_steps mask their contribution to 0
// so the BlockReduce sum is still correct.
constexpr unsigned int kThreadsPerBlock = 64;

using BlockReduceFloat = cub::BlockReduce<float, kThreadsPerBlock>;

// ── GoalCritic kernel ────────────────────────────────────────────────────
// One block per trajectory; T threads cooperate on the per-trajectory
// mean-dist reduction via BlockReduce::Sum, then thread 0 emits cost[b].
__global__ void goalCriticKernel(
  unsigned int B, unsigned int T,
  const float * __restrict__ traj_x,
  const float * __restrict__ traj_y,
  float goal_x, float goal_y,
  float weight, int power,
  float * __restrict__ costs)
{
  const unsigned int b = blockIdx.x;
  const unsigned int t = threadIdx.x;
  if (b >= B) return;

  // Distance for this (b, t). Threads beyond T contribute 0 to keep the
  // BlockReduce sum well-defined; the mean divides by T (not the active
  // thread count) to match xt::mean's semantics over the full row.
  float dist = 0.0f;
  if (t < T) {
    const float dx = traj_x[b * T + t] - goal_x;
    const float dy = traj_y[b * T + t] - goal_y;
    dist = sqrtf(dx * dx + dy * dy);
  }

  __shared__ typename BlockReduceFloat::TempStorage tmp;
  const float sum = BlockReduceFloat(tmp).Sum(dist);

  if (t == 0) {
    const float mean = sum / static_cast<float>(T);
    float cost = mean * weight;
    if (power != 1) {
      cost = powf(cost, static_cast<float>(power));
    }
    // No atomic needed: one thread per block, one block per b.
    costs[b] += cost;
  }
}

// Normalize an angle to (-π, π]. Matches angles::normalize_angle's contract
// over the relevant range (we only ever feed it shortest-angular-distance
// inputs, so the |x| < 2π hot path is sufficient).
__device__ inline float normalizeAngle(float a)
{
  while (a >   static_cast<float>(M_PI)) a -= 2.0f * static_cast<float>(M_PI);
  while (a <= -static_cast<float>(M_PI)) a += 2.0f * static_cast<float>(M_PI);
  return a;
}

__device__ inline float shortestAngularDistance(float from, float to)
{
  return normalizeAngle(to - from);
}

// ── GoalAngleCritic kernel ───────────────────────────────────────────────
__global__ void goalAngleCriticKernel(
  unsigned int B, unsigned int T,
  const float * __restrict__ traj_yaws,
  float goal_yaw, float goal_yaw_sym, bool symmetric,
  float weight, int power,
  float * __restrict__ costs)
{
  const unsigned int b = blockIdx.x;
  const unsigned int t = threadIdx.x;
  if (b >= B) return;

  float ad = 0.0f;
  if (t < T) {
    const float y = traj_yaws[b * T + t];
    ad = fabsf(shortestAngularDistance(y, goal_yaw));
    if (symmetric) {
      const float ad_sym = fabsf(shortestAngularDistance(y, goal_yaw_sym));
      ad = fminf(ad, ad_sym);
    }
  }

  __shared__ typename BlockReduceFloat::TempStorage tmp;
  const float sum = BlockReduceFloat(tmp).Sum(ad);
  if (t == 0) {
    const float mean = sum / static_cast<float>(T);
    float cost = mean * weight;
    if (power != 1) cost = powf(cost, static_cast<float>(power));
    costs[b] += cost;
  }
}

// ── PreferForwardCritic kernel ───────────────────────────────────────────
__global__ void preferForwardCriticKernel(
  unsigned int B, unsigned int T,
  const float * __restrict__ state_vx,
  float model_dt,
  float weight, int power,
  float * __restrict__ costs)
{
  const unsigned int b = blockIdx.x;
  const unsigned int t = threadIdx.x;
  if (b >= B) return;

  float backward_dt = 0.0f;
  if (t < T) {
    const float vx = state_vx[b * T + t];
    backward_dt = fmaxf(-vx, 0.0f) * model_dt;
  }

  __shared__ typename BlockReduceFloat::TempStorage tmp;
  const float sum = BlockReduceFloat(tmp).Sum(backward_dt);
  if (t == 0) {
    float cost = sum * weight;
    if (power != 1) cost = powf(cost, static_cast<float>(power));
    costs[b] += cost;
  }
}

// ── ConstraintCritic kernel ──────────────────────────────────────────────
__global__ void constraintCriticKernel(
  unsigned int B, unsigned int T,
  const float * __restrict__ state_vx,
  const float * __restrict__ state_vy,  // may be nullptr for non-holonomic
  float min_vel, float max_vel, float model_dt,
  float weight, int power,
  float * __restrict__ costs)
{
  const unsigned int b = blockIdx.x;
  const unsigned int t = threadIdx.x;
  if (b >= B) return;

  float term_dt = 0.0f;
  if (t < T) {
    const float vx = state_vx[b * T + t];
    const float vy = (state_vy != nullptr) ? state_vy[b * T + t] : 0.0f;
    const float sgn = (vx > 0.0f) ? 1.0f : -1.0f;
    const float vel_total = sgn * sqrtf(vx * vx + vy * vy);
    const float out_max = fmaxf(vel_total - max_vel, 0.0f);
    const float out_min = fmaxf(min_vel - vel_total, 0.0f);
    term_dt = (out_max + out_min) * model_dt;
  }

  __shared__ typename BlockReduceFloat::TempStorage tmp;
  const float sum = BlockReduceFloat(tmp).Sum(term_dt);
  if (t == 0) {
    float cost = sum * weight;
    if (power != 1) cost = powf(cost, static_cast<float>(power));
    costs[b] += cost;
  }
}

// ── ObstaclesCritic kernel (no-footprint mode) ───────────────────────────
// Constants from costmap_2d/cost_values.h (and our compat alias):
//   FREE_SPACE                  = 0
//   INSCRIBED_INFLATED_OBSTACLE = 253
//   LETHAL_OBSTACLE             = 254
//   NO_INFORMATION              = 255
__device__ inline bool worldToMapGpu(
  const CostmapInfo & cm, float wx, float wy, unsigned int & mx, unsigned int & my)
{
  if (wx < cm.origin_x || wy < cm.origin_y) return false;
  mx = static_cast<unsigned int>((wx - cm.origin_x) / cm.resolution);
  my = static_cast<unsigned int>((wy - cm.origin_y) / cm.resolution);
  return mx < cm.size_x && my < cm.size_y;
}

__device__ inline float distanceToObstacleGpu(
  float pose_cost, float scale_factor, float min_radius,
  float circumscribed_radius, bool using_footprint)
{
  float dist = (scale_factor * min_radius - logf(pose_cost) + logf(253.0f)) / scale_factor;
  if (!using_footprint) {
    dist -= circumscribed_radius;
  }
  return dist;
}

// ── Footprint Bresenham helpers ──────────────────────────────────────────
// Mirror of nav2_costmap_2d::FootprintCollisionChecker::lineCost +
// pointCost, but operating on a __device__ costmap buffer.
__device__ inline float pointCostGpu(
  const uint8_t * cm, const CostmapInfo & info, int x, int y)
{
  if (x < 0 || y < 0 || x >= static_cast<int>(info.size_x) ||
      y >= static_cast<int>(info.size_y))
  {
    return 254.0f;  // LETHAL — matches CPU's lineCost early-exit semantics
  }
  return static_cast<float>(cm[y * info.size_x + x]);
}

__device__ inline float lineCostGpu(
  const uint8_t * cm, const CostmapInfo & info, int x0, int y0, int x1, int y1)
{
  // Bresenham line cell rasterisation (Wikipedia variant — same cell set as
  // Nav2's LineIterator, traced verbatim in the FootprintCollisionChecker
  // CPU port). Returns the max cell cost along the line.
  int dx  =  abs(x1 - x0);
  int dy  = -abs(y1 - y0);
  int sx  = x0 < x1 ? 1 : -1;
  int sy  = y0 < y1 ? 1 : -1;
  int err = dx + dy;
  float line_cost = 0.0f;
  while (true) {
    const float pc = pointCostGpu(cm, info, x0, y0);
    if (pc == 254.0f) return pc;  // early-exit at LETHAL
    if (pc > line_cost) line_cost = pc;
    if (x0 == x1 && y0 == y1) break;
    int e2 = 2 * err;
    if (e2 >= dy) { err += dy; x0 += sx; }
    if (e2 <= dx) { err += dx; y0 += sy; }
  }
  return line_cost;
}

// Footprint cost at pose (x, y, theta). Footprint is in robot frame; rotate
// + translate to world, worldToMap each vertex, Bresenham each edge plus
// the closing segment, return max cost. Matches FootprintCollisionChecker::
// footprintCostAtPose in nav_algo_core/compat.hpp.
__device__ inline float footprintCostAtPoseGpu(
  const float * fp_x, const float * fp_y, unsigned int fp_n,
  float pose_x, float pose_y, float pose_theta,
  const uint8_t * cm, const CostmapInfo & info)
{
  if (fp_n < 2 || fp_x == nullptr || fp_y == nullptr) return 0.0f;
  const float c = cosf(pose_theta);
  const float s = sinf(pose_theta);

  // First vertex (cache for closing edge).
  unsigned int x_first = 0, y_first = 0;
  {
    const float wx = pose_x + fp_x[0] * c - fp_y[0] * s;
    const float wy = pose_y + fp_x[0] * s + fp_y[0] * c;
    if (!worldToMapGpu(info, wx, wy, x_first, y_first)) return 254.0f;
  }

  float fp_cost = 0.0f;
  unsigned int x_prev = x_first, y_prev = y_first;
  for (unsigned int i = 1; i < fp_n; ++i) {
    const float wx = pose_x + fp_x[i] * c - fp_y[i] * s;
    const float wy = pose_y + fp_x[i] * s + fp_y[i] * c;
    unsigned int x_curr, y_curr;
    if (!worldToMapGpu(info, wx, wy, x_curr, y_curr)) return 254.0f;
    const float lc = lineCostGpu(
      cm, info,
      static_cast<int>(x_prev), static_cast<int>(y_prev),
      static_cast<int>(x_curr), static_cast<int>(y_curr));
    if (lc == 254.0f) return lc;
    if (lc > fp_cost) fp_cost = lc;
    x_prev = x_curr;
    y_prev = y_curr;
  }
  // Closing edge (last → first).
  const float closing = lineCostGpu(
    cm, info,
    static_cast<int>(x_prev),  static_cast<int>(y_prev),
    static_cast<int>(x_first), static_cast<int>(y_first));
  if (closing > fp_cost) fp_cost = closing;
  return fp_cost;
}

__device__ inline bool inCollisionGpu(float cost, bool tracking_unknown)
{
  // LETHAL_OBSTACLE = 254. NO_INFORMATION (255) blocks only when tracking
  // unknown space (Nav2 default in our yaml: track_unknown_space=true → so
  // 255 is treated as lethal).
  if (cost >= 254.0f && cost <= 254.5f) return true;
  if (cost >= 254.5f && tracking_unknown) return true;
  return false;
}

__global__ void obstaclesCriticKernel(
  unsigned int B, unsigned int T,
  ObstaclesConfig cfg,
  const float * __restrict__ traj_x,
  const float * __restrict__ traj_y,
  const float * __restrict__ traj_yaws,
  const uint8_t * __restrict__ costmap,
  CostmapInfo cm_info,
  const float * __restrict__ fp_x,
  const float * __restrict__ fp_y,
  unsigned int fp_n,
  float * __restrict__ costs)
{
  const unsigned int b = blockIdx.x;
  const unsigned int t = threadIdx.x;
  if (b >= B) return;
  const bool active = (t < T);

  // ── Phase 1: per-thread cost-at-pose + decide collision / margin / repulsion
  // Direct port of ObstaclesCritic::costAtPose (obstacles_critic.cpp):
  //   1. Single-cell pointCost first (OOB → NO_INFORMATION = 255).
  //   2. If consider_footprint AND single-cell ≥ possibly_inscribed_cost,
  //      escalate to footprintCostAtPose. The single-cell early-out skips
  //      the Bresenham work for the vast majority of free-space poses.
  float pose_cost = 0.0f;
  bool  using_footprint = false;
  bool  local_collide = false;
  float local_traj_cost = 0.0f;
  float local_repulsive = 0.0f;

  if (active) {
    const unsigned int idx = b * T + t;
    const float x   = traj_x[idx];
    const float y   = traj_y[idx];
    const float yaw = traj_yaws[idx];

    unsigned int mx, my;
    if (worldToMapGpu(cm_info, x, y, mx, my)) {
      pose_cost = static_cast<float>(costmap[my * cm_info.size_x + mx]);
    } else {
      pose_cost = 255.0f;
    }

    // Footprint escalation (matches CPU: cost ≥ possibly_inscribed_cost OR
    // possibly_inscribed_cost < 1, the latter being the "no inflation
    // layer" sentinel).
    if (cfg.consider_footprint && fp_n >= 2 &&
        (pose_cost >= cfg.possibly_inscribed_cost ||
         cfg.possibly_inscribed_cost < 1.0f))
    {
      pose_cost = footprintCostAtPoseGpu(
        fp_x, fp_y, fp_n, x, y, yaw, costmap, cm_info);
      using_footprint = true;
    }

    if (pose_cost >= 1.0f) {
      if (inCollisionGpu(pose_cost, cfg.tracking_unknown)) {
        local_collide = true;
      } else if (cfg.inflation_radius > 0.0f && cfg.inflation_scale_factor > 0.0f) {
        const float dist = distanceToObstacleGpu(
          pose_cost, cfg.inflation_scale_factor, cfg.circumscribed_radius,
          cfg.circumscribed_radius, using_footprint);
        if (dist < cfg.collision_margin_distance) {
          local_traj_cost = cfg.collision_margin_distance - dist;
        } else if (!cfg.near_goal) {
          local_repulsive = cfg.inflation_radius - dist;
        }
      }
    }
  }

  // ── Phase 2: first-collision-t via atomicMin → mask later contributions
  __shared__ int first_collide_t;
  if (t == 0) first_collide_t = static_cast<int>(T);
  __syncthreads();
  if (local_collide) atomicMin(&first_collide_t, static_cast<int>(t));
  __syncthreads();

  const bool gate = (static_cast<int>(t) < first_collide_t);
  const float traj_cost_masked = gate ? local_traj_cost : 0.0f;
  const float repulsive_masked = gate ? local_repulsive : 0.0f;

  __shared__ typename BlockReduceFloat::TempStorage sum_tc;
  __shared__ typename BlockReduceFloat::TempStorage sum_rp;
  const float total_traj_cost = BlockReduceFloat(sum_tc).Sum(traj_cost_masked);
  const float total_repulsive = BlockReduceFloat(sum_rp).Sum(repulsive_masked);

  if (t == 0) {
    const bool collide_any = (first_collide_t < static_cast<int>(T));
    const float raw = collide_any ? cfg.collision_cost : total_traj_cost;
    const float repulsive_per_T = total_repulsive / static_cast<float>(T);
    float cost = cfg.critical_weight * raw + cfg.repulsion_weight * repulsive_per_T;
    if (cfg.power != 1) cost = powf(cost, static_cast<float>(cfg.power));
    costs[b] += cost;
  }
}

// ── PathAlignCritic kernel ──────────────────────────────────────────────
// One block per trajectory, T threads. Threads with `p = t * stride` such
// that p < T and (t*stride) % stride == 0 (i.e. t < T/stride) are active
// and each handles one stride point.
//
// To match the CPU's mono-monotonic accumulation of traj_integrated_distance,
// each thread computes its own prefix sum over j = 1 .. t (own loop). For
// stride=4 T=56 → 14 active threads, max j = 14 → 196 sequential mults per
// thread = trivial.
//
// findClosestPathPt mirrors std::lower_bound + nearer-of-iter/iter-1 logic.
__device__ inline int findClosestPathPtGpu(
  const float * dists, int n, float target)
{
  // lower_bound: first idx where dists[idx] >= target.
  int lo = 0, hi = n;
  while (lo < hi) {
    int mid = (lo + hi) >> 1;
    if (dists[mid] < target) lo = mid + 1;
    else hi = mid;
  }
  if (lo == 0) return 0;  // matches CPU's "iter == begin + init → 0"
  if (lo == n) return n - 1;
  // Pick closer of lo and lo-1
  const float d_lo = dists[lo]     - target;
  const float d_hi = target - dists[lo - 1];
  return (d_hi < d_lo) ? lo - 1 : lo;
}

__global__ void pathAlignCriticKernel(
  unsigned int B, unsigned int T,
  unsigned int path_segments_count,
  unsigned int stride,
  bool use_path_orientations,
  const float * __restrict__ traj_x,
  const float * __restrict__ traj_y,
  const float * __restrict__ traj_yaws,
  const float * __restrict__ P_x,
  const float * __restrict__ P_y,
  const float * __restrict__ P_yaw,
  const float * __restrict__ path_int_dist,
  const uint8_t * __restrict__ path_pts_valid,
  float weight, int power,
  float * __restrict__ costs)
{
  const unsigned int b = blockIdx.x;
  const unsigned int t = threadIdx.x;
  if (b >= B) return;

  // Active iff this thread maps to a valid stride point p = (t+1)*stride < T.
  // Matches the CPU loop bound `p = stride; p < T; p += stride`, i.e.
  //   thread 0 → p=stride, thread 1 → p=2*stride, ...
  const unsigned int p = (t + 1) * stride;
  const bool active = (p < T) && (stride > 0);

  float local_sum   = 0.0f;
  int   local_count = 0;

  if (active) {
    // Compute traj_integrated_distance up to stride point (t+1).
    // CPU sums dist(p_{j*stride}, p_{(j-1)*stride}) for j = 1..(t+1).
    float td = 0.0f;
    for (unsigned int j = 1; j <= (t + 1); ++j) {
      const unsigned int pj  = j * stride;
      const unsigned int pjm = (j - 1) * stride;
      const float dx = traj_x[b * T + pj]  - traj_x[b * T + pjm];
      const float dy = traj_y[b * T + pj]  - traj_y[b * T + pjm];
      td += sqrtf(dx * dx + dy * dy);
    }

    // Find matching path point.
    const int path_pt = findClosestPathPtGpu(
      path_int_dist, static_cast<int>(path_segments_count), td);

    if (path_pts_valid[path_pt] != 0) {
      const float dx = P_x[path_pt] - traj_x[b * T + p];
      const float dy = P_y[path_pt] - traj_y[b * T + p];
      float r2 = dx * dx + dy * dy;
      if (use_path_orientations && traj_yaws != nullptr && P_yaw != nullptr) {
        const float dyaw =
          shortestAngularDistance(traj_yaws[b * T + p], P_yaw[path_pt]);
        r2 += dyaw * dyaw;
      }
      local_sum   = sqrtf(r2);
      local_count = 1;
    }
  }

  // Block reductions.
  __shared__ typename BlockReduceFloat::TempStorage sum_tmp;
  using BlockReduceInt = cub::BlockReduce<int, kThreadsPerBlock>;
  __shared__ typename BlockReduceInt::TempStorage cnt_tmp;
  const float total_sum = BlockReduceFloat(sum_tmp).Sum(local_sum);
  const int   total_cnt = BlockReduceInt(cnt_tmp).Sum(local_count);

  if (t == 0) {
    float mean = 0.0f;
    if (total_cnt > 0) {
      mean = total_sum / static_cast<float>(total_cnt);
    }
    float cost = mean * weight;
    if (power != 1) cost = powf(cost, static_cast<float>(power));
    costs[b] += cost;
  }
}

// ── PathFollowCritic kernel ──────────────────────────────────────────────
// Uses only last trajectory column (t = T-1). One thread per block.
__global__ void pathFollowCriticKernel(
  unsigned int B, unsigned int T,
  const float * __restrict__ traj_x,
  const float * __restrict__ traj_y,
  float path_x, float path_y,
  float weight, int power,
  float * __restrict__ costs)
{
  const unsigned int b = blockIdx.x;
  if (b >= B) return;
  if (threadIdx.x != 0) return;

  const unsigned int idx = b * T + (T - 1);
  const float dx = traj_x[idx] - path_x;
  const float dy = traj_y[idx] - path_y;
  const float dist = sqrtf(dx * dx + dy * dy);

  float cost = weight * dist;
  if (power != 1) cost = powf(cost, static_cast<float>(power));
  costs[b] += cost;
}

// ── PathAngleCritic kernel (default-yaml branch only) ────────────────────
__global__ void pathAngleCriticKernel(
  unsigned int B, unsigned int T,
  const float * __restrict__ traj_x,
  const float * __restrict__ traj_y,
  const float * __restrict__ traj_yaws,
  float path_x, float path_y,
  float weight, int power,
  float * __restrict__ costs)
{
  const unsigned int b = blockIdx.x;
  const unsigned int t = threadIdx.x;
  if (b >= B) return;

  float yaw_diff = 0.0f;
  if (t < T) {
    const unsigned int idx = b * T + t;
    const float yaw_between = atan2f(path_y - traj_y[idx], path_x - traj_x[idx]);
    yaw_diff = fabsf(shortestAngularDistance(traj_yaws[idx], yaw_between));
  }

  __shared__ typename BlockReduceFloat::TempStorage tmp;
  const float sum = BlockReduceFloat(tmp).Sum(yaw_diff);
  if (t == 0) {
    const float mean = sum / static_cast<float>(T);
    float cost = mean * weight;
    if (power != 1) cost = powf(cost, static_cast<float>(power));
    costs[b] += cost;
  }
}

}  // namespace

int launchGoalCritic(
  const CriticConfig & cfg,
  const float * traj_x_device,
  const float * traj_y_device,
  float         goal_x,
  float         goal_y,
  float       * costs_device)
{
  if (cfg.time_steps > kThreadsPerBlock) {
    return cudaErrorInvalidConfiguration;
  }

  goalCriticKernel<<<cfg.batch_size, kThreadsPerBlock>>>(
    cfg.batch_size, cfg.time_steps,
    traj_x_device, traj_y_device,
    goal_x, goal_y,
    cfg.weight, cfg.power,
    costs_device);

  cudaError_t err = cudaPeekAtLastError();
  if (err != cudaSuccess) return static_cast<int>(err);
  err = cudaDeviceSynchronize();
  return static_cast<int>(err);
}

int launchGoalAngleCritic(
  const CriticConfig & cfg,
  const float * traj_yaws_device,
  float         goal_yaw,
  bool          symmetric_yaw_tolerance,
  float       * costs_device)
{
  if (cfg.time_steps > kThreadsPerBlock) return cudaErrorInvalidConfiguration;

  // Pre-compute symmetric goal yaw normalize to (-π, π] on the host once.
  float goal_yaw_sym = goal_yaw + static_cast<float>(M_PI);
  while (goal_yaw_sym >  static_cast<float>(M_PI))  goal_yaw_sym -= 2.0f * static_cast<float>(M_PI);
  while (goal_yaw_sym <= -static_cast<float>(M_PI)) goal_yaw_sym += 2.0f * static_cast<float>(M_PI);

  goalAngleCriticKernel<<<cfg.batch_size, kThreadsPerBlock>>>(
    cfg.batch_size, cfg.time_steps,
    traj_yaws_device,
    goal_yaw, goal_yaw_sym, symmetric_yaw_tolerance,
    cfg.weight, cfg.power,
    costs_device);

  cudaError_t err = cudaPeekAtLastError();
  if (err != cudaSuccess) return static_cast<int>(err);
  return static_cast<int>(cudaDeviceSynchronize());
}

int launchPreferForwardCritic(
  const CriticConfig & cfg,
  const float * state_vx_device,
  float         model_dt,
  float       * costs_device)
{
  if (cfg.time_steps > kThreadsPerBlock) return cudaErrorInvalidConfiguration;

  preferForwardCriticKernel<<<cfg.batch_size, kThreadsPerBlock>>>(
    cfg.batch_size, cfg.time_steps,
    state_vx_device, model_dt,
    cfg.weight, cfg.power,
    costs_device);

  cudaError_t err = cudaPeekAtLastError();
  if (err != cudaSuccess) return static_cast<int>(err);
  return static_cast<int>(cudaDeviceSynchronize());
}

int launchConstraintCritic(
  const CriticConfig & cfg,
  const float * state_vx_device,
  const float * state_vy_device,
  float         min_vel,
  float         max_vel,
  float         model_dt,
  float       * costs_device)
{
  if (cfg.time_steps > kThreadsPerBlock) return cudaErrorInvalidConfiguration;

  constraintCriticKernel<<<cfg.batch_size, kThreadsPerBlock>>>(
    cfg.batch_size, cfg.time_steps,
    state_vx_device, state_vy_device,
    min_vel, max_vel, model_dt,
    cfg.weight, cfg.power,
    costs_device);

  cudaError_t err = cudaPeekAtLastError();
  if (err != cudaSuccess) return static_cast<int>(err);
  return static_cast<int>(cudaDeviceSynchronize());
}

int launchPathFollowCritic(
  const CriticConfig & cfg,
  const float * traj_x_device,
  const float * traj_y_device,
  float         path_x,
  float         path_y,
  float       * costs_device)
{
  // One thread per block; T can be anything (we only read column T-1).
  pathFollowCriticKernel<<<cfg.batch_size, 1>>>(
    cfg.batch_size, cfg.time_steps,
    traj_x_device, traj_y_device,
    path_x, path_y,
    cfg.weight, cfg.power,
    costs_device);

  cudaError_t err = cudaPeekAtLastError();
  if (err != cudaSuccess) return static_cast<int>(err);
  return static_cast<int>(cudaDeviceSynchronize());
}

int launchPathAngleCritic(
  const CriticConfig & cfg,
  const float * traj_x_device,
  const float * traj_y_device,
  const float * traj_yaws_device,
  float         path_x,
  float         path_y,
  float       * costs_device)
{
  if (cfg.time_steps > kThreadsPerBlock) return cudaErrorInvalidConfiguration;

  pathAngleCriticKernel<<<cfg.batch_size, kThreadsPerBlock>>>(
    cfg.batch_size, cfg.time_steps,
    traj_x_device, traj_y_device, traj_yaws_device,
    path_x, path_y,
    cfg.weight, cfg.power,
    costs_device);

  cudaError_t err = cudaPeekAtLastError();
  if (err != cudaSuccess) return static_cast<int>(err);
  return static_cast<int>(cudaDeviceSynchronize());
}

int launchObstaclesCritic(
  const ObstaclesConfig & cfg,
  const float * traj_x_device,
  const float * traj_y_device,
  const float * traj_yaws_device,
  const uint8_t * costmap_device,
  CostmapInfo costmap_info,
  const float * footprint_x_device,
  const float * footprint_y_device,
  unsigned int  footprint_n,
  float       * costs_device)
{
  if (cfg.time_steps > kThreadsPerBlock) return cudaErrorInvalidConfiguration;

  obstaclesCriticKernel<<<cfg.batch_size, kThreadsPerBlock>>>(
    cfg.batch_size, cfg.time_steps, cfg,
    traj_x_device, traj_y_device, traj_yaws_device,
    costmap_device, costmap_info,
    footprint_x_device, footprint_y_device, footprint_n,
    costs_device);

  cudaError_t err = cudaPeekAtLastError();
  if (err != cudaSuccess) return static_cast<int>(err);
  return static_cast<int>(cudaDeviceSynchronize());
}

int launchPathAlignCritic(
  const PathAlignConfig & cfg,
  const float * traj_x_device,
  const float * traj_y_device,
  const float * traj_yaws_device,
  const float * path_x_device,
  const float * path_y_device,
  const float * path_yaws_device,
  const float * path_int_dist_device,
  const uint8_t * path_pts_valid_device,
  float       * costs_device)
{
  if (cfg.time_steps > kThreadsPerBlock) return cudaErrorInvalidConfiguration;

  pathAlignCriticKernel<<<cfg.batch_size, kThreadsPerBlock>>>(
    cfg.batch_size, cfg.time_steps,
    cfg.path_segments_count,
    cfg.trajectory_point_step,
    cfg.use_path_orientations,
    traj_x_device, traj_y_device, traj_yaws_device,
    path_x_device, path_y_device, path_yaws_device,
    path_int_dist_device, path_pts_valid_device,
    cfg.weight, cfg.power,
    costs_device);

  cudaError_t err = cudaPeekAtLastError();
  if (err != cudaSuccess) return static_cast<int>(err);
  return static_cast<int>(cudaDeviceSynchronize());
}

}  // namespace nav_algo_mppi_cuda
