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

}  // namespace nav_algo_mppi_cuda
