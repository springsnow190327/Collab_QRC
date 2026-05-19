// Copyright 2026 Collab_QRC
// SPDX-License-Identifier: Apache-2.0
//
// CUDA host-side launchers for the 8 MPPI critic kernels that ports the
// `score()` body of nav_algo_core/src/mppi/critics/*.cpp onto the GPU.
//
// All critics share the same shape: read trajectories (B×T x, y, yaws),
// optionally read path (P x, y, yaws, dists) + state pose, write into a
// device-resident costs[B] vector that the rest of the optimizer pipeline
// reads back at the end of one evalControl() iteration.
//
// Convention: kernels ACCUMULATE into costs[B] (just like CPU `data.costs +=`),
// so the host is responsible for zeroing costs[B] once per cycle before
// launching the critic sequence.

#ifndef NAV_ALGO_MPPI_CUDA__CRITICS_CUH_
#define NAV_ALGO_MPPI_CUDA__CRITICS_CUH_

#include <cstdint>

namespace nav_algo_mppi_cuda
{

struct CriticConfig
{
  unsigned int batch_size;
  unsigned int time_steps;
  int          power;          // cost_power yaml field, almost always 1
  float        weight;         // cost_weight (or repulsion/critical for obstacles)
};

// ── GoalCritic ────────────────────────────────────────────────────────────
// CPU body (goal_critic.cpp):
//   if (!within_position_goal_tolerance(threshold, state.pose, path)) return;
//   goal_x, goal_y = path[last]
//   dists[b,t] = sqrt((traj_x - goal_x)² + (traj_y - goal_y)²)
//   costs[b] += pow(mean_t(dists[b,:]) * weight, power)
//
// The threshold gate is host-side: caller checks robot↔goal distance and
// either calls this launcher or skips (no kernel launch). 0 cost when gated.
int launchGoalCritic(
  const CriticConfig & cfg,
  const float * traj_x_device,
  const float * traj_y_device,
  float         goal_x,
  float         goal_y,
  float       * costs_device);

// ── GoalAngleCritic ──────────────────────────────────────────────────────
// CPU body (goal_angle_critic.cpp):
//   if (!within_position_goal_tolerance(threshold, state.pose, path)) return;
//   goal_yaw = path.yaws[last]
//   ad[b,t] = abs(shortest_angular_distance(traj_yaws[b,t], goal_yaw))
//   if symmetric: ad[b,t] = min(ad, abs(shortest_angular_distance(yaw, goal_yaw + π)))
//   costs[b] += pow(mean_t(ad[b,:]) * weight, power)
int launchGoalAngleCritic(
  const CriticConfig & cfg,
  const float * traj_yaws_device,
  float         goal_yaw,
  bool          symmetric_yaw_tolerance,
  float       * costs_device);

// ── PreferForwardCritic ──────────────────────────────────────────────────
// CPU body (prefer_forward_critic.cpp):
//   if (within_position_goal_tolerance(threshold, state.pose, path)) return;
//                                                                  ^ NOT inverted: skip when within
//   backward[b,t] = max(-state.vx[b,t], 0)
//   costs[b] += pow(sum_t(backward * model_dt) * weight, power)
int launchPreferForwardCritic(
  const CriticConfig & cfg,
  const float * state_vx_device,
  float         model_dt,
  float       * costs_device);

// ── ConstraintCritic ─────────────────────────────────────────────────────
// CPU body (constraint_critic.cpp), DiffDrive branch (Ackermann path not
// in this port since yaml uses DiffDrive):
//   vel_total[b,t] = sgn(vx) * sqrt(vx² + vy²)
//   out_max[b,t]   = max(vel_total - max_vel, 0)
//   out_min[b,t]   = max(min_vel - vel_total, 0)
//   costs[b]      += pow(sum_t((out_max + out_min) * model_dt) * weight, power)
//
// max_vel / min_vel are host-derived from yaml vx/vy max/min:
//   max_vel = sqrt(vx_max² + vy_max²)
//   min_vel = sgn(vx_min) * sqrt(vx_min² + vy_max²)
int launchConstraintCritic(
  const CriticConfig & cfg,
  const float * state_vx_device,
  const float * state_vy_device,
  float         min_vel,
  float         max_vel,
  float         model_dt,
  float       * costs_device);

// ── PathFollowCritic ─────────────────────────────────────────────────────
// CPU body (path_follow_critic.cpp):
//   Pick `offseted_idx = min(furthest_reached + offset_from_furthest, path.size-1)`
//   then walk forward while !path_pts_valid[i] (skip invalid points).
//   goal_x, goal_y = path[offseted_idx]
//   dist[b] = sqrt((last_traj_x - goal_x)² + (last_traj_y - goal_y)²)
//   costs[b] += pow(weight * dist[b], power)
//
// The offset-and-skip logic is host-side; host passes the final (goal_x,
// goal_y). The kernel uses ONLY the last trajectory point — no T dimension
// to reduce over. Single thread per block writes the cost.
int launchPathFollowCritic(
  const CriticConfig & cfg,
  const float * traj_x_device,
  const float * traj_y_device,
  float         path_x,
  float         path_y,
  float       * costs_device);

// ── PathAngleCritic ──────────────────────────────────────────────────────
// CPU body (path_angle_critic.cpp), default-yaml path (forward_preference=true
// so the reversing_allowed branch is skipped):
//   Pick offseted_idx (host-side)
//   yaws_between[b,t] = atan2(goal_y - traj_y, goal_x - traj_x)
//   yaws[b,t]         = abs(shortest_angular_distance(traj_yaws, yaws_between))
//   costs[b] += pow(mean_t(yaws) * weight, power)
//
// Host-side gate: skip if `posePointAngle(robot_pose, goal_x, goal_y) <
// max_angle_to_furthest`.
int launchPathAngleCritic(
  const CriticConfig & cfg,
  const float * traj_x_device,
  const float * traj_y_device,
  const float * traj_yaws_device,
  float         path_x,
  float         path_y,
  float       * costs_device);

}  // namespace nav_algo_mppi_cuda

#endif  // NAV_ALGO_MPPI_CUDA__CRITICS_CUH_
