// Copyright 2026 Collab_QRC
// SPDX-License-Identifier: Apache-2.0
//
// CUDA replacement for nav_algo_core::mppi::Optimizer::updateControlSequence
// (optimizer.cpp:356-388). Three phases, three launchers:
//
//   1. Cost-shaping bias term — for each dimension d ∈ {vx, vy, wz}:
//        costs[b] += (gamma / std_d²) * sum_t(control_d[t] * (state.cv_d[b,t] - control_d[t]))
//      One call per active dimension (DiffDrive yaml skips vy).
//
//   2. Softmax over costs[B] — produces softmax[B]:
//        m = min(costs); e[b] = exp(-(costs[b]-m)/temperature)
//        softmax[b] = e[b] / sum(e)
//
//   3. Weighted average per T column — produces control[T]:
//        control_d[t] = sum_b state.cv_d[b,t] * softmax[b]
//      One call per dimension.
//
// Constraint clamping (applyControlSequenceConstraints, T elements) stays
// host-side; trivially small.

#ifndef NAV_ALGO_MPPI_CUDA__CONTROL_UPDATE_CUH_
#define NAV_ALGO_MPPI_CUDA__CONTROL_UPDATE_CUH_

#include <cstddef>

namespace nav_algo_mppi_cuda
{

struct ControlUpdateConfig
{
  unsigned int batch_size;   // B
  unsigned int time_steps;   // T
  float        temperature;  // yaml temperature
  float        gamma;        // yaml gamma
  float        std_vx;       // yaml vx_std (for cost-shape weight)
  float        std_vy;       // yaml vy_std
  float        std_wz;       // yaml wz_std
};

// One dimension at a time. control_dim_device: T floats. state_bt_device:
// B*T floats (row-major, b slow). costs_device: B floats (modified in-place).
int launchCostShape(
  const ControlUpdateConfig & cfg,
  const float * control_dim_device,
  const float * state_bt_device,
  float         std_dim,                  // pass cfg.std_vx/vy/wz
  float       * costs_device);

// Softmax over B → softmax_device. costs_device unchanged.
int launchSoftmax(
  const ControlUpdateConfig & cfg,
  const float * costs_device,
  float       * softmax_device);

// Weighted average: control_dim_device[t] = sum_b state[b,t] * softmax[b].
int launchWeightedAvg(
  const ControlUpdateConfig & cfg,
  const float * state_bt_device,
  const float * softmax_device,
  float       * control_dim_device);

}  // namespace nav_algo_mppi_cuda

#endif  // NAV_ALGO_MPPI_CUDA__CONTROL_UPDATE_CUH_
