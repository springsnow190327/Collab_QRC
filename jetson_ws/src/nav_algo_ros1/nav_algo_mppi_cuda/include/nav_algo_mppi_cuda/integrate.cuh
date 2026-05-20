// Copyright 2026 Collab_QRC
// SPDX-License-Identifier: Apache-2.0
//
// CUDA replacement for nav_algo_core::mppi::Optimizer::integrateStateVelocities
// (the 2-D State → Trajectories overload at optimizer.cpp:307-356).
//
// Math (verbatim from the CPU version):
//   yaws[b,t]   = yaw0 + cumsum_t(wz[b,t] * dt)
//   cos[b,t]    = cos(yaws[b,t-1])   for t≥1; cos(yaw0) at t=0
//   sin[b,t]    = sin(yaws[b,t-1])   for t≥1; sin(yaw0) at t=0
//   dx[b,t]     = vx[b,t]·cos[b,t]  (− vy·sin if holonomic)
//   dy[b,t]     = vx[b,t]·sin[b,t]  (+ vy·cos if holonomic)
//   x [b,t]     = x0  + cumsum_t(dx * dt)
//   y [b,t]     = y0  + cumsum_t(dy * dt)
//
// Kernel layout: B blocks × T threads. Each block handles one trajectory;
// the T threads cooperate on the cumsum via CUB BlockScan (no shared-mem
// race because all reads use t-1's value from the prefix-scan output).

#ifndef NAV_ALGO_MPPI_CUDA__INTEGRATE_CUH_
#define NAV_ALGO_MPPI_CUDA__INTEGRATE_CUH_

#include <cstddef>

namespace nav_algo_mppi_cuda
{

struct IntegrateConfig
{
  // Flat-tensor sizes — match xt::xtensor<float, 2> shape (B, T) on CPU.
  unsigned int batch_size;        // B
  unsigned int time_steps;        // T
  float        dt;                // model_dt
  float        initial_x;
  float        initial_y;
  float        initial_yaw;
  bool         holonomic;         // true → vy term active
};

// Launches the integrate kernel. All pointers are device-resident floats,
// row-major (b, t) → b*T + t. The output tensors (x, y, yaws) are written
// in place; the input velocity tensors (vx, vy, wz) are read-only.
//
// `vy_device` may be nullptr when cfg.holonomic == false (DiffDrive); the
// kernel skips the vy contribution in that case.
//
// Returns 0 on success, or the cudaError_t code on launch / sync failure.
int launchIntegrateStateVelocities(
  const IntegrateConfig & cfg,
  const float * vx_device,
  const float * vy_device,
  const float * wz_device,
  float       * x_device,
  float       * y_device,
  float       * yaws_device);

}  // namespace nav_algo_mppi_cuda

#endif  // NAV_ALGO_MPPI_CUDA__INTEGRATE_CUH_
