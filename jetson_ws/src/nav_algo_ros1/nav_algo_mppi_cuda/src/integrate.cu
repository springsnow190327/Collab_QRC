// Copyright 2026 Collab_QRC
// SPDX-License-Identifier: Apache-2.0
//
// CUDA integrate kernel for MPPI rollouts. See integrate.cuh for the math.

#include "nav_algo_mppi_cuda/integrate.cuh"
#include <cub/block/block_scan.cuh>
#include <cuda_runtime.h>

namespace nav_algo_mppi_cuda
{
namespace
{

// T must equal blockDim.x. We pick 64 to cover the canonical Nav2 config
// (time_steps=56). Tunable for other horizons by changing this constant +
// recompile. Power-of-two avoids CUB BlockScan slow-path.
constexpr unsigned int kThreadsPerBlock = 64;

using BlockScanFloat = cub::BlockScan<float, kThreadsPerBlock>;

__global__ void integrateKernel(
  IntegrateConfig cfg,
  const float * __restrict__ vx,
  const float * __restrict__ vy,
  const float * __restrict__ wz,
  float       * __restrict__ x,
  float       * __restrict__ y,
  float       * __restrict__ yaws)
{
  const unsigned int b = blockIdx.x;       // trajectory index
  const unsigned int t = threadIdx.x;      // time-step index within trajectory
  if (b >= cfg.batch_size) return;
  const bool active = (t < cfg.time_steps);

  const unsigned int idx = b * cfg.time_steps + t;

  // ── Stage 1: cumsum of wz·dt → yaws ──────────────────────────────────────
  float wz_dt = active ? wz[idx] * cfg.dt : 0.0f;
  __shared__ typename BlockScanFloat::TempStorage scan_tmp;
  float yaw_offset;
  BlockScanFloat(scan_tmp).InclusiveSum(wz_dt, yaw_offset);
  float yaw_t = cfg.initial_yaw + yaw_offset;
  if (active) yaws[idx] = yaw_t;
  __syncthreads();

  // ── Stage 2: cos/sin of "previous" yaw ───────────────────────────────────
  // The CPU reference uses yaws_cutted = yaws[:, 0:-1]; at t=0 it uses
  // initial_yaw. Mirror that explicitly to match the CPU output 1:1.
  float prev_yaw = (t == 0) ? cfg.initial_yaw
                            : (yaws[b * cfg.time_steps + t - 1]);
  // ↑ At t==0, prev_yaw uses initial_yaw (matches CPU).
  // At t>0, read from yaws[t-1] — already written + visible after sync.

  float c = cosf(prev_yaw);
  float s = sinf(prev_yaw);

  // ── Stage 3: dx, dy with optional holonomic vy term ──────────────────────
  float vx_t = active ? vx[idx] : 0.0f;
  float vy_t = (cfg.holonomic && vy != nullptr && active) ? vy[idx] : 0.0f;
  float dx_t = vx_t * c;
  float dy_t = vx_t * s;
  if (cfg.holonomic) {
    dx_t -= vy_t * s;
    dy_t += vy_t * c;
  }

  // ── Stage 4: cumsum of dx·dt, dy·dt → x, y ──────────────────────────────
  __shared__ typename BlockScanFloat::TempStorage scan_tmp_x;
  __shared__ typename BlockScanFloat::TempStorage scan_tmp_y;
  float x_offset, y_offset;
  BlockScanFloat(scan_tmp_x).InclusiveSum(dx_t * cfg.dt, x_offset);
  BlockScanFloat(scan_tmp_y).InclusiveSum(dy_t * cfg.dt, y_offset);

  if (active) {
    x[idx] = cfg.initial_x + x_offset;
    y[idx] = cfg.initial_y + y_offset;
  }
}

}  // namespace

int launchIntegrateStateVelocities(
  const IntegrateConfig & cfg,
  const float * vx_device,
  const float * vy_device,
  const float * wz_device,
  float       * x_device,
  float       * y_device,
  float       * yaws_device)
{
  if (cfg.time_steps > kThreadsPerBlock) {
    // Out of supported range — caller must rebuild with larger
    // kThreadsPerBlock or fall back to CPU. Mirrors the CPU-side
    // shape-assertion in Optimizer::optimize.
    return cudaErrorInvalidConfiguration;
  }

  dim3 grid(cfg.batch_size);
  dim3 block(kThreadsPerBlock);

  integrateKernel<<<grid, block>>>(
    cfg, vx_device, vy_device, wz_device,
    x_device, y_device, yaws_device);

  cudaError_t err = cudaPeekAtLastError();
  if (err != cudaSuccess) return static_cast<int>(err);
  err = cudaDeviceSynchronize();
  return static_cast<int>(err);
}

}  // namespace nav_algo_mppi_cuda
