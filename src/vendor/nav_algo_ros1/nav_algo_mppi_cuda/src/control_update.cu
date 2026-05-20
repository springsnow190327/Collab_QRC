// Copyright 2026 Collab_QRC
// SPDX-License-Identifier: Apache-2.0
//
// CUDA kernels for MPPI control-update step. See control_update.cuh.

#include "nav_algo_mppi_cuda/control_update.cuh"
#include <cub/block/block_reduce.cuh>
#include <cuda_runtime.h>

namespace nav_algo_mppi_cuda
{
namespace
{

// Per-trajectory cost-shape: 1 block per b, T threads cooperate on sum_t.
constexpr unsigned int kCostShapeThreads = 64;
using CSReduce = cub::BlockReduce<float, kCostShapeThreads>;

__global__ void costShapeKernel(
  unsigned int B, unsigned int T,
  const float * __restrict__ control_dim,
  const float * __restrict__ state_bt,
  float gamma_over_std2,
  float * __restrict__ costs)
{
  const unsigned int b = blockIdx.x;
  const unsigned int t = threadIdx.x;
  if (b >= B) return;

  float local = 0.0f;
  if (t < T) {
    const float c  = control_dim[t];
    const float bn = state_bt[b * T + t] - c;
    local = c * bn;
  }

  __shared__ typename CSReduce::TempStorage tmp;
  const float sum = CSReduce(tmp).Sum(local);
  if (t == 0) {
    costs[b] += gamma_over_std2 * sum;
  }
}

// Softmax over B in one block. Grid-stride loop over B handles B > blockDim.
constexpr unsigned int kSoftmaxThreads = 256;
using SMReduce = cub::BlockReduce<float, kSoftmaxThreads>;

__global__ void softmaxKernel(
  unsigned int B, float temperature,
  const float * __restrict__ costs,
  float * __restrict__ softmax)
{
  const unsigned int tid = threadIdx.x;
  const unsigned int stride = blockDim.x;

  // Phase 1: find min(costs[B])
  float local_min = INFINITY;
  for (unsigned int i = tid; i < B; i += stride) {
    local_min = fminf(local_min, costs[i]);
  }
  __shared__ typename SMReduce::TempStorage tmp_min;
  const float block_min = SMReduce(tmp_min).Reduce(local_min, cub::Min());
  __shared__ float min_shared;
  if (tid == 0) min_shared = block_min;
  __syncthreads();
  const float min_cost = min_shared;

  // Phase 2: compute exp((costs - min) / -temp), accumulate sum
  float local_sum = 0.0f;
  const float inv_neg_temp = -1.0f / temperature;
  for (unsigned int i = tid; i < B; i += stride) {
    const float ev = expf(inv_neg_temp * (costs[i] - min_cost));
    softmax[i] = ev;
    local_sum += ev;
  }
  __shared__ typename SMReduce::TempStorage tmp_sum;
  const float block_sum = SMReduce(tmp_sum).Sum(local_sum);
  __shared__ float sum_shared;
  if (tid == 0) sum_shared = block_sum;
  __syncthreads();
  const float total = sum_shared;
  // Defensive: if all costs are infinite the sum is 0; avoid div-by-0.
  if (total <= 0.0f) {
    const float uniform = 1.0f / static_cast<float>(B);
    for (unsigned int i = tid; i < B; i += stride) {
      softmax[i] = uniform;
    }
    return;
  }

  // Phase 3: normalize
  for (unsigned int i = tid; i < B; i += stride) {
    softmax[i] /= total;
  }
}

// Weighted average per T column. T blocks, 256 threads each.
constexpr unsigned int kWAvgThreads = 256;
using WAReduce = cub::BlockReduce<float, kWAvgThreads>;

__global__ void weightedAvgKernel(
  unsigned int B, unsigned int T,
  const float * __restrict__ state_bt,
  const float * __restrict__ softmax,
  float * __restrict__ control_dim)
{
  const unsigned int t = blockIdx.x;
  if (t >= T) return;
  const unsigned int tid = threadIdx.x;
  const unsigned int stride = blockDim.x;

  float local = 0.0f;
  for (unsigned int b = tid; b < B; b += stride) {
    local += state_bt[b * T + t] * softmax[b];
  }
  __shared__ typename WAReduce::TempStorage tmp;
  const float total = WAReduce(tmp).Sum(local);
  if (tid == 0) {
    control_dim[t] = total;
  }
}

}  // namespace

int launchCostShape(
  const ControlUpdateConfig & cfg,
  const float * control_dim_device,
  const float * state_bt_device,
  float         std_dim,
  float       * costs_device)
{
  if (cfg.time_steps > kCostShapeThreads) return cudaErrorInvalidConfiguration;
  const float gamma_over_std2 = cfg.gamma / (std_dim * std_dim);
  costShapeKernel<<<cfg.batch_size, kCostShapeThreads>>>(
    cfg.batch_size, cfg.time_steps,
    control_dim_device, state_bt_device,
    gamma_over_std2, costs_device);
  cudaError_t err = cudaPeekAtLastError();
  if (err != cudaSuccess) return static_cast<int>(err);
  return static_cast<int>(cudaDeviceSynchronize());
}

int launchSoftmax(
  const ControlUpdateConfig & cfg,
  const float * costs_device,
  float       * softmax_device)
{
  softmaxKernel<<<1, kSoftmaxThreads>>>(
    cfg.batch_size, cfg.temperature,
    costs_device, softmax_device);
  cudaError_t err = cudaPeekAtLastError();
  if (err != cudaSuccess) return static_cast<int>(err);
  return static_cast<int>(cudaDeviceSynchronize());
}

int launchWeightedAvg(
  const ControlUpdateConfig & cfg,
  const float * state_bt_device,
  const float * softmax_device,
  float       * control_dim_device)
{
  weightedAvgKernel<<<cfg.time_steps, kWAvgThreads>>>(
    cfg.batch_size, cfg.time_steps,
    state_bt_device, softmax_device, control_dim_device);
  cudaError_t err = cudaPeekAtLastError();
  if (err != cudaSuccess) return static_cast<int>(err);
  return static_cast<int>(cudaDeviceSynchronize());
}

}  // namespace nav_algo_mppi_cuda
