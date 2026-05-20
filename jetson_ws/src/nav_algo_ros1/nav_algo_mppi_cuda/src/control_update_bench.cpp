// Copyright 2026 Collab_QRC
// SPDX-License-Identifier: Apache-2.0
//
// CPU-vs-GPU bench for the 3 control-update kernels (cost-shape, softmax,
// weighted-avg). CPU refs match Optimizer::updateControlSequence exactly.
// Sequence ordering and the (gamma/std²) multiplier must reproduce the
// final cmd_vel that Nav2 sim's MPPI emits.

#include "nav_algo_mppi_cuda/control_update.cuh"

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

// ── Cost-shape CPU reference ──────────────────────────────────────────────
void cpuCostShape(
  unsigned int B, unsigned int T,
  const std::vector<float> & control_dim,
  const std::vector<float> & state_bt,
  float gamma, float std_dim,
  std::vector<float> & costs)
{
  const double gos2 = static_cast<double>(gamma) / (static_cast<double>(std_dim) * std_dim);
  for (unsigned int b = 0; b < B; ++b) {
    double sum = 0.0;
    for (unsigned int t = 0; t < T; ++t) {
      const double c  = control_dim[t];
      const double bn = state_bt[b * T + t] - c;
      sum += c * bn;
    }
    costs[b] += static_cast<float>(gos2 * sum);
  }
}

// ── Softmax CPU reference ─────────────────────────────────────────────────
void cpuSoftmax(
  unsigned int B, float temperature,
  const std::vector<float> & costs,
  std::vector<float> & softmax)
{
  float m = INFINITY;
  for (unsigned int i = 0; i < B; ++i) m = std::fmin(m, costs[i]);
  double total = 0.0;
  softmax.assign(B, 0.0f);
  for (unsigned int i = 0; i < B; ++i) {
    const double e = std::exp(-(static_cast<double>(costs[i]) - m) / temperature);
    softmax[i] = static_cast<float>(e);
    total += e;
  }
  if (total <= 0.0) {
    const float uniform = 1.0f / static_cast<float>(B);
    for (auto & v : softmax) v = uniform;
    return;
  }
  for (unsigned int i = 0; i < B; ++i) softmax[i] = static_cast<float>(softmax[i] / total);
}

// ── Weighted-average CPU reference ────────────────────────────────────────
void cpuWeightedAvg(
  unsigned int B, unsigned int T,
  const std::vector<float> & state_bt,
  const std::vector<float> & softmax,
  std::vector<float> & control_dim)
{
  control_dim.assign(T, 0.0f);
  for (unsigned int t = 0; t < T; ++t) {
    double sum = 0.0;
    for (unsigned int b = 0; b < B; ++b) {
      sum += static_cast<double>(state_bt[b * T + t]) * softmax[b];
    }
    control_dim[t] = static_cast<float>(sum);
  }
}

}  // namespace

int main()
{
  const unsigned int B = 2000;
  const unsigned int T = 56;
  const size_t Nbt = B * T;

  // Random inputs in plausible ranges.
  std::mt19937 rng(42);
  std::uniform_real_distribution<float> ctrl_dist(-0.20f, 0.30f);
  std::uniform_real_distribution<float> state_dist(-0.30f, 0.30f);
  std::uniform_real_distribution<float> cost_dist(0.0f, 100.0f);
  std::vector<float> control_seq(T);
  std::vector<float> state_bt(Nbt);
  std::vector<float> costs_init(B);
  for (auto & v : control_seq) v = ctrl_dist(rng);
  for (auto & v : state_bt)     v = state_dist(rng);
  for (auto & v : costs_init)   v = cost_dist(rng);

  nav_algo_mppi_cuda::ControlUpdateConfig cfg{};
  cfg.batch_size  = B;
  cfg.time_steps  = T;
  cfg.temperature = 0.3f;
  cfg.gamma       = 0.015f;
  cfg.std_vx      = 0.20f;
  cfg.std_vy      = 0.20f;
  cfg.std_wz      = 0.40f;

  // Upload to GPU.
  float *d_control, *d_state, *d_costs, *d_softmax, *d_control_out;
  checkCuda(cudaMalloc(&d_control,     T   * sizeof(float)), "alloc control");
  checkCuda(cudaMalloc(&d_state,       Nbt * sizeof(float)), "alloc state");
  checkCuda(cudaMalloc(&d_costs,       B   * sizeof(float)), "alloc costs");
  checkCuda(cudaMalloc(&d_softmax,     B   * sizeof(float)), "alloc softmax");
  checkCuda(cudaMalloc(&d_control_out, T   * sizeof(float)), "alloc control_out");
  checkCuda(cudaMemcpy(d_control, control_seq.data(), T   * sizeof(float), cudaMemcpyHostToDevice), "H2D control");
  checkCuda(cudaMemcpy(d_state,   state_bt.data(),     Nbt * sizeof(float), cudaMemcpyHostToDevice), "H2D state");

  int total_fail = 0;

  // ── Cost-shape ──────────────────────────────────────────────────────────
  {
    std::vector<float> cpu_costs = costs_init;
    auto t0 = std::chrono::steady_clock::now();
    cpuCostShape(B, T, control_seq, state_bt, cfg.gamma, cfg.std_vx, cpu_costs);
    auto t1 = std::chrono::steady_clock::now();
    double cpu_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    std::vector<double> ms_runs;
    for (int run = 0; run < 6; ++run) {
      checkCuda(cudaMemcpy(d_costs, costs_init.data(), B * sizeof(float), cudaMemcpyHostToDevice), "re-init costs");
      auto g0 = std::chrono::steady_clock::now();
      int rc = nav_algo_mppi_cuda::launchCostShape(cfg, d_control, d_state, cfg.std_vx, d_costs);
      auto g1 = std::chrono::steady_clock::now();
      if (rc != 0) { std::fprintf(stderr, "CostShape failed: %d\n", rc); total_fail++; break; }
      if (run > 0) ms_runs.push_back(std::chrono::duration<double, std::milli>(g1 - g0).count());
    }
    std::sort(ms_runs.begin(), ms_runs.end());
    double gpu_ms = ms_runs.empty() ? 1.0 : ms_runs[ms_runs.size()/2];

    std::vector<float> gpu_costs(B);
    checkCuda(cudaMemcpy(gpu_costs.data(), d_costs, B * sizeof(float), cudaMemcpyDeviceToHost), "D2H costs");
    float d = maxAbsDiff(cpu_costs, gpu_costs);
    const float TOL = 1e-3f;
    bool pass = d < TOL;
    std::printf("CostShape              CPU %.3f ms | GPU %.3f ms | %5.1f× | max|Δ| %.3e | %s\n",
                cpu_ms, gpu_ms, cpu_ms / gpu_ms, d, pass ? "PASS" : "FAIL");
    if (!pass) total_fail++;
  }

  // ── Softmax ─────────────────────────────────────────────────────────────
  {
    // Use post-cost-shape costs (already on GPU). Mirror on CPU.
    std::vector<float> cpu_costs = costs_init;
    cpuCostShape(B, T, control_seq, state_bt, cfg.gamma, cfg.std_vx, cpu_costs);

    std::vector<float> cpu_softmax;
    auto t0 = std::chrono::steady_clock::now();
    cpuSoftmax(B, cfg.temperature, cpu_costs, cpu_softmax);
    auto t1 = std::chrono::steady_clock::now();
    double cpu_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    std::vector<double> ms_runs;
    for (int run = 0; run < 6; ++run) {
      auto g0 = std::chrono::steady_clock::now();
      int rc = nav_algo_mppi_cuda::launchSoftmax(cfg, d_costs, d_softmax);
      auto g1 = std::chrono::steady_clock::now();
      if (rc != 0) { std::fprintf(stderr, "Softmax failed: %d\n", rc); total_fail++; break; }
      if (run > 0) ms_runs.push_back(std::chrono::duration<double, std::milli>(g1 - g0).count());
    }
    std::sort(ms_runs.begin(), ms_runs.end());
    double gpu_ms = ms_runs.empty() ? 1.0 : ms_runs[ms_runs.size()/2];

    std::vector<float> gpu_softmax(B);
    checkCuda(cudaMemcpy(gpu_softmax.data(), d_softmax, B * sizeof(float), cudaMemcpyDeviceToHost), "D2H softmax");
    float d = maxAbsDiff(cpu_softmax, gpu_softmax);
    const float TOL = 1e-5f;
    bool pass = d < TOL;
    std::printf("Softmax                CPU %.3f ms | GPU %.3f ms | %5.1f× | max|Δ| %.3e | %s\n",
                cpu_ms, gpu_ms, cpu_ms / gpu_ms, d, pass ? "PASS" : "FAIL");
    if (!pass) total_fail++;
  }

  // ── WeightedAvg ─────────────────────────────────────────────────────────
  {
    std::vector<float> cpu_costs = costs_init;
    cpuCostShape(B, T, control_seq, state_bt, cfg.gamma, cfg.std_vx, cpu_costs);
    std::vector<float> cpu_softmax;
    cpuSoftmax(B, cfg.temperature, cpu_costs, cpu_softmax);
    std::vector<float> cpu_control;
    auto t0 = std::chrono::steady_clock::now();
    cpuWeightedAvg(B, T, state_bt, cpu_softmax, cpu_control);
    auto t1 = std::chrono::steady_clock::now();
    double cpu_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    std::vector<double> ms_runs;
    for (int run = 0; run < 6; ++run) {
      auto g0 = std::chrono::steady_clock::now();
      int rc = nav_algo_mppi_cuda::launchWeightedAvg(cfg, d_state, d_softmax, d_control_out);
      auto g1 = std::chrono::steady_clock::now();
      if (rc != 0) { std::fprintf(stderr, "WeightedAvg failed: %d\n", rc); total_fail++; break; }
      if (run > 0) ms_runs.push_back(std::chrono::duration<double, std::milli>(g1 - g0).count());
    }
    std::sort(ms_runs.begin(), ms_runs.end());
    double gpu_ms = ms_runs.empty() ? 1.0 : ms_runs[ms_runs.size()/2];

    std::vector<float> gpu_control(T);
    checkCuda(cudaMemcpy(gpu_control.data(), d_control_out, T * sizeof(float), cudaMemcpyDeviceToHost), "D2H control");
    float d = maxAbsDiff(cpu_control, gpu_control);
    const float TOL = 1e-5f;
    bool pass = d < TOL;
    std::printf("WeightedAvg            CPU %.3f ms | GPU %.3f ms | %5.1f× | max|Δ| %.3e | %s\n",
                cpu_ms, gpu_ms, cpu_ms / gpu_ms, d, pass ? "PASS" : "FAIL");
    if (!pass) total_fail++;
  }

  cudaFree(d_control); cudaFree(d_state); cudaFree(d_costs);
  cudaFree(d_softmax); cudaFree(d_control_out);
  return total_fail;
}
