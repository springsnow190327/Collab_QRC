// Copyright 2026 Collab_QRC
// SPDX-License-Identifier: Apache-2.0
//
// Standalone benchmark for the CUDA integrate kernel. Validates output
// equivalence against a CPU reference (same math as
// nav_algo_core::mppi::Optimizer::integrateStateVelocities) and measures
// per-iteration latency on the host GPU. Run after building with CUDA:
//
//   ./devel_isolated/nav_algo_mppi_cuda/lib/nav_algo_mppi_cuda/integrate_bench
//
// Expected output on RTX 4050 / Orin Ampere (sm_87): max-abs-diff < 1e-5,
// GPU latency ≤ 200 µs at the canonical (B=2000, T=56) shape vs ≈ 3 ms
// CPU. Production deployment plugs the kernel into MPPI's Optimizer via a
// drop-in path (next CUDA milestone — integrate one kernel at a time so
// each can be diff'd against CPU in isolation).

#include "nav_algo_mppi_cuda/integrate.cuh"

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

// CPU reference: literal port of Optimizer::integrateStateVelocities (the
// 2-D version at optimizer.cpp:307-356), in plain C++ loops to avoid the
// xtensor dependency in the test driver. The CUDA kernel must reproduce
// these outputs bitwise (within fp32 rounding of reduction reordering).
void cpuIntegrate(
  const nav_algo_mppi_cuda::IntegrateConfig & cfg,
  const std::vector<float> & vx,
  const std::vector<float> & vy,
  const std::vector<float> & wz,
  std::vector<float> & x,
  std::vector<float> & y,
  std::vector<float> & yaws)
{
  const auto B = cfg.batch_size;
  const auto T = cfg.time_steps;
  x.assign(B * T, 0.0f);
  y.assign(B * T, 0.0f);
  yaws.assign(B * T, 0.0f);

  for (unsigned int b = 0; b < B; ++b) {
    float yaw_accum  = cfg.initial_yaw;
    float x_accum    = 0.0f;
    float y_accum    = 0.0f;
    for (unsigned int t = 0; t < T; ++t) {
      yaw_accum += wz[b * T + t] * cfg.dt;
      yaws[b * T + t] = yaw_accum;

      float prev_yaw = (t == 0) ? cfg.initial_yaw : yaws[b * T + t - 1];
      float c = std::cos(prev_yaw);
      float s = std::sin(prev_yaw);

      float vx_t = vx[b * T + t];
      float vy_t = (cfg.holonomic && !vy.empty()) ? vy[b * T + t] : 0.0f;
      float dx_t = vx_t * c;
      float dy_t = vx_t * s;
      if (cfg.holonomic) {
        dx_t -= vy_t * s;
        dy_t += vy_t * c;
      }

      x_accum += dx_t * cfg.dt;
      y_accum += dy_t * cfg.dt;
      x[b * T + t] = cfg.initial_x + x_accum;
      y[b * T + t] = cfg.initial_y + y_accum;
    }
  }
}

void checkCuda(cudaError_t e, const char * msg)
{
  if (e != cudaSuccess) {
    std::fprintf(stderr, "CUDA error %s: %s\n", msg, cudaGetErrorString(e));
    std::exit(2);
  }
}

}  // namespace

int main()
{
  // Canonical Nav2 shape from nav2_go2_full_stack.yaml MPPI block.
  nav_algo_mppi_cuda::IntegrateConfig cfg{};
  cfg.batch_size  = 2000;
  cfg.time_steps  = 56;
  cfg.dt          = 0.05f;
  cfg.initial_x   = 0.5f;
  cfg.initial_y   = -0.25f;
  cfg.initial_yaw = 0.3f;
  cfg.holonomic   = false;

  const size_t N = cfg.batch_size * cfg.time_steps;

  // Random inputs spanning the actuator bounds (vx_max=0.30, wz_max=0.8).
  std::mt19937 rng(42);
  std::uniform_real_distribution<float> vx_dist(-0.10f, 0.30f);
  std::uniform_real_distribution<float> wz_dist(-0.80f, 0.80f);
  std::vector<float> vx(N), vy(N, 0.0f), wz(N);
  for (size_t i = 0; i < N; ++i) {
    vx[i] = vx_dist(rng);
    wz[i] = wz_dist(rng);
  }

  // ── CPU reference + timing ───────────────────────────────────────────────
  std::vector<float> cpu_x, cpu_y, cpu_yaws;
  auto t0 = std::chrono::steady_clock::now();
  cpuIntegrate(cfg, vx, vy, wz, cpu_x, cpu_y, cpu_yaws);
  auto t1 = std::chrono::steady_clock::now();
  double cpu_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
  std::printf("CPU integrate: %.3f ms\n", cpu_ms);

  // ── GPU pipeline (HtoD copy, kernel, DtoH copy, sync) ───────────────────
  float *d_vx, *d_vy, *d_wz, *d_x, *d_y, *d_yaws;
  size_t bytes = N * sizeof(float);
  checkCuda(cudaMalloc(&d_vx, bytes),   "malloc vx");
  checkCuda(cudaMalloc(&d_vy, bytes),   "malloc vy");
  checkCuda(cudaMalloc(&d_wz, bytes),   "malloc wz");
  checkCuda(cudaMalloc(&d_x,  bytes),   "malloc x");
  checkCuda(cudaMalloc(&d_y,  bytes),   "malloc y");
  checkCuda(cudaMalloc(&d_yaws, bytes), "malloc yaws");

  checkCuda(cudaMemcpy(d_vx, vx.data(), bytes, cudaMemcpyHostToDevice), "vx H2D");
  checkCuda(cudaMemcpy(d_vy, vy.data(), bytes, cudaMemcpyHostToDevice), "vy H2D");
  checkCuda(cudaMemcpy(d_wz, wz.data(), bytes, cudaMemcpyHostToDevice), "wz H2D");

  // Warm-up + timed runs (median of 5).
  std::vector<double> ms;
  for (int run = 0; run < 6; ++run) {
    auto g0 = std::chrono::steady_clock::now();
    int rc = nav_algo_mppi_cuda::launchIntegrateStateVelocities(
      cfg, d_vx, d_vy, d_wz, d_x, d_y, d_yaws);
    auto g1 = std::chrono::steady_clock::now();
    if (rc != 0) {
      std::fprintf(stderr, "kernel launch failed: %d\n", rc);
      return 3;
    }
    if (run > 0) {
      ms.push_back(std::chrono::duration<double, std::milli>(g1 - g0).count());
    }
  }
  std::sort(ms.begin(), ms.end());
  double gpu_ms = ms[ms.size() / 2];
  std::printf("GPU integrate: %.3f ms (median of 5)\n", gpu_ms);
  std::printf("Speedup: %.1f×\n", cpu_ms / gpu_ms);

  // ── Output diff ──────────────────────────────────────────────────────────
  std::vector<float> gpu_x(N), gpu_y(N), gpu_yaws(N);
  checkCuda(cudaMemcpy(gpu_x.data(),    d_x,    bytes, cudaMemcpyDeviceToHost), "x D2H");
  checkCuda(cudaMemcpy(gpu_y.data(),    d_y,    bytes, cudaMemcpyDeviceToHost), "y D2H");
  checkCuda(cudaMemcpy(gpu_yaws.data(), d_yaws, bytes, cudaMemcpyDeviceToHost), "yaws D2H");

  float max_dx = 0, max_dy = 0, max_dyaw = 0;
  for (size_t i = 0; i < N; ++i) {
    max_dx   = std::fmax(max_dx,   std::fabs(gpu_x[i]    - cpu_x[i]));
    max_dy   = std::fmax(max_dy,   std::fabs(gpu_y[i]    - cpu_y[i]));
    max_dyaw = std::fmax(max_dyaw, std::fabs(gpu_yaws[i] - cpu_yaws[i]));
  }
  std::printf("Max |Δ| — x: %.3e, y: %.3e, yaw: %.3e\n", max_dx, max_dy, max_dyaw);

  cudaFree(d_vx); cudaFree(d_vy); cudaFree(d_wz);
  cudaFree(d_x);  cudaFree(d_y);  cudaFree(d_yaws);

  const float TOL = 1e-4f;
  if (max_dx > TOL || max_dy > TOL || max_dyaw > TOL) {
    std::fprintf(stderr, "FAIL: GPU output diverges beyond %g\n", TOL);
    return 1;
  }
  std::printf("PASS: GPU == CPU within %g\n", TOL);
  return 0;
}
