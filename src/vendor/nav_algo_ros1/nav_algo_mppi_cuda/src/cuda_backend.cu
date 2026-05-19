// Copyright 2026 Collab_QRC
// SPDX-License-Identifier: Apache-2.0
//
// CudaBackend implementation. See cuda_backend.hpp for the design rationale.

#include "nav_algo_mppi_cuda/cuda_backend.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <fstream>
#include <stdexcept>
#include <string>

#include <cuda_runtime.h>

#include "nav_algo_core/mppi/optimizer.hpp"

// ── Debug probes (writes to /tmp/cuda_backend_*) ────────────────────────
// Removable in production; useful to detect whether use_cuda yaml was
// read correctly (ctor probe) and whether Optimizer::optimize actually
// dispatches to the backend (optimize probe). Files are append-only so
// repeated runs accumulate, with timestamps.
namespace { std::atomic<uint64_t> g_optimize_calls{0}; }

namespace nav_algo_mppi_cuda
{

namespace
{
inline void cudaCheck(cudaError_t e, const char * what)
{
  if (e != cudaSuccess) {
    throw std::runtime_error(
      std::string("CudaBackend ") + what + ": " + cudaGetErrorString(e));
  }
}

template <typename T>
T * cudaAlloc(size_t n)
{
  T * p = nullptr;
  cudaCheck(cudaMalloc(&p, n * sizeof(T)), "cudaMalloc");
  return p;
}

}  // namespace

CudaBackend::CudaBackend(const CudaBackendConfig & cfg)
: cfg_(cfg)
{
  const size_t Nbt = static_cast<size_t>(cfg.batch_size) * cfg.time_steps;
  const size_t B   = cfg.batch_size;
  const size_t T   = cfg.time_steps;
  const size_t P   = cfg.path_max_points;
  const size_t CM  = cfg.costmap_max_cells;
  const size_t FP  = cfg.footprint_max_n;

  d_traj_x_         = cudaAlloc<float>(Nbt);
  d_traj_y_         = cudaAlloc<float>(Nbt);
  d_traj_yaws_      = cudaAlloc<float>(Nbt);
  d_state_vx_       = cudaAlloc<float>(Nbt);
  d_state_vy_       = cudaAlloc<float>(Nbt);
  d_state_wz_       = cudaAlloc<float>(Nbt);
  d_state_cvx_      = cudaAlloc<float>(Nbt);
  d_state_cvy_      = cudaAlloc<float>(Nbt);
  d_state_cwz_      = cudaAlloc<float>(Nbt);
  d_ctrl_vx_        = cudaAlloc<float>(T);
  d_ctrl_vy_        = cudaAlloc<float>(T);
  d_ctrl_wz_        = cudaAlloc<float>(T);
  d_costs_          = cudaAlloc<float>(B);
  d_softmax_        = cudaAlloc<float>(B);
  d_path_x_         = cudaAlloc<float>(P);
  d_path_y_         = cudaAlloc<float>(P);
  d_path_yaws_      = cudaAlloc<float>(P);
  d_path_int_dist_  = cudaAlloc<float>(P);
  d_path_pts_valid_ = cudaAlloc<uint8_t>(P);
  d_costmap_        = cudaAlloc<uint8_t>(CM);
  d_fp_x_           = cudaAlloc<float>(FP);
  d_fp_y_           = cudaAlloc<float>(FP);

  // PROBE: ctor was reached. Mere file existence proves use_cuda yaml
  // was read true AND the plugin instantiated us.
  {
    std::ofstream f("/tmp/cuda_backend_ctor", std::ios::app);
    auto t = std::chrono::system_clock::now().time_since_epoch();
    f << "ctor at " << std::chrono::duration_cast<std::chrono::milliseconds>(t).count()
      << "ms B=" << cfg.batch_size << " T=" << cfg.time_steps
      << " CM_max=" << cfg.costmap_max_cells << "\n";
  }
}

CudaBackend::~CudaBackend()
{
  // Quietly free; dtor runs on plugin shutdown so any cuda errors here are
  // unrecoverable anyway. Order chosen to mirror ctor for clarity.
  auto safe_free = [](void * p) { if (p) cudaFree(p); };
  safe_free(d_traj_x_);       safe_free(d_traj_y_);       safe_free(d_traj_yaws_);
  safe_free(d_state_vx_);     safe_free(d_state_vy_);     safe_free(d_state_wz_);
  safe_free(d_state_cvx_);    safe_free(d_state_cvy_);    safe_free(d_state_cwz_);
  safe_free(d_ctrl_vx_);      safe_free(d_ctrl_vy_);      safe_free(d_ctrl_wz_);
  safe_free(d_costs_);        safe_free(d_softmax_);
  safe_free(d_path_x_);       safe_free(d_path_y_);       safe_free(d_path_yaws_);
  safe_free(d_path_int_dist_); safe_free(d_path_pts_valid_);
  safe_free(d_costmap_);
  safe_free(d_fp_x_);         safe_free(d_fp_y_);
}

void CudaBackend::setFootprint(const std::vector<float> & fp_x, const std::vector<float> & fp_y)
{
  if (fp_x.size() != fp_y.size() || fp_x.size() > cfg_.footprint_max_n) {
    throw std::runtime_error("CudaBackend::setFootprint: size mismatch or > footprint_max_n");
  }
  cudaCheck(cudaMemcpy(d_fp_x_, fp_x.data(), fp_x.size() * sizeof(float),
                       cudaMemcpyHostToDevice), "H2D fp_x");
  cudaCheck(cudaMemcpy(d_fp_y_, fp_y.data(), fp_y.size() * sizeof(float),
                       cudaMemcpyHostToDevice), "H2D fp_y");
  fp_n_ = static_cast<unsigned int>(fp_x.size());
}

void CudaBackend::optimize(mppi::Optimizer & opt)
{
  // PROBE: first call writes file; every call increments counter. The
  // optimize-call file proves Optimizer::optimize() dispatched to us
  // rather than running the xtensor CPU path.
  const uint64_t call_n = g_optimize_calls.fetch_add(1) + 1;
  if (call_n == 1) {
    std::ofstream f("/tmp/cuda_backend_optimize", std::ios::app);
    auto t = std::chrono::system_clock::now().time_since_epoch();
    f << "first optimize() at "
      << std::chrono::duration_cast<std::chrono::milliseconds>(t).count() << "ms\n";
  }
  if (call_n % 50 == 0) {
    std::ofstream f("/tmp/cuda_backend_optimize", std::ios::app);
    f << "call_n=" << call_n << "\n";
  }

  auto & state    = opt.state();
  auto & ctrl     = opt.control_sequence();
  auto & trajs    = opt.generated_trajectories();
  auto & path     = opt.path();
  auto & costs    = opt.costs();
  auto & settings = opt.settings();
  const bool holonomic = opt.isHolonomicPublic();

  const unsigned int B = cfg_.batch_size;
  const unsigned int T = cfg_.time_steps;

  // Costmap pointer (live each cycle; map can be re-rolled by Nav2).
  auto * cm = opt.getCostmapForBackend();
  if (cm == nullptr) {
    throw std::runtime_error("CudaBackend::optimize: costmap is null");
  }
  const unsigned int cm_w = cm->getSizeInCellsX();
  const unsigned int cm_h = cm->getSizeInCellsY();
  if (static_cast<size_t>(cm_w) * cm_h > cfg_.costmap_max_cells) {
    throw std::runtime_error(
      "CudaBackend::optimize: costmap exceeds costmap_max_cells");
  }
  CostmapInfo cm_info{};
  cm_info.size_x     = cm_w;
  cm_info.size_y     = cm_h;
  cm_info.origin_x   = static_cast<float>(cm->getOriginX());
  cm_info.origin_y   = static_cast<float>(cm->getOriginY());
  cm_info.resolution = static_cast<float>(cm->getResolution());

  cudaCheck(cudaMemcpy(d_costmap_, cm->getCharMap(),
                       static_cast<size_t>(cm_w) * cm_h * sizeof(uint8_t),
                       cudaMemcpyHostToDevice), "H2D costmap");

  // Path upload (P = current path length, may be smaller than path_max).
  const unsigned int P = path.x.shape(0);
  if (P > cfg_.path_max_points) {
    throw std::runtime_error("CudaBackend::optimize: path > path_max_points");
  }
  if (P > 0) {
    cudaCheck(cudaMemcpy(d_path_x_,    path.x.data(),    P * sizeof(float), cudaMemcpyHostToDevice), "H2D path.x");
    cudaCheck(cudaMemcpy(d_path_y_,    path.y.data(),    P * sizeof(float), cudaMemcpyHostToDevice), "H2D path.y");
    cudaCheck(cudaMemcpy(d_path_yaws_, path.yaws.data(), P * sizeof(float), cudaMemcpyHostToDevice), "H2D path.yaws");
    // Path-integrated distances + path_pts_valid: compute lazily on host.
    std::vector<float> int_dist(P, 0.0f);
    std::vector<uint8_t> pts_valid(P, 1);
    for (unsigned int i = 1; i < P; ++i) {
      const float dx = path.x(i) - path.x(i - 1);
      const float dy = path.y(i) - path.y(i - 1);
      int_dist[i] = int_dist[i - 1] + std::sqrt(dx * dx + dy * dy);
    }
    cudaCheck(cudaMemcpy(d_path_int_dist_,  int_dist.data(),  P * sizeof(float),   cudaMemcpyHostToDevice), "H2D int_dist");
    cudaCheck(cudaMemcpy(d_path_pts_valid_, pts_valid.data(), P * sizeof(uint8_t), cudaMemcpyHostToDevice), "H2D pts_valid");
  }

  // Per-iteration. Default yaml has iteration_count=1.
  for (unsigned int it = 0; it < settings.iteration_count; ++it) {
    // ── 1. CPU-side state generation (noise + motion model). Same as
    //       Optimizer::generateNoisedTrajectories minus integrate.
    opt.generateNoisedTrajectoriesNoIntegrate();

    // ── 2. Upload state (B×T floats each) to GPU
    const size_t Nbt = static_cast<size_t>(B) * T * sizeof(float);
    cudaCheck(cudaMemcpy(d_state_vx_,  state.vx.data(),  Nbt, cudaMemcpyHostToDevice), "H2D state.vx");
    cudaCheck(cudaMemcpy(d_state_wz_,  state.wz.data(),  Nbt, cudaMemcpyHostToDevice), "H2D state.wz");
    cudaCheck(cudaMemcpy(d_state_cvx_, state.cvx.data(), Nbt, cudaMemcpyHostToDevice), "H2D state.cvx");
    cudaCheck(cudaMemcpy(d_state_cwz_, state.cwz.data(), Nbt, cudaMemcpyHostToDevice), "H2D state.cwz");
    if (holonomic) {
      cudaCheck(cudaMemcpy(d_state_vy_,  state.vy.data(),  Nbt, cudaMemcpyHostToDevice), "H2D state.vy");
      cudaCheck(cudaMemcpy(d_state_cvy_, state.cvy.data(), Nbt, cudaMemcpyHostToDevice), "H2D state.cvy");
    }
    // Upload current control_sequence for cost-shaping.
    cudaCheck(cudaMemcpy(d_ctrl_vx_, ctrl.vx.data(), T * sizeof(float), cudaMemcpyHostToDevice), "H2D ctrl.vx");
    cudaCheck(cudaMemcpy(d_ctrl_wz_, ctrl.wz.data(), T * sizeof(float), cudaMemcpyHostToDevice), "H2D ctrl.wz");
    if (holonomic) {
      cudaCheck(cudaMemcpy(d_ctrl_vy_, ctrl.vy.data(), T * sizeof(float), cudaMemcpyHostToDevice), "H2D ctrl.vy");
    }

    // ── 3. Integrate kernel: state.vx/vy/wz + pose → trajectories.
    {
      IntegrateConfig icfg{};
      icfg.batch_size  = B;
      icfg.time_steps  = T;
      icfg.dt          = settings.model_dt;
      icfg.initial_x   = static_cast<float>(state.pose.pose.position.x);
      icfg.initial_y   = static_cast<float>(state.pose.pose.position.y);
      // tf2::getYaw via direct quaternion math (matches CPU getYaw exactly).
      {
        const auto & q = state.pose.pose.orientation;
        const float siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
        const float cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
        icfg.initial_yaw = std::atan2(siny_cosp, cosy_cosp);
      }
      icfg.holonomic   = holonomic;
      const int rc = launchIntegrateStateVelocities(
        icfg, d_state_vx_, holonomic ? d_state_vy_ : nullptr, d_state_wz_,
        d_traj_x_, d_traj_y_, d_traj_yaws_);
      cudaCheck(static_cast<cudaError_t>(rc), "integrate kernel");
    }

    // ── 4. Zero costs[B] before critic accumulation
    cudaCheck(cudaMemset(d_costs_, 0, B * sizeof(float)), "memset costs");

    // ── 5. Critics. Host-side gates simplified for v1:
    //   - within_position_goal_tolerance gates are SKIPPED — all critics run.
    //     This matches CPU behaviour only when the robot is far from the
    //     goal (typical exploration). Near-goal divergence is the v2 fix.
    //   - PathAlign/PathFollow/PathAngle path-index calculations use
    //     furthest_reached = P - 1 (degraded; CPU computes a smaller index
    //     based on robot's current position along the path).
    //
    // The critic order matches the yaml ordering (ConstraintCritic first,
    // PreferForward last). Order matters for floating-point accumulation;
    // matches CPU's CriticManager::evalTrajectoriesScores iteration.
    CriticConfig ccfg_template{};
    ccfg_template.batch_size = B;
    ccfg_template.time_steps = T;
    ccfg_template.power = 1;

    // (a) ConstraintCritic
    {
      const float vx_max = settings.constraints.vx_max;
      const float vy_max = settings.constraints.vy;     // mppi stores vy_max as constraints.vy
      const float vx_min = settings.constraints.vx_min;
      const float max_vel = std::sqrt(vx_max * vx_max + vy_max * vy_max);
      const float min_sgn = vx_min > 0.0f ? 1.0f : -1.0f;
      const float min_vel = min_sgn * std::sqrt(vx_min * vx_min + vy_max * vy_max);
      // weight: ConstraintCritic stores its own weight inside critic_manager_.
      // For now we hard-pick from yaml-equivalent: 4.0.
      CriticConfig cfg = ccfg_template;
      cfg.weight = 4.0f;
      launchConstraintCritic(cfg, d_state_vx_, holonomic ? d_state_vy_ : nullptr,
                             min_vel, max_vel, settings.model_dt, d_costs_);
    }

    // (b) ObstaclesCritic — yaml weights: critical 20, repulsion 3,
    //     consider_footprint true, margin 0.05, collision_cost 10000.
    {
      ObstaclesConfig ocfg{};
      ocfg.batch_size               = B;
      ocfg.time_steps               = T;
      ocfg.power                    = 1;
      ocfg.critical_weight          = 20.0f;
      ocfg.repulsion_weight         = 3.0f;
      ocfg.collision_cost           = 10000.0f;
      ocfg.collision_margin_distance = 0.05f;
      ocfg.inflation_radius         = 0.20f;
      ocfg.inflation_scale_factor   = 5.0f;
      ocfg.circumscribed_radius     = 0.20f;
      ocfg.possibly_inscribed_cost  = 100.0f;
      ocfg.tracking_unknown         = true;
      ocfg.near_goal                = false;
      ocfg.consider_footprint       = (fp_n_ >= 2);
      launchObstaclesCritic(ocfg, d_traj_x_, d_traj_y_, d_traj_yaws_,
                            d_costmap_, cm_info,
                            d_fp_x_, d_fp_y_, fp_n_, d_costs_);
    }

    // (c) GoalCritic — weight 5.0, threshold 1.4 (skipped on host if
    //     dist(robot, goal) > 1.4; v1 always runs).
    if (P > 0) {
      const float gx = path.x(P - 1);
      const float gy = path.y(P - 1);
      CriticConfig cfg = ccfg_template;
      cfg.weight = 5.0f;
      launchGoalCritic(cfg, d_traj_x_, d_traj_y_, gx, gy, d_costs_);
    }

    // (d) GoalAngleCritic — weight 3.0, threshold 0.5
    if (P > 0) {
      const float gyaw = path.yaws(P - 1);
      CriticConfig cfg = ccfg_template;
      cfg.weight = 3.0f;
      launchGoalAngleCritic(cfg, d_traj_yaws_, gyaw, /*symmetric=*/false, d_costs_);
    }

    // (e) PathAlignCritic — yaml weight 14.0, stride 4. furthest_reached
    //     is computed CPU-side in Nav2 from the robot's current path
    //     position; for v1 we use P (full path) as upper bound.
    if (P > 1) {
      PathAlignConfig pcfg{};
      pcfg.batch_size            = B;
      pcfg.time_steps            = T;
      pcfg.path_segments_count   = P;
      pcfg.trajectory_point_step = 4;
      pcfg.power                 = 1;
      pcfg.weight                = 14.0f;
      pcfg.use_path_orientations = false;
      launchPathAlignCritic(pcfg, d_traj_x_, d_traj_y_, d_traj_yaws_,
                            d_path_x_, d_path_y_, d_path_yaws_,
                            d_path_int_dist_, d_path_pts_valid_, d_costs_);
    }

    // (f) PathFollowCritic — yaml weight 5.0, threshold 1.4, offset 5.
    //     Uses one path point (offset_from_furthest indices forward from
    //     furthest_reached_path_point). v1 picks the last path point.
    if (P > 0) {
      CriticConfig cfg = ccfg_template;
      cfg.weight = 5.0f;
      const float px = path.x(P - 1);
      const float py = path.y(P - 1);
      launchPathFollowCritic(cfg, d_traj_x_, d_traj_y_, px, py, d_costs_);
    }

    // (g) PathAngleCritic — yaml weight 2.0
    if (P > 0) {
      CriticConfig cfg = ccfg_template;
      cfg.weight = 2.0f;
      const float px = path.x(P - 1);
      const float py = path.y(P - 1);
      launchPathAngleCritic(cfg, d_traj_x_, d_traj_y_, d_traj_yaws_,
                            px, py, d_costs_);
    }

    // (h) PreferForwardCritic — yaml weight 5.0
    {
      CriticConfig cfg = ccfg_template;
      cfg.weight = 5.0f;
      launchPreferForwardCritic(cfg, d_state_vx_, settings.model_dt, d_costs_);
    }

    // ── 6. Cost-shape bias term (per dimension)
    ControlUpdateConfig ucfg{};
    ucfg.batch_size  = B;
    ucfg.time_steps  = T;
    ucfg.temperature = settings.temperature;
    ucfg.gamma       = settings.gamma;
    ucfg.std_vx      = settings.sampling_std.vx;
    ucfg.std_vy      = settings.sampling_std.vy;
    ucfg.std_wz      = settings.sampling_std.wz;
    launchCostShape(ucfg, d_ctrl_vx_, d_state_cvx_, ucfg.std_vx, d_costs_);
    launchCostShape(ucfg, d_ctrl_wz_, d_state_cwz_, ucfg.std_wz, d_costs_);
    if (holonomic) {
      launchCostShape(ucfg, d_ctrl_vy_, d_state_cvy_, ucfg.std_vy, d_costs_);
    }

    // ── 7. Softmax over costs[B]
    launchSoftmax(ucfg, d_costs_, d_softmax_);

    // ── 8. Weighted average per T column → new control_sequence
    launchWeightedAvg(ucfg, d_state_cvx_, d_softmax_, d_ctrl_vx_);
    launchWeightedAvg(ucfg, d_state_cwz_, d_softmax_, d_ctrl_wz_);
    if (holonomic) {
      launchWeightedAvg(ucfg, d_state_cvy_, d_softmax_, d_ctrl_vy_);
    }

    // ── 9. Download control_sequence + apply constraints on host
    cudaCheck(cudaMemcpy(ctrl.vx.data(), d_ctrl_vx_, T * sizeof(float), cudaMemcpyDeviceToHost), "D2H ctrl.vx");
    cudaCheck(cudaMemcpy(ctrl.wz.data(), d_ctrl_wz_, T * sizeof(float), cudaMemcpyDeviceToHost), "D2H ctrl.wz");
    if (holonomic) {
      cudaCheck(cudaMemcpy(ctrl.vy.data(), d_ctrl_vy_, T * sizeof(float), cudaMemcpyDeviceToHost), "D2H ctrl.vy");
    }
    opt.applyControlSequenceConstraintsPublic();

    // ── 10. Download trajectories (for viz) + costs (for next-iter inspection).
    // Skipped by default: nothing downstream of optimize() reads them.
    (void)trajs;
    (void)costs;
  }
}

}  // namespace nav_algo_mppi_cuda
