# Task for the desktop agent — Native ROS 2 CUDA-MPPI plugin

**Context:** the laptop session already shipped a complete ROS 1 Noetic
CUDA-MPPI stack (`src/vendor/nav_algo_ros1/`, 11 commits on `main` head).
This task adds a parallel **native ROS 2 Humble** integration path so the
same CUDA kernels can plug into `nav2` directly — useful for desktop
MuJoCo sim (no `ros1_bridge` hops, no rebuild for Jetson).

The laptop already pushed/committed all of:

- `src/vendor/nav_algo_ros1/nav_algo_core/` — ported Nav2 algorithm body
  (13K LOC verbatim from Nav2 Humble + 380-line `compat.hpp`)
- `src/vendor/nav_algo_ros1/nav_algo_mppi_cuda/` — **11 CUDA kernels**
  + `CudaBackend` class implementing `mppi::ICudaBackend`
- `src/vendor/nav_algo_ros1/nav_algo_mppi_ros1/` — `nav_core::BaseLocalPlanner`
  plugin that injects `CudaBackend` into `Optimizer::optimize()`
- `src/vendor/nav_algo_ros1/nav_algo_smac_ros1/` — `nav_core::BaseGlobalPlanner`
  plugin wrapping `AStarAlgorithm<NodeLattice>`
- `src/vendor/nav_algo_ros1/nav_algo_bringup/` — `move_base` launch + 5 yaml
  + `integration_test.sh` (verified GPU dispatches end-to-end on RTX 4050)

**The goal of this task:** ship a `nav2_core::Controller` plugin that
uses the *same* `nav_algo_mppi_cuda` library, so we can run CUDA-MPPI
under ROS 2 Humble's `nav2_controller_server` directly, without going
through ROS 1.

The kernels are already ROS-agnostic; the only change needed is
plumbing into a Nav2 plugin instead of a `move_base` one.

---

## Why this matters

- Laptop sim already runs Nav2 Humble. Currently MPPI is CPU-bound
  (~30 ms per `evalControl`). With CUDA backend wired in, the sim
  optimiser drops to sub-millisecond and CFPA2 + Nav2 can iterate at
  full controller frequency without CPU saturation.
- Same kernels work on the Jetson (Orin NX sm_87 already in
  `-gencode`); ROS 2 path is just for the desktop dev loop.
- Code-reuse stress test for the `ICudaBackend` injection pattern
  (Golden Rule #24 in CLAUDE.md). If it holds across ROS versions
  with no algorithm changes, the architecture is right.

---

## Three approaches — pick one

The challenge is that stock Nav2 `MPPIController::optimize()` is not
`virtual` and `MPPIController::optimizer_` is owned by value. So we
can't just override or monkey-patch upstream. Pick one of these:

### Approach A — dual-mode `nav_algo_core` (highest code reuse, biggest lift)

Build the *same* `src/vendor/nav_algo_ros1/nav_algo_core/` source tree
natively under ROS 2 by making `compat.hpp` + `parameters_handler.cpp`
conditional on a `NAV_ALGO_NATIVE_ROS2` build flag.

```cpp
// compat.hpp
#ifdef NAV_ALGO_NATIVE_ROS2
  #include <rclcpp/rclcpp.hpp>
  #include <rclcpp_lifecycle/lifecycle_node.hpp>
  #include <geometry_msgs/msg/pose_stamped.hpp>
  // ... all the real ROS 2 headers
  // No aliases — types resolve to their real ROS 2 definitions
#else
  // ... current ROS 1 compat content
#endif
```

`parameters_handler.{hpp,cpp}` needs a parallel ROS 2 implementation
that uses `rclcpp_lifecycle::LifecycleNode::declare_parameter` /
`get_parameter` instead of `ros::NodeHandle::param`. Restore the
original Nav2 implementation (lives in
`src/vendor/nav2_humble_src/nav2_mppi_controller/src/parameters_handler.cpp`)
under the same `#ifdef`.

Then create `nav_algo_mppi_ros2/` package:

- `CMakeLists.txt`: `ament_cmake`, `find_package(nav2_core REQUIRED)`,
  build `nav_algo_core` source files with `-DNAV_ALGO_NATIVE_ROS2`,
  link `nav_algo_mppi_cuda`.
- `src/cuda_mppi_controller.cpp`: `nav2_core::Controller` impl
  (mirrors `nav_algo_mppi_ros1/src/mppi_controller_ros.cpp` but uses
  `geometry_msgs::msg::TwistStamped`, `nav2_util::LifecycleNode`,
  etc. — directly from rclcpp).

**Effort:** ~3-4 hours.
**Pro:** one source tree for the algorithm; ROS 1 + ROS 2 share the
same files.
**Con:** parameters_handler dual-implementation is finicky; risk of
breaking the ROS 1 path while wiring the ROS 2 side.

### Approach B — fork upstream `nav2_mppi_controller` (cleaner)

Apply a ~100-LOC patch to a vendored copy of Nav2 Humble's
`nav2_mppi_controller`:

1. Add `ICudaBackend * cuda_backend_ = nullptr;` member to
   `nav2_mppi_controller::Optimizer`.
2. Add `setCudaBackend(ICudaBackend*)` public method.
3. Modify `Optimizer::optimize()` to dispatch:

   ```cpp
   void Optimizer::optimize() {
     if (cuda_backend_) {
       cuda_backend_->optimize(*this);
       return;
     }
     // existing xtensor body unchanged
   }
   ```

4. Add accessors on `Optimizer` matching what we did in
   `nav_algo_core/include/nav_algo_core/mppi/optimizer.hpp` (state(),
   control_sequence(), generated_trajectories(), path(), costs(),
   settings(), critic_manager(), critics_data(), motion_model(),
   isHolonomicPublic(), applyControlSequenceConstraintsPublic(),
   generateNoisedTrajectoriesNoIntegrate()).
5. New `nav2_mppi_controller_cuda` package: `nav2_core::Controller`
   subclass of `nav2_mppi_controller::MPPIController` that constructs
   a `CudaBackend` from yaml `use_cuda` and calls `setCudaBackend(...)`
   during `configure()`.

**Where the upstream copy lives:** the laptop already vendored Nav2
Humble at `src/vendor/nav2_humble_src/` (sparse-checkout of
`navigation2`). Copy `nav2_mppi_controller/` out of there into a new
working dir `src/vendor/nav2_mppi_controller_cuda/`, apply the patch.

`ICudaBackend` declaration lives in
`src/vendor/nav_algo_ros1/nav_algo_core/include/nav_algo_core/mppi/cuda_backend.hpp`
— pure-virtual interface, no CUDA deps. The patched
`nav2_mppi_controller_cuda` can `#include` it directly without pulling
the ROS 1 build path.

**Effort:** 1.5-2 hours.
**Pro:** Native ROS 2 build, no compat shim complexity. Algorithm
matches upstream Nav2 bit-for-bit (no port drift).
**Con:** Maintenance burden on tracking upstream Nav2 changes (small
patch though).

### Approach C — symlink source + new CMake (medium)

Same idea as A but instead of conditional compilation, *symlink* the
source files into a new `nav_algo_mppi_ros2/` package directory. The
ROS 2 package's CMakeLists builds those files with `rclcpp` + native
ROS 2 includes (no `compat.hpp`), and provides its own
`parameters_handler` implementation that uses rclcpp params.

**Effort:** 2 hours.
**Pro:** Avoids dual-mode complexity in nav_algo_core; ROS 1 stays
clean.
**Con:** Source files are physically duplicated (via symlinks), some
build-system magic to make rclcpp's logging macros expand correctly.

---

## Recommendation: **Approach B**

Cleanest separation, smallest patch surface, no dual-mode complexity
on `nav_algo_core`. Lets us also contribute the patch upstream to
Nav2 if it goes well — they've been receptive to acceleration
contributions in the past.

---

## Concrete steps (Approach B)

1. **Vendor upstream copy.** Either:
   - `cp -r src/vendor/nav2_humble_src/nav2_mppi_controller src/vendor/nav2_mppi_controller_cuda` (uses the laptop's existing sparse checkout)
   - or `git clone --depth 1 --branch humble https://github.com/ros-navigation/navigation2.git /tmp/nav2 && cp -r /tmp/nav2/nav2_mppi_controller src/vendor/nav2_mppi_controller_cuda`

   Rename it: `mv .../nav2_mppi_controller_cuda/<files>` (the package
   name in `package.xml`) — keep the namespace `nav2_mppi_controller`
   in source (so all critic plugins still load), just rename the
   package + library.

2. **Apply the Optimizer patch.** In
   `src/vendor/nav2_mppi_controller_cuda/include/nav2_mppi_controller/optimizer.hpp`:

   ```cpp
   #include "nav_algo_core/mppi/cuda_backend.hpp"  // pure-virtual interface

   class Optimizer {
   public:
     // ... existing members ...
     void setCudaBackend(mppi::ICudaBackend * b) { cuda_backend_ = b; }

     // Public accessors needed by the CUDA backend. Match the
     // signature set we added to nav_algo_core (10-15 trivial getters).
     models::State            & state()                    { return state_; }
     models::ControlSequence  & control_sequence()         { return control_sequence_; }
     models::Trajectories     & generated_trajectories()   { return generated_trajectories_; }
     models::Path             & path()                     { return path_; }
     xt::xtensor<float, 1>    & costs()                    { return costs_; }
     models::OptimizerSettings& settings()                 { return settings_; }
     CriticManager            & critic_manager()           { return critic_manager_; }
     CriticData               & critics_data()             { return critics_data_; }
     std::shared_ptr<MotionModel> & motion_model()         { return motion_model_; }
     bool isHolonomicPublic() const { return motion_model_ ? motion_model_->isHolonomic() : false; }
     void applyControlSequenceConstraintsPublic() { applyControlSequenceConstraints(); }
     void generateNoisedTrajectoriesNoIntegrate()
     {
       noise_generator_.setNoisedControls(state_, control_sequence_);
       noise_generator_.generateNextNoises();
       updateStateVelocities(state_);
     }
     nav2_costmap_2d::Costmap2D * getCostmapForBackend()    { return costmap_; }
     ParametersHandler * getParametersHandler() const       { return parameters_handler_; }

   protected:
     // ... existing ...
     mppi::ICudaBackend * cuda_backend_ = nullptr;
   };
   ```

   In `src/optimizer.cpp::optimize()`:

   ```cpp
   void Optimizer::optimize()
   {
     if (cuda_backend_) {
       cuda_backend_->optimize(*this);
       return;
     }
     for (size_t i = 0; i < settings_.iteration_count; ++i) {
       generateNoisedTrajectories();
       critic_manager_.evalTrajectoriesScores(critics_data_);
       updateControlSequence();
     }
   }
   ```

3. **New ROS 2 plugin package: `nav2_mppi_controller_cuda_plugin`.**
   Header (`cuda_mppi_controller.hpp`):

   ```cpp
   #include "nav2_mppi_controller/controller.hpp"          // patched upstream
   #include "nav_algo_mppi_cuda/cuda_backend.hpp"

   class CudaMPPIController : public nav2_mppi_controller::MPPIController {
   public:
     void configure(parent, name, tf, costmap_ros) override {
       MPPIController::configure(parent, name, tf, costmap_ros);
       // Check yaml use_cuda; if true, create CudaBackend, attach.
       auto pnh = parent.lock();
       bool use_cuda = false;
       pnh->declare_parameter(name + ".use_cuda", false);
       use_cuda = pnh->get_parameter(name + ".use_cuda").as_bool();
       if (use_cuda) {
         nav_algo_mppi_cuda::CudaBackendConfig bcfg{};
         bcfg.batch_size       = optimizer_.settings().batch_size;
         bcfg.time_steps       = optimizer_.settings().time_steps;
         bcfg.path_max_points  = 1024;
         bcfg.costmap_max_cells = 4 * 1024 * 1024;
         bcfg.footprint_max_n  = 16;
         cuda_backend_ = std::make_unique<nav_algo_mppi_cuda::CudaBackend>(bcfg);

         std::vector<float> fp_x, fp_y;
         for (const auto & p : costmap_ros->getRobotFootprint()) {
           fp_x.push_back(p.x); fp_y.push_back(p.y);
         }
         cuda_backend_->setFootprint(fp_x, fp_y);
         cuda_backend_->loadCriticParams(optimizer_.getParametersHandler(), name_);
         optimizer_.setCudaBackend(cuda_backend_.get());
         RCLCPP_INFO(logger_, "CUDA backend ENABLED.");
       }
     }
   private:
     std::unique_ptr<nav_algo_mppi_cuda::CudaBackend> cuda_backend_;
   };
   PLUGINLIB_EXPORT_CLASS(CudaMPPIController, nav2_core::Controller)
   ```

   `package.xml` deps: `rclcpp`, `nav2_core`, `nav2_mppi_controller`
   (the patched one), `nav_algo_mppi_cuda`.

4. **Build verification.** From repo root:

   ```bash
   source /opt/ros/humble/setup.bash
   colcon build --packages-select nav2_mppi_controller_cuda nav2_mppi_controller_cuda_plugin nav_algo_mppi_cuda
   ```

   `nav_algo_mppi_cuda` package needs no changes — it's ROS-agnostic
   and already builds against the catkin envelope. May need a small
   `CMakeLists` tweak to also accept `ament_cmake` as a build option;
   easier alternative is to manually include the .cu/.cpp files in
   the ROS 2 plugin package's CMakeLists.

   Actually simplest path: copy the kernels into the ROS 2 plugin
   package's `src/cuda/` directory, build them there with
   `find_package(CUDA REQUIRED)` and `cuda_add_library`. Avoid the
   cross-build-system mess of trying to consume the catkin
   `nav_algo_mppi_cuda` from a colcon workspace.

5. **Test in sim.** Modify a Nav2 yaml (e.g.
   `src/go2w/go2w_config/config/nav/nav2_go2_full_stack.yaml`):

   ```yaml
   FollowPath:
     plugin: "nav2_mppi_controller_cuda_plugin/CudaMPPIController"
     use_cuda: true
     # ... rest of yaml stays identical to existing MPPI block
   ```

   Run `./scripts/launch/nav_test_3d_explore.sh` — should drive with
   CUDA-accelerated MPPI. Verify via the probe file pattern (compile
   with `-DNAV_ALGO_CUDA_PROBE` to enable the `/tmp/cuda_backend_*`
   sentinels).

---

## Where to find what

- **`ICudaBackend` interface (pure-virtual, no CUDA dep):**
  [`src/vendor/nav_algo_ros1/nav_algo_core/include/nav_algo_core/mppi/cuda_backend.hpp`](src/vendor/nav_algo_ros1/nav_algo_core/include/nav_algo_core/mppi/cuda_backend.hpp)
- **`CudaBackend` impl (the GPU pipeline that does all the work):**
  [`src/vendor/nav_algo_ros1/nav_algo_mppi_cuda/src/cuda_backend.cu`](src/vendor/nav_algo_ros1/nav_algo_mppi_cuda/src/cuda_backend.cu)
- **`CriticParams` (yaml-loaded values for all 8 critics):**
  [`src/vendor/nav_algo_ros1/nav_algo_mppi_cuda/include/nav_algo_mppi_cuda/cuda_backend.hpp`](src/vendor/nav_algo_ros1/nav_algo_mppi_cuda/include/nav_algo_mppi_cuda/cuda_backend.hpp)
- **11 kernels + device-memory RAII:**
  [`src/vendor/nav_algo_ros1/nav_algo_mppi_cuda/src/`](src/vendor/nav_algo_ros1/nav_algo_mppi_cuda/src/)
  (`integrate.cu`, `critics.cu`, `control_update.cu`, `cuda_backend.cu`,
  `device_memory.hpp`).
- **ROS 1 plugin (template for the ROS 2 version's structure):**
  [`src/vendor/nav_algo_ros1/nav_algo_mppi_ros1/src/mppi_controller_ros.cpp`](src/vendor/nav_algo_ros1/nav_algo_mppi_ros1/src/mppi_controller_ros.cpp)
  — see the `use_cuda` branch in `initialize()`.
- **Nav2 Humble upstream (vendored sparse checkout):**
  `src/vendor/nav2_humble_src/` — has `nav2_mppi_controller`,
  `nav2_smac_planner`, `nav2_core`, `nav2_costmap_2d`, `nav2_util`.

---

## Constraints + caveats

- **Don't break the ROS 1 build.** Run
  `bash src/vendor/nav_algo_ros1/nav_algo_bringup/test/integration_test.sh`
  in the existing `nav_algo:build_env_cuda` Docker image after any
  change to `nav_algo_core` or `nav_algo_mppi_cuda`. cmd_vel should
  stay at ~0.238 m/s (matches the v2 gates' post-commit value).
- **xtensor 0.24.7** required. Both ROS 1 Noetic Docker image and
  /opt/ros/humble have compatible versions on Ubuntu 22.04.
- **CUDA toolkit:** 12.6 on the laptop; Jetson Orin NX with JetPack
  5.x has CUDA 11.4 which is still compatible (all the APIs we use —
  `cudaMalloc`, `cudaMemcpy`, `cudaStream`, `cub::BlockReduce/Scan` —
  are stable since CUDA 11). `-gencode arch=compute_87,sm_87` is in
  the CMakeLists already, so Orin builds work natively.
- **Math equivalence:** the v2 backend (commit `096e380`) matches
  Nav2 sim semantics for the gates. Verify by running the same
  scenario in both ROS 1 + ROS 2 with `use_cuda: false` first
  (xtensor CPU path), then `use_cuda: true`. cmd_vel should be
  identical within fp32 epsilon between the two GPU paths, and very
  close to the xtensor reference.

---

## Current commit state at start of this task

- `main` head: `096e380` (CudaBackend v2: yaml-driven critic params +
  host-side gates). `b0b1ea0` is the lifecycle hardening commit right
  before it.
- 2 commits ahead of `origin/main`. Push first before starting this
  task so the desktop agent has a clean base.

`git log --oneline -5`:

```
096e380 CudaBackend v2: yaml-driven critic params + host-side gates
b0b1ea0 CUDA lifecycle hardening: RAII device buffers, sticky-error reset
81f971d CLAUDE.md: full GPU MPPI pipeline (11 kernels + Optimizer integration)
a00e35c nav_algo_mppi_*: wire CudaBackend into Optimizer + plugin
94595ea nav_algo_mppi_cuda: control-update kernels (cost-shape + softmax + weighted-avg)
```

---

## Out of scope (don't do these unless explicitly asked)

- Don't touch the Smac global planner port. SmacPlannerLattice on the
  global side is fine on CPU (~80 ms for our usual scenes); GPU lift
  is for MPPI only at this stage.
- Don't touch CFPA2. That's a separate parallel-track effort
  (commit `d0526bf` — pure C++ port, already done).
- Don't break the existing ROS 1 integration test. Run it as a
  regression check.

---

## Done when

1. `colcon build --packages-select nav2_mppi_controller_cuda_plugin` succeeds.
2. RViz Nav2 goal in MuJoCo sim drives the robot with
   `use_cuda: true`.
3. `/tmp/cuda_backend_optimize` probe file (if built with
   `-DNAV_ALGO_CUDA_PROBE`) shows the GPU path running.
4. cmd_vel is qualitatively similar to the CPU baseline on the same
   scene.
5. Commit + push.
