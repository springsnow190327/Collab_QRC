#!/usr/bin/env bash
# Nav test with CUDA-accelerated MPPI controller (CudaMPPIController).
#
# Drop-in for nav_test_fastlio.sh — same MuJoCo sim + Fast-LIO SLAM + Nav2
# stack, but the Nav2 controller_server loads the GPU MPPI plugin instead of
# the upstream CPU MPPIController.  Plugin wired via nav2_go2{w}_full_stack.yaml
# (FollowPath.plugin: nav2_mppi_controller_cuda_plugin::CudaMPPIController).
#
# Usage:
#   ./scripts/launch/nav_test_cuda_mppi.sh                        # Go2W (has_wheels=true)
#   ./scripts/launch/nav_test_cuda_mppi.sh has_wheels:=false      # Go2 (walking only)
#   ./scripts/launch/nav_test_cuda_mppi.sh gui:=false rviz:=false # headless
#   ./scripts/launch/nav_test_cuda_mppi.sh nav_costmap_mode:=3d   # ETH trav grid
#
# CUDA probe files written to /tmp/ when the plugin initialises:
#   /tmp/cuda_backend_ctor       — written at CudaBackend construction
#   /tmp/cuda_backend_optimize   — written on first optimize() call
# (requires: colcon build ... -DNAV_ALGO_CUDA_PROBE=ON)
#
# Fallback: set use_cuda:=false in the yaml or pass
#   nav2_yaml_override:=nav2_go2w_full_stack_no_cuda.yaml
# to revert to the upstream CPU xtensor MPPI path with no code changes.
set -euo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/_preflight_kill.sh"

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
ROS2_SETUP_BASH="${ROS2_SETUP_BASH:-/opt/ros/humble/setup.bash}"

safe_source() { set +u; source "$1"; set -u; }

if [[ -f "${HOME}/miniforge3/etc/profile.d/conda.sh" ]]; then
  safe_source "${HOME}/miniforge3/etc/profile.d/conda.sh"
  conda activate cmu_env
elif command -v micromamba >/dev/null 2>&1; then
  eval "$(micromamba shell hook -s bash)"
  micromamba activate cmu_env
fi

safe_source "${ROS2_SETUP_BASH}"
safe_source "${WS_DIR}/install/setup.bash"

# mujoco_ros2_control needs libmujoco.so.3.6.0 on LD_LIBRARY_PATH.
# See memory/feedback_libmujoco_ld_path.md.
if [[ -n "${CONDA_PREFIX:-}" ]] && \
   [[ -d "${CONDA_PREFIX}/lib/python3.10/site-packages/mujoco" ]]; then
  export LD_LIBRARY_PATH="${CONDA_PREFIX}/lib/python3.10/site-packages/mujoco:${LD_LIBRARY_PATH:-}"
fi

export FASTRTPS_DEFAULT_PROFILES_FILE="${WS_DIR}/config/fastdds_no_shm.xml"

# ── CUDA plugin pre-flight ────────────────────────────────────────────────────
# Fail fast if the plugin wasn't built — controller_server would crash 10s in.
_SO="${WS_DIR}/install/nav2_mppi_controller_cuda_plugin/lib/libcuda_mppi_controller.so"
if [[ ! -f "$_SO" ]]; then
  echo "ERROR: libcuda_mppi_controller.so not found."
  echo "  Build it first:"
  echo "    colcon build --symlink-install \\"
  echo "      --packages-select nav2_mppi_controller_cuda nav2_mppi_controller_cuda_plugin"
  exit 1
fi
unset _SO

# Sanity-check the ament index marker file (file-based, no runtime env needed).
_MARKER="${WS_DIR}/install/nav2_mppi_controller_cuda_plugin/share/ament_index/resource_index/packages/nav2_mppi_controller_cuda_plugin"
if [[ ! -f "$_MARKER" ]]; then
  echo "ERROR: nav2_mppi_controller_cuda_plugin ament_index marker missing."
  echo "  Build it first:"
  echo "    colcon build --symlink-install \\"
  echo "      --packages-select nav2_mppi_controller_cuda nav2_mppi_controller_cuda_plugin"
  exit 1
fi
unset _MARKER

echo "=== MuJoCo Nav Test (CUDA MPPI — CudaMPPIController) ==="
echo "  plugin : nav2_mppi_controller_cuda_plugin::CudaMPPIController"
echo "  kernels: integrate + 8 critics + cost_shape + softmax + weighted_avg"
echo "  probes : /tmp/cuda_backend_{ctor,optimize}  (if built with CUDA_PROBE=ON)"
echo ""

exec ros2 launch go2_gazebo_sim nav_test_mujoco_fastlio.launch.py "$@"
