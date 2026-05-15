#!/usr/bin/env bash
# 3D-frontier exploration sim: demo_ramp scene + Point-LIO / Fast-LIO +
# ETH elevation_mapping_cupy traversability + optional nvblox 3D IG + CFPA2.
#
# Two run modes:
#   ./scripts/launch/nav_test_3d_explore.sh                       # ETH elevation traversability, 2D IG
#   ./scripts/launch/nav_test_3d_explore.sh enable_nvblox_mapper:=true  # 3D IG
#   IG_DIM=3d ./scripts/launch/nav_test_3d_explore.sh enable_nvblox_mapper:=true
#
# The IG_DIM switch swaps which cfpa2_single_robot.yaml gets symlinked into
# the cfpa2 install share before launch — the base launch hardcodes the
# yaml path, so we override at the symlink layer instead of touching it.
set -euo pipefail
# Shield the script from signals that propagate when preflight kills a stale
# ros2 launch process (which may be the session leader of this terminal).
# SIGHUP: terminal loss / session leader death.
# SIGTERM: process-group broadcast when a group leader is killed.
# Both are restored to default after preflight by exec → ros2 launch.
trap '' SIGHUP SIGTERM

source "$(dirname "${BASH_SOURCE[0]}")/_preflight_kill.sh"

trap - SIGTERM   # restore SIGTERM so Ctrl+C-based kills work on ros2 launch

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

# CRITICAL: mujoco_ros2_control loads MuJoCo sensor plugins from cmu_env's
# site-packages/mujoco/plugin/, but those plugins are linked against
# libmujoco.so.3.6.0 which cmu_env's conda activation does NOT add to
# LD_LIBRARY_PATH. Without this export, libsensor.so fails to dlopen →
# MuJoCo can't bind <sensor> blocks in the MJCF → MuJoCo GUI fails to come
# up and there's no IMU / LiDAR / pose stream for Fast-LIO. Same gotcha as
# nav_test_gbplanner_demo3.sh; see feedback_libmujoco_ld_path memory.
if [[ -n "${CONDA_PREFIX:-}" ]] && [[ -d "${CONDA_PREFIX}/lib/python3.10/site-packages/mujoco" ]]; then
  export LD_LIBRARY_PATH="${CONDA_PREFIX}/lib/python3.10/site-packages/mujoco:${LD_LIBRARY_PATH:-}"
fi

# CuPy wheels only bundle part of the CUDA user-space stack. On machines with
# an NVIDIA driver but no full CUDA toolkit, NVRTC and CUDA headers come from
# pip packages (nvidia-cuda-nvrtc-cu12, nvidia-cuda-runtime-cu12). Point CuPy at
# those paths so elevation_mapping_cupy can JIT kernels without system sudo.
if [[ -n "${CONDA_PREFIX:-}" ]]; then
  CUDA_PIP_ROOT="${CONDA_PREFIX}/lib/python3.10/site-packages/nvidia"
  if [[ -d "${CUDA_PIP_ROOT}/cuda_runtime" ]]; then
    export CUDA_PATH="${CUDA_PATH:-${CUDA_PIP_ROOT}/cuda_runtime}"
    export LD_LIBRARY_PATH="${CUDA_PIP_ROOT}/cuda_runtime/lib:${LD_LIBRARY_PATH:-}"
  fi
  if [[ -d "${CUDA_PIP_ROOT}/cuda_nvrtc/lib" ]]; then
    export LD_LIBRARY_PATH="${CUDA_PIP_ROOT}/cuda_nvrtc/lib:${LD_LIBRARY_PATH:-}"
  fi
fi

SC_PGO_PREFIX="${HOME}/COMP0225_LRC_stack/install/sc_pgo"
if [[ -d "${SC_PGO_PREFIX}" ]]; then
  export AMENT_PREFIX_PATH="${SC_PGO_PREFIX}:${AMENT_PREFIX_PATH}"
  export LD_LIBRARY_PATH="${SC_PGO_PREFIX}/lib:${LD_LIBRARY_PATH}"
fi

export FASTRTPS_DEFAULT_PROFILES_FILE="${WS_DIR}/config/fastdds_no_shm.xml"

# Dense Mid-360 sim: default 1000×20 rays = 2.95° vertical spacing → walls
# show only 1-3 voxels tall in nvblox at 1 m range. Override to 1024×96 →
# ~0.61° spacing → walls fill in vertically (every 0.6 cm at 1 m, ~16 voxels
# per 1 m wall). mj_multiRay handles ~1M rays/sec, this needs only 1M/sec.
export MUJOCO_LIDAR_HZ_SAMPLES="${MUJOCO_LIDAR_HZ_SAMPLES:-1024}"
export MUJOCO_LIDAR_VT_SAMPLES="${MUJOCO_LIDAR_VT_SAMPLES:-96}"
export STUCK_RAMP_SUPPRESS_ENABLED="${STUCK_RAMP_SUPPRESS_ENABLED:-true}"
export STUCK_RAMP_MIN_X="${STUCK_RAMP_MIN_X:-5.3}"
export STUCK_RAMP_MAX_X="${STUCK_RAMP_MAX_X:-9.8}"
export STUCK_RAMP_MAX_ABS_Y="${STUCK_RAMP_MAX_ABS_Y:-0.9}"
export STUCK_RAMP_MIN_FORWARD_GOAL_M="${STUCK_RAMP_MIN_FORWARD_GOAL_M:-0.15}"
export STUCK_WATCHDOG_WINDOW_SEC="${STUCK_WATCHDOG_WINDOW_SEC:-9999.0}"

ENABLE_NVBLOX_REQUESTED=0
NAV_COSTMAP_MODE="3d"
for arg in "$@"; do
  case "${arg}" in
    enable_nvblox_mapper:=true|enable_nvblox_mapper:=True|enable_nvblox_mapper:=1)
      ENABLE_NVBLOX_REQUESTED=1
      ;;
    nav_costmap_mode:=2d)
      NAV_COSTMAP_MODE="2d"
      ;;
    nav_costmap_mode:=3d)
      NAV_COSTMAP_MODE="3d"
      ;;
  esac
done

if [[ -z "${IG_DIM:-}" ]]; then
  if [[ "${ENABLE_NVBLOX_REQUESTED}" == "1" ]]; then
    IG_DIM="3d"
  else
    IG_DIM="2d"
  fi
fi

if [[ "${IG_DIM}" == "3d" && "${ENABLE_NVBLOX_REQUESTED}" != "1" ]]; then
  echo "WARN: IG_DIM=3d requested without enable_nvblox_mapper:=true; CFPA2 needs /robot/voxels_3d for true 3D IG." >&2
fi
CFPA2_INSTALL_CONFIG="${WS_DIR}/install/cfpa2_collaborative_autonomy/share/cfpa2_collaborative_autonomy/config"
CFPA2_SRC_CONFIG="${WS_DIR}/src/collaborative_exploration/cfpa2_collaborative_autonomy/config"

if [[ "${IG_DIM}" == "3d" ]]; then
  if [[ ! -f "${CFPA2_SRC_CONFIG}/cfpa2_single_robot_3d.yaml" ]]; then
    echo "ERROR: missing ${CFPA2_SRC_CONFIG}/cfpa2_single_robot_3d.yaml" >&2
    exit 1
  fi
  # The base launch hardcodes cfpa2_single_robot.yaml. We backup the 2D yaml
  # and overlay the 3D one. (--symlink-install means the install/ tree is just
  # symlinks back to src/, so editing src/ is sufficient.)
  # Create the 2D backup only on first run; subsequent runs must not
  # overwrite it with the already-merged 3D yaml.
  if [[ ! -f "${CFPA2_SRC_CONFIG}/cfpa2_single_robot.yaml.bak.2d" ]]; then
    cp "${CFPA2_SRC_CONFIG}/cfpa2_single_robot.yaml" \
       "${CFPA2_SRC_CONFIG}/cfpa2_single_robot.yaml.bak.2d"
  fi
  # Compose: base 2D yaml + 3D overlay → cfpa2_single_robot.yaml (in-place).
  # Order matters: overlay's keys win.
python3 - "${CFPA2_SRC_CONFIG}" "${CFPA2_MAX_GOAL_DISTANCE_M:-2.5}" <<'PY'
import sys, os, yaml
cfg_dir, max_goal_distance_m = sys.argv[1], float(sys.argv[2])
base = yaml.safe_load(open(os.path.join(cfg_dir, "cfpa2_single_robot.yaml.bak.2d")))
overlay = yaml.safe_load(open(os.path.join(cfg_dir, "cfpa2_single_robot_3d.yaml")))
def merge(a, b):
    for k, v in b.items():
        if isinstance(v, dict) and isinstance(a.get(k), dict):
            merge(a[k], v)
        else:
            a[k] = v
merge(base, overlay)
params = base["/**"]["ros__parameters"]
params["cfpa2_max_goal_distance_m"] = max_goal_distance_m
params["ramp_ascent_enabled"] = True
params["ramp_ascent_goal_topic_suffix"] = "/ramp_ascent_goal"
params["ramp_ascent_max_goal_distance_m"] = float(os.environ.get("RAMP_ASCENT_MAX_GOAL_DISTANCE_M", "5.0"))
params["ramp_ascent_goal_stale_sec"] = 8.0
params["ramp_ascent_require_grid_reachable"] = False
params["ramp_ascent_exclusive"] = True
params["ramp_ascent_ignore_blacklist"] = True
params["ramp_ascent_switch_min_dist_m"] = 0.25
params["ramp_ascent_utility"] = 100000.0
params["ramp_ascent_corridor_lock_sec"] = 20.0
params["ramp_ascent_lock_min_x"] = 5.3
params["ramp_ascent_lock_max_x"] = 13.0
params["ramp_ascent_lock_max_abs_y"] = 1.2
params["startup_delay_sec"] = float(os.environ.get("CFPA2_STARTUP_DELAY_SEC", "24.0"))
with open(os.path.join(cfg_dir, "cfpa2_single_robot.yaml"), "w") as f:
    yaml.safe_dump(base, f, sort_keys=False)
print(f"[nav_test_3d_explore] composed cfpa2_single_robot.yaml = 2D base + 3D overlay")
PY
elif [[ "${IG_DIM}" == "2d" ]]; then
  # 2D IG does not require /voxels_3d, but in nav_costmap_mode:=3d it must
  # still use the elevation traversability OccupancyGrid for frontier BFS.
  python3 - "${CFPA2_SRC_CONFIG}" "${NAV_COSTMAP_MODE}" "${CFPA2_MAX_GOAL_DISTANCE_M:-2.5}" <<'PY'
import sys, os, yaml
cfg_dir, nav_costmap_mode, max_goal_distance_m = sys.argv[1], sys.argv[2], float(sys.argv[3])
base_path = os.path.join(cfg_dir, "cfpa2_single_robot.yaml.bak.2d")
if not os.path.exists(base_path):
    base_path = os.path.join(cfg_dir, "cfpa2_single_robot.yaml")
cfg = yaml.safe_load(open(base_path))
params = cfg["/**"]["ros__parameters"]
params["planning_map_topic_suffix"] = (
    "/traversability_grid" if nav_costmap_mode == "3d" else "/map"
)
params["ig_dimension"] = "2d"
params["cfpa2_max_goal_distance_m"] = max_goal_distance_m
params["ramp_ascent_enabled"] = (nav_costmap_mode == "3d")
params["ramp_ascent_goal_topic_suffix"] = "/ramp_ascent_goal"
params["ramp_ascent_max_goal_distance_m"] = float(os.environ.get("RAMP_ASCENT_MAX_GOAL_DISTANCE_M", "5.0"))
params["ramp_ascent_goal_stale_sec"] = 8.0
params["ramp_ascent_require_grid_reachable"] = False
params["ramp_ascent_exclusive"] = True
params["ramp_ascent_ignore_blacklist"] = True
params["ramp_ascent_switch_min_dist_m"] = 0.25
params["ramp_ascent_utility"] = 100000.0
params["ramp_ascent_corridor_lock_sec"] = 20.0
params["ramp_ascent_lock_min_x"] = 5.3
params["ramp_ascent_lock_max_x"] = 13.0
params["ramp_ascent_lock_max_abs_y"] = 1.2
params["startup_delay_sec"] = float(os.environ.get("CFPA2_STARTUP_DELAY_SEC", "24.0"))
with open(os.path.join(cfg_dir, "cfpa2_single_robot.yaml"), "w") as f:
    yaml.safe_dump(cfg, f, sort_keys=False)
print(
    "[nav_test_3d_explore] composed cfpa2_single_robot.yaml = "
    f"2D IG + planning_map={params['planning_map_topic_suffix']}"
)
PY
else
  echo "ERROR: IG_DIM='${IG_DIM}' invalid (use '2d' or '3d')" >&2
  exit 1
fi

echo "=== 3D Frontier Exploration Sim ==="
echo "  scene:        src/go2w/go2_gazebo_sim/mujoco/demo_ramp.xml"
echo "  ig_dimension: ${IG_DIM}"
echo "  trav source:  elevation_mapping_cupy + grid_map ETH-style filters"
echo "  nvblox vox:   optional 0.10 m stream when enable_nvblox_mapper:=true"
echo ""

exec ros2 launch go2_gazebo_sim nav_test_3d_explore.launch.py "$@"
