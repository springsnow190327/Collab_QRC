#!/usr/bin/env bash
# 3D-frontier exploration sim: demo_ramp scene + Point-LIO / Fast-LIO +
# nvblox_frontend (CUDA 3D mapping on RTX 4050) + CFPA2.
#
# Two run modes:
#   ./scripts/launch/nav_test_3d_explore.sh                       # 3D IG (default)
#   IG_DIM=2d ./scripts/launch/nav_test_3d_explore.sh             # baseline 2D IG (A/B)
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

IG_DIM="${IG_DIM:-3d}"
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
  python3 - "${CFPA2_SRC_CONFIG}" <<'PY'
import sys, os, yaml
cfg_dir = sys.argv[1]
base = yaml.safe_load(open(os.path.join(cfg_dir, "cfpa2_single_robot.yaml.bak.2d")))
overlay = yaml.safe_load(open(os.path.join(cfg_dir, "cfpa2_single_robot_3d.yaml")))
def merge(a, b):
    for k, v in b.items():
        if isinstance(v, dict) and isinstance(a.get(k), dict):
            merge(a[k], v)
        else:
            a[k] = v
merge(base, overlay)
with open(os.path.join(cfg_dir, "cfpa2_single_robot.yaml"), "w") as f:
    yaml.safe_dump(base, f, sort_keys=False)
print(f"[nav_test_3d_explore] composed cfpa2_single_robot.yaml = 2D base + 3D overlay")
PY
elif [[ "${IG_DIM}" == "2d" ]]; then
  # Restore the original 2D yaml if a previous 3d run mutated it.
  if [[ -f "${CFPA2_SRC_CONFIG}/cfpa2_single_robot.yaml.bak.2d" ]]; then
    cp "${CFPA2_SRC_CONFIG}/cfpa2_single_robot.yaml.bak.2d" \
       "${CFPA2_SRC_CONFIG}/cfpa2_single_robot.yaml"
    echo "[nav_test_3d_explore] restored cfpa2_single_robot.yaml (2D baseline)"
  fi
else
  echo "ERROR: IG_DIM='${IG_DIM}' invalid (use '2d' or '3d')" >&2
  exit 1
fi

echo "=== 3D Frontier Exploration Sim ==="
echo "  scene:        src/go2w/go2_gazebo_sim/mujoco/demo_ramp.xml"
echo "  ig_dimension: ${IG_DIM}"
echo "  nvblox vox:   0.10 m on RTX 4050 (CUDA 12.6)"
echo ""

exec ros2 launch go2_gazebo_sim nav_test_3d_explore.launch.py "$@"
