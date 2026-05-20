#!/usr/bin/env bash
# Launch Go2 (no wheels) inside the ops2 SLAM-reconstructed scene built from
# the DBSCAN-cleaned voxel-marching-cubes mesh (bags/meshes/ops2_cuda/
# scans_v4_clean.obj). Same autonomy stack as the legacy slam_ops2 wrapper,
# only the scene mesh differs: this one is a SINGLE visual mesh (982k v / 2M
# tri) instead of 22 convex-hull tiles, with bike-rack / handrail / table
# detail preserved by the 3cm voxel + iso=0.2 + DBSCAN pipeline.
#
# Mesh is VISUAL-ONLY (contype=0 conaffinity=0). MuJoCo collides
# <geom type="mesh"> via the convex hull of the whole 80×32×13m mesh, which
# engulfs the robot's standing volume. For hard collision the next step is
# coacd convex decomposition or per-room tile splitting (see CLAUDE.md
# 2026-05-16 entry).
set -u -o pipefail

source "$(dirname "${BASH_SOURCE[0]}")/_preflight_kill.sh"
trap '' SIGHUP SIGTERM

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
# Scene variant (POLYFIT_VARIANT, default handwalls):
#   handwalls — hand-traced collision walls (draw_walls_2d.py, 2026-05-20),
#               44 segments. Cleanest geometry, no auto-fit corridor-cutting.
#               Re-editable source in mujoco/handwalls/ops2_hand_walls.json.
#   clustered — regenerated polyfit (DBSCAN + vertical-only + height-clamp).
#   real      — original 19 oversized polyfit slabs (legacy).
#   nopoly    — mesh hull only, no walls (robot may clip thin walls).
# NO_POLYFIT=1 is honored as a shortcut for nopoly (back-compat).
POLYFIT_VARIANT="${POLYFIT_VARIANT:-handwalls}"
if [[ "${NO_POLYFIT:-0}" = "1" ]]; then
  POLYFIT_VARIANT="nopoly"
fi
case "${POLYFIT_VARIANT}" in
  handwalls) SCENE="${WS_DIR}/src/go2w/go2_gazebo_sim/mujoco/slam_ops2_v4_go2_handwalls.xml" ;;
  clustered) SCENE="${WS_DIR}/src/go2w/go2_gazebo_sim/mujoco/slam_ops2_v4_go2_clustered.xml" ;;
  real)      SCENE="${WS_DIR}/src/go2w/go2_gazebo_sim/mujoco/slam_ops2_v4_go2_real.xml" ;;
  nopoly)    SCENE="${WS_DIR}/src/go2w/go2_gazebo_sim/mujoco/slam_ops2_v4_go2_nopoly.xml" ;;
  *) echo "unknown POLYFIT_VARIANT=${POLYFIT_VARIANT}" >&2; exit 1 ;;
esac

# 3d preflight: kill stale elevation_mapping state + CuPy cache
for _pat in elevation_mapping_node elevation_mapping_cupy filter_chain_runner \
            grid_map_to_occupancy_grid; do
  pkill -TERM -f "${_pat}" 2>/dev/null || true
done
sleep 1
rm -rf "${HOME}/.cupy/kernel_cache" 2>/dev/null || true

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

if [[ -n "${CONDA_PREFIX:-}" ]] && [[ -d "${CONDA_PREFIX}/lib/python3.10/site-packages/mujoco" ]]; then
  export LD_LIBRARY_PATH="${CONDA_PREFIX}/lib/python3.10/site-packages/mujoco:${LD_LIBRARY_PATH:-}"
fi
export FASTRTPS_DEFAULT_PROFILES_FILE="${WS_DIR}/config/fastdds_no_shm.xml"
export MUJOCO_INIT_KEYFRAME=home

echo "=== SLAM ops2 v4 scene + 3D autonomy (Go2) ==="
echo "  scene: $SCENE"
echo "  spawn: (0, 0, 0.32) — origin"
echo ""

# Pick the BEST fine-tuned weights as default (was previously pretrain only).
# Override with TRAV_WEIGHTS=/path/to/weights.dat if you want a specific one.
DEFAULT_FINETUNED="${WS_DIR}/training_runs/weights_ops2_tiled.dat"
DEFAULT_PRETRAIN="${WS_DIR}/training_runs/weights_pretrain.dat"
if [[ -z "${TRAV_WEIGHTS:-}" ]] && [[ -z "${TRAV_WEIGHTS_BASELINE:-}" ]]; then
  if [[ -f "${DEFAULT_FINETUNED}" ]]; then
    TRAV_WEIGHTS="${DEFAULT_FINETUNED}"
    echo "  trav weights: $(basename ${TRAV_WEIGHTS})  [FINE-TUNED, default]"
  elif [[ -f "${DEFAULT_PRETRAIN}" ]]; then
    TRAV_WEIGHTS="${DEFAULT_PRETRAIN}"
    echo "  trav weights: $(basename ${TRAV_WEIGHTS})  [pretrain, no fine-tune available]"
  fi
fi
TRAV_WEIGHTS_ARG=()
if [[ -n "${TRAV_WEIGHTS:-}" ]]; then
  if [[ ! -f "${TRAV_WEIGHTS}" ]]; then
    echo "ERROR: TRAV_WEIGHTS file not found: ${TRAV_WEIGHTS}" >&2
    exit 1
  fi
  echo "  trav CNN weights: ${TRAV_WEIGHTS}"
  TRAV_WEIGHTS_ARG=("trav_weight_file:=${TRAV_WEIGHTS}")
fi

exec ros2 launch go2_gazebo_sim nav_test_3d_explore.launch.py \
  "mujoco_model_path:=${SCENE}" \
  "spawn_x:=0.0" \
  "spawn_y:=0.0" \
  "spawn_yaw:=0.0" \
  "has_wheels:=false" \
  "upper_bound_clearance:=true" \
  "${TRAV_WEIGHTS_ARG[@]}" \
  "$@"
