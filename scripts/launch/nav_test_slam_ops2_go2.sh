#!/usr/bin/env bash
# Launch Go2 (no wheels) inside the ops2 SLAM-reconstructed scene with the
# FULL 3D autonomy stack: Fast-LIO + ETH elevation_mapping_cupy traversability
# pipeline + CFPA2 frontier exploration.
#
# Uses nav_test_3d_explore.launch.py (which has the trav pipeline) instead of
# nav_test_mujoco_fastlio.launch.py (which does NOT, leaving CFPA2 hanging on
# /robot/traversability_grid).
#
# ======================================================================
# IMPORTANT: SLAM mesh is VISUAL-ONLY (contype=0 conaffinity=0)
# ======================================================================
# The 22 SLAM-derived mesh tiles in slam_ops2_go2_real.xml are NOT physically
# collidable. They show the scanned environment visually for context but do
# NOT block the robot. Reason: MuJoCo collides <geom type="mesh"> via the
# CONVEX HULL of each mesh; for our 80×32×10m SLAM scene this engulfs the
# robot's standing volume inside one tile's hull slab → body collapses
# below floor → no walking.
#
# Workarounds tried + verdicts:
#   - 22 XY tiles, full collision   → robot trapped inside tile hull slabs
#   - 252/660 coacd convex pieces   → tiny inter-piece bumps trip feet,
#                                     robot face-plants forward
#   - <geom type="sdf">             → MuJoCo accepts it but sim freezes
#                                     (per-step SDF eval prohibitive)
# Current setup: floor (z=0 plane) is the walkable surface. SLAM mesh is
# visual reference only. CFPA2's traversability_grid (built from elevation
# mapping consuming the LIVE pointcloud) still routes around obstacles
# because /robot/cloud_registered hits the mesh visually-but-correctly.
# So planning sees the walls; physics doesn't enforce them.
#
# For HARD wall collision, the next step is a heightfield + per-wall convex
# boxes. Not done yet — see CLAUDE.md "Active state (2026-05-16)" item 1.
set -u -o pipefail

source "$(dirname "${BASH_SOURCE[0]}")/_preflight_kill.sh"
trap '' SIGHUP SIGTERM

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SCENE="${WS_DIR}/src/go2w/go2_gazebo_sim/mujoco/slam_ops2_go2_real.xml"

# 3d-only preflight: kill stale elevation_mapping state + CuPy cache
for _pat in elevation_mapping_node elevation_mapping_cupy filter_chain_runner \
            grid_map_to_occupancy_grid; do
  pkill -TERM -f "${_pat}" 2>/dev/null || true
done
sleep 1
rm -rf "${HOME}/.cupy/kernel_cache" 2>/dev/null || true

# Source env
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

# mujoco_ros2_control's libsensor.so needs libmujoco.so.3.6.0 on LD_LIBRARY_PATH
if [[ -n "${CONDA_PREFIX:-}" ]] && [[ -d "${CONDA_PREFIX}/lib/python3.10/site-packages/mujoco" ]]; then
  export LD_LIBRARY_PATH="${CONDA_PREFIX}/lib/python3.10/site-packages/mujoco:${LD_LIBRARY_PATH:-}"
fi
export FASTRTPS_DEFAULT_PROFILES_FILE="${WS_DIR}/config/fastdds_no_shm.xml"

# Initialize MuJoCo to the "home" keyframe so the robot spawns in standing pose
# (legs folded at thigh=0.9 / calf=-1.8 rad). Without this, mujoco_ros2_control
# starts at qpos0 = all zeros → legs straight down → collapse on first contact
# and CHAMP can't stand it back up. See mujoco_ros2_control_plugin.cpp:282-303.
export MUJOCO_INIT_KEYFRAME=home

echo "=== SLAM ops2 scene + 3D autonomy (Go2) ==="
echo "  scene: $SCENE"
echo "  spawn: (5.12, -8.76, 0.60) — verified open"
echo ""

exec ros2 launch go2_gazebo_sim nav_test_3d_explore.launch.py \
  "mujoco_model_path:=${SCENE}" \
  "spawn_x:=5.12" \
  "spawn_y:=-8.76" \
  "spawn_yaw:=0.0" \
  "$@"
