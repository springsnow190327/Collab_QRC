#!/usr/bin/env bash
# nav_test_hil_desktop.sh — desktop half of the Orin Nano HIL test bench.
#
# Runs ONLY: MuJoCo + sensor publishers + CHAMP control + RViz.
# Skips: fast_lio, elevation_mapping, filter_chain, nav2, cfpa2 — those all
# run on the Jetson via scripts/real/run_jetson_hil.sh.
#
# Companion to scripts/real/run_jetson_hil.sh (Jetson side).
#
# Frame conventions agreed with Jetson side:
#   - Jetson runs fast_lio + fast_lio_tf_adapter → publishes `map → base_link` TF
#   - Desktop publishes `base_link → body` static + joint TFs (CHAMP/robot_state_publisher)
#   - Desktop does NOT publish map / odom / world dynamic TFs (avoid TF conflict)
#
# Usage:
#   ./scripts/launch/nav_test_hil_desktop.sh                    # default ops2 scene
#   NO_POLYFIT=1 ./scripts/launch/nav_test_hil_desktop.sh       # lighter scene
#   ./scripts/launch/nav_test_hil_desktop.sh rviz:=false gui:=false  # headless

set -u -o pipefail

source "$(dirname "${BASH_SOURCE[0]}")/_preflight_kill.sh"
trap '' SIGHUP SIGTERM

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
# Scene variant selection:
#   POLYFIT_VARIANT=clustered (default) — regenerated polyfit walls (2026-05-20):
#       per-segment DBSCAN clustering + vertical-only + 4m height clamp + dedup.
#       68 tight wall boxes vs the original 19 oversized 47×10m slabs that
#       overfit and cut across corridors. Keeps sim-side collision geometry.
#   POLYFIT_VARIANT=real     — original 19 oversized polyfit walls (legacy).
#   POLYFIT_VARIANT=nopoly   — no polyfit walls at all (mesh hull only; robot
#       may clip thin walls). Equivalent to old NO_POLYFIT=1.
# NO_POLYFIT=1 still honored as a shortcut for nopoly (back-compat).
#   POLYFIT_VARIANT=handwalls (default) — hand-traced walls (draw_walls_2d.py,
#       2026-05-20): 33 wall segments drawn by hand on the building heightmap.
#       Cleanest collision geometry, no auto-fit artifacts. Source artifacts in
#       bags/meshes/ops2_cuda/handwalls/.
POLYFIT_VARIANT="${POLYFIT_VARIANT:-handwalls}"
if [[ "${NO_POLYFIT:-0}" = "1" ]]; then
  POLYFIT_VARIANT="nopoly"
fi
case "${POLYFIT_VARIANT}" in
  handwalls) SCENE="${WS_DIR}/src/go2w/go2_gazebo_sim/mujoco/slam_ops2_v4_go2_handwalls.xml" ;;
  clustered) SCENE="${WS_DIR}/src/go2w/go2_gazebo_sim/mujoco/slam_ops2_v4_go2_clustered.xml" ;;
  nopoly)    SCENE="${WS_DIR}/src/go2w/go2_gazebo_sim/mujoco/slam_ops2_v4_go2_nopoly.xml" ;;
  real)      SCENE="${WS_DIR}/src/go2w/go2_gazebo_sim/mujoco/slam_ops2_v4_go2_real.xml" ;;
  *) echo "unknown POLYFIT_VARIANT=${POLYFIT_VARIANT}" >&2; exit 1 ;;
esac

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

echo "=== HIL desktop side — MuJoCo + sensors + CHAMP + RViz ==="
echo "  scene: $SCENE"
echo "  Jetson companion : scripts/real/run_jetson_hil.sh"
echo ""

# Call the base launch directly, skipping all autonomy nodes.
# Key flags:
#   enable_assets=true             : MuJoCo + robot spawn
#   enable_perception=true         : sensor bridges (lidar/imu/contacts)
#   enable_slam=false              : NO fast_lio on desktop (Jetson owns SLAM)
#   enable_control=true            : CHAMP cmd_vel → joint torques (closed loop)
#   enable_navigation=false        : NO nav2 on desktop (Jetson owns navigation)
#   use_fast_lio=true              : keep pointcloud_adapter active
#   odom_bridge_publish_tf=false   : Jetson SLAM owns map → base_link;
#                                    don't publish a conflicting `odom →
#                                    base_link` from the MuJoCo bridge
#                                    (two parents for base_link → TF rejects).
#   (also: enable_slam=false gates the imu_to_body_tf static off — that
#    one used to fire in standalone sim to give the scan pipeline a body
#    frame, but Jetson SLAM publishes camera_init → body itself.)
# Richer RViz config: nav_test.rviz has GridMap displays for elevation_map_raw
# / filtered, scan_3d, voxels_cloud, plus the SetGoal tool bound to
# /robot/goal_pose (2D Goal Pose button → straight to Jetson Nav2). Override
# with rviz_config:=<abs path> if you want the lighter single_go2w_gazebo_cfpa2.
HIL_RVIZ_CONFIG="${HIL_RVIZ_CONFIG:-$(ros2 pkg prefix go2_gazebo_sim 2>/dev/null)/share/go2_gazebo_sim/rviz/nav_test.rviz}"

exec ros2 launch go2_gazebo_sim single_go2w_mujoco_cfpa2.launch.py \
  "mujoco_model_path:=${SCENE}" \
  "robot_namespace:=robot" \
  "use_sim_time:=true" \
  "spawn_x:=0.0" "spawn_y:=0.0" "spawn_yaw:=0.0" \
  "has_wheels:=false" \
  "enable_assets:=true" \
  "enable_perception:=true" \
  "enable_slam:=false" \
  "use_fast_lio:=true" \
  "enable_control:=true" \
  "enable_navigation:=false" \
  "odom_bridge_publish_tf:=false" \
  "cleanup_stale:=true" \
  "rviz_config:=${HIL_RVIZ_CONFIG}" \
  "$@"
