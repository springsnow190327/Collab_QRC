#!/usr/bin/env bash
# Go2 (non-W) MuJoCo sim with TARE+FAR layered goal chain.
#
# Wraps nav_test_go2_demo3 with TARE inserted between CFPA2 and FAR:
#
#   CFPA2 → /way_point_coord (frontier seed, 2 Hz)
#           → tare_planner_node → /way_point_tare (5 Hz, reachable)
#             → waypoint_mux → /way_point_coord_nav (fallback: CFPA2 raw)
#                → FAR global planner
#
# Why: on demo3, CFPA2 can assign unreachable frontier goals (behind
# pillars / in unobserved space). FAR then silently stops publishing
# waypoints and the robot sits `STUCK(N)` forever. TARE picks reachable
# viewpoints by construction. See docs/claude/go2_integration.md for the
# full stuck-diagnosis timeline.
#
# Usage:
#   ./scripts/launch/nav_test_go2_tare.sh                    # demo3 default
#   ./scripts/launch/nav_test_go2_tare.sh gui:=false         # headless
#   ./scripts/launch/nav_test_go2_tare.sh mujoco_model_path:=... scene_area_m2:=96  # demo1
set -u -o pipefail

# Kill any stale nav/sim processes from a prior launch (see _preflight_kill.sh).
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

# SC-PGO from the LRC stack (same as nav_test_fastlio.sh).
SC_PGO_PREFIX="${HOME}/COMP0225_LRC_stack/install/sc_pgo"
if [[ -d "${SC_PGO_PREFIX}" ]]; then
  export AMENT_PREFIX_PATH="${SC_PGO_PREFIX}:${AMENT_PREFIX_PATH}"
  export LD_LIBRARY_PATH="${SC_PGO_PREFIX}/lib:${LD_LIBRARY_PATH}"
fi

export FASTRTPS_DEFAULT_PROFILES_FILE="${WS_DIR}/config/fastdds_no_shm.xml"

echo "=== MuJoCo Nav Test (Go2 + TARE + FAR) ==="
exec ros2 launch go2_gazebo_sim nav_test_go2_tare.launch.py "$@"
