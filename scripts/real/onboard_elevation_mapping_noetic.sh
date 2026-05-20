#!/usr/bin/env bash
# onboard_elevation_mapping_noetic.sh — start the traversability pipeline on the Jetson.
#
# Prerequisites (run first in separate terminals):
#   ~/noetic_fastlio_ws/scripts/onboard_pointlio_noetic.sh
#   (or onboard_fastlio_noetic.sh for FAST-LIO2)
#
# What this starts:
#   roslaunch trav_pipeline_ros1 trav_pipeline.launch
#
#   → elevation_mapping_cupy  (/robot/cloud_registered_body → /robot/elevation_map_raw)
#   → trav_filter_occ_grid    (/robot/elevation_map_raw → /robot/traversability_grid)
#
# The /robot/traversability_grid OccupancyGrid is then bridged to the laptop
# via ros1_bridge → Nav2 global_costmap static_layer on the ROS 2 side.
# (Add nav_msgs/OccupancyGrid to bridge_topics.yaml if not already there.)
#
# Usage:
#   ./onboard_elevation_mapping_noetic.sh                      # default weights
#   ./onboard_elevation_mapping_noetic.sh weights=<path.dat>   # custom weights
#   ./onboard_elevation_mapping_noetic.sh stop                 # kill all nodes
#
# Ctrl+C terminates cleanly via trap.

set -e
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
WS_ROOT="/home/unitree/noetic_fastlio_ws"

# ── Strip miniconda (same guard as onboard_fastlio_noetic.sh) ─────────
if [[ -n "${CONDA_PREFIX:-}" ]] || echo "$PATH" | grep -q miniconda; then
  type conda 2>/dev/null | head -1 | grep -q function && conda deactivate 2>/dev/null || true
  export PATH="$(echo "$PATH" | tr ':' '\n' | grep -vE "(miniconda|conda)" | tr '\n' ':' | sed 's/:$//')"
  unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PROMPT_MODIFIER CONDA_SHLVL CONDA_PYTHON_EXE
fi
unset PYTHONPATH PYTHONHOME

# ── Parse args ────────────────────────────────────────────────────────
WEIGHTS=""
for arg in "$@"; do
  case "$arg" in
    weights=*) WEIGHTS="${arg#weights=}" ;;
    stop)
      echo "  Stopping elevation_mapping + trav_filter_occ_grid..."
      rosnode kill /robot/elevation_mapping /robot/trav_filter_occ_grid 2>/dev/null || true
      exit 0
      ;;
    *) echo "WARN: unknown arg '$arg'" >&2 ;;
  esac
done

# ── Source ROS + workspace ────────────────────────────────────────────
source /opt/ros/noetic/setup.bash
source "${WS_ROOT}/devel/setup.bash"

# ── Check roscore is up ───────────────────────────────────────────────
if ! rostopic list &>/dev/null; then
  echo "ERROR: roscore is not running."
  echo "  Start SLAM first: ~/noetic_fastlio_ws/scripts/onboard_pointlio_noetic.sh"
  echo "  (roscore is started by the SLAM launcher)"
  exit 1
fi

# ── Check Point-LIO / FAST-LIO is publishing the input cloud ─────────
if ! rostopic info /robot/cloud_registered_body &>/dev/null; then
  echo "WARN: /robot/cloud_registered_body not published yet."
  echo "      The pipeline will start but elevation_mapping will block"
  echo "      waiting for the first point cloud."
fi

# ── Determine weights path ────────────────────────────────────────────
# Prefer ops2-tuned weights if they exist next to the default weights.
PKG_SHARE="$(rospack find trav_pipeline_ros1)"
PKG_WEIGHTS_DIR="${PKG_SHARE}/config"
OPS2_WEIGHTS="${PKG_WEIGHTS_DIR}/weights_ops2_tiled.dat"
PRETRAIN_WEIGHTS="${PKG_WEIGHTS_DIR}/weights_pretrain.dat"

if [[ -n "$WEIGHTS" ]]; then
  WEIGHT_ARG="weight_file:=${WEIGHTS}"
  echo "  weights : ${WEIGHTS} (from arg)"
elif [[ -f "$OPS2_WEIGHTS" ]]; then
  WEIGHT_ARG="weight_file:=${OPS2_WEIGHTS}"
  echo "  weights : ${OPS2_WEIGHTS} (ops2 fine-tune)"
elif [[ -f "$PRETRAIN_WEIGHTS" ]]; then
  WEIGHT_ARG="weight_file:=${PRETRAIN_WEIGHTS}"
  echo "  weights : ${PRETRAIN_WEIGHTS} (pretrain fallback)"
else
  WEIGHT_ARG=""
  echo "  weights : pkg default (elevation_mapping_cupy/config/core/weights.dat)"
fi

# ── Trap for clean shutdown ───────────────────────────────────────────
cleanup() {
  echo ""
  echo "  Shutting down traversability pipeline..."
  rosnode kill /robot/elevation_mapping /robot/trav_filter_occ_grid 2>/dev/null || true
}
trap cleanup INT TERM

# ── Launch ────────────────────────────────────────────────────────────
echo ""
echo "################################################"
echo "  Traversability pipeline (Noetic)"
echo "  elevation_mapping_cupy + trav_filter_occ_grid"
echo "  output: /robot/traversability_grid (OccupancyGrid)"
echo "  bridge: add nav_msgs/OccupancyGrid to bridge_topics.yaml"
echo "################################################"
echo ""

roslaunch trav_pipeline_ros1 trav_pipeline.launch \
  ${WEIGHT_ARG:+"$WEIGHT_ARG"} \
  "$@"
