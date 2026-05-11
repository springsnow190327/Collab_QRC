#!/usr/bin/env bash
# DARPA SubT scenes + GBPlanner3 (Docker Noetic) + Collab_QRC MuJoCo.
#
# Scenes (pass as first positional arg or scene:=<name>):
#   urban_2story    [default] — 40×40×21m 2-story warehouse with INTERNAL STAIRS
#                                (true 3D test for gbplanner3 + stair-climb)
#   pittsburgh_mine            — 252×327×2.75m DARPA SubT flat mine network
#   stairwell                  — 39×40×18m stairwell-only mini scene
#   vertical_shaft             — 33×29×34m vertical cave shaft
#
# Usage:
#   ./scripts/launch/nav_test_gbplanner_darpa.sh                       # urban_2story default
#   ./scripts/launch/nav_test_gbplanner_darpa.sh scene:=pittsburgh_mine
#   ./scripts/launch/nav_test_gbplanner_darpa.sh stop
#   ./scripts/launch/nav_test_gbplanner_darpa.sh start_mission

set -u -o pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
UAS_REPO_ROOT="${UAS_REPO_ROOT:-$HOME/Research/uas_deploy/unified_autonomy_stack}"
COLLAB_QRC_ROOT="${COLLAB_QRC_ROOT:-$WS_DIR}"
OVERLAY_COMPOSE="${COLLAB_QRC_ROOT}/scripts/sim/gbplanner3_mujoco/compose/docker-compose.collab_qrc.yml"

# Parse scene= positional arg (allow either "scene:=X" or unprefixed first arg)
SCENE_NAME="urban_2story"
NON_SCENE_ARGS=()
for arg in "$@"; do
  case "$arg" in
    scene:=*) SCENE_NAME="${arg#scene:=}";;
    stop|start_mission) ;;       # handled below
    *) NON_SCENE_ARGS+=("$arg");;
  esac
done

# ===== Subcommands =====
case "${1:-}" in
  stop)
    echo "==> Stopping gbplanner3 containers..."
    cd "$UAS_REPO_ROOT"
    UAS_REPO_ROOT="$UAS_REPO_ROOT" COLLAB_QRC_ROOT="$COLLAB_QRC_ROOT" \
      make stop DOCKER_COMPOSE_FILE="$OVERLAY_COMPOSE" 2>&1 | tail -10
    pkill -f "static_transform_publisher.*base_to_lidar_alias_gbplanner" 2>/dev/null || true
    pkill -f "static_transform_publisher.*world_to_map_tf_gbplanner" 2>/dev/null || true
    source "$(dirname "${BASH_SOURCE[0]}")/_preflight_kill.sh"
    exit 0
    ;;
  start_mission)
    GBP=$(docker ps --format "{{.Names}}" | grep gbplanner | head -1)
    [[ -z "$GBP" ]] && { echo "ERROR: no gbplanner container running"; exit 1; }
    docker exec -t "$GBP" bash -lc '
      source /opt/ros/noetic/setup.bash
      rosservice call /planner_control_interface/std_srvs/automatic_planning "{}"
    '
    exit 0
    ;;
esac

# Map scene name → MJCF path + 2D area
case "$SCENE_NAME" in
  urban_2story)     SCENE_FILE="urban_2story_go2.xml";   AREA=1600;;
  pittsburgh_mine)  SCENE_FILE="pittsburgh_mine_go2.xml"; AREA=82000;;
  stairwell)        SCENE_FILE="stairwell_go2.xml";       AREA=1560;;
  vertical_shaft)   SCENE_FILE="vertical_shaft_go2.xml";  AREA=957;;
  *) echo "ERROR: unknown scene '$SCENE_NAME'"; exit 1;;
esac
SCENE="${WS_DIR}/src/go2w/go2_gazebo_sim/mujoco/${SCENE_FILE}"
[[ -f "$SCENE" ]] || { echo "ERROR: $SCENE not found"; exit 1; }
echo "==> Scene: $SCENE_NAME ($SCENE_FILE, ${AREA} m²)"

# ===== Full start =====
echo "==> [1/4] Preflight kill..."
source "$(dirname "${BASH_SOURCE[0]}")/_preflight_kill.sh"
if docker ps --format "{{.Names}}" | grep -q unified_autonomy_stack; then
  cd "$UAS_REPO_ROOT" && \
    UAS_REPO_ROOT="$UAS_REPO_ROOT" COLLAB_QRC_ROOT="$COLLAB_QRC_ROOT" \
    make stop DOCKER_COMPOSE_FILE="$OVERLAY_COMPOSE" 2>/dev/null | tail -3
fi

echo ""
echo "==> [2/4] Start Docker stack..."
cd "$UAS_REPO_ROOT"
nohup env \
  UAS_REPO_ROOT="$UAS_REPO_ROOT" \
  COLLAB_QRC_ROOT="$COLLAB_QRC_ROOT" \
  DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
  make launch DOCKER_COMPOSE_FILE="$OVERLAY_COMPOSE" \
  > /tmp/gbplanner3_darpa_uas.log 2>&1 &
disown
echo "    log: /tmp/gbplanner3_darpa_uas.log"

echo "==> Waiting up to 60s for gbplanner_node..."
for i in $(seq 1 60); do
  if docker ps --format "{{.Names}}" | grep -q gbplanner; then
    GBP=$(docker ps --format "{{.Names}}" | grep gbplanner | head -1)
    if docker exec "$GBP" bash -lc 'source /opt/ros/noetic/setup.bash && rosnode list 2>/dev/null | grep -q gbplanner_node' 2>/dev/null; then
      echo "    ✓ gbplanner_node up"; break
    fi
  fi
  sleep 1
  [[ $i -eq 60 ]] && { echo "    ✗ TIMEOUT"; exit 1; }
done

echo ""
echo "==> [3/4] Source env + static TF aliases..."
safe_source() { set +u; source "$1"; set -u; }
if [[ -f "${HOME}/miniforge3/etc/profile.d/conda.sh" ]]; then
  safe_source "${HOME}/miniforge3/etc/profile.d/conda.sh"
  conda activate cmu_env
elif command -v micromamba >/dev/null 2>&1; then
  eval "$(micromamba shell hook -s bash)"
  micromamba activate cmu_env
fi
safe_source /opt/ros/humble/setup.bash
safe_source "${WS_DIR}/install/setup.bash"

# CRITICAL fix: see nav_test_gbplanner_demo3.sh for the long explanation.
# mujoco_ros2_control sensor plugins need libmujoco.so.3.6.0 on LD_LIBRARY_PATH.
if [[ -n "${CONDA_PREFIX:-}" ]] && [[ -d "${CONDA_PREFIX}/lib/python3.10/site-packages/mujoco" ]]; then
  export LD_LIBRARY_PATH="${CONDA_PREFIX}/lib/python3.10/site-packages/mujoco:${LD_LIBRARY_PATH:-}"
fi

# base_link → lidar
nohup ros2 run tf2_ros static_transform_publisher \
  --x 0.16143 --y 0.0 --z 0.12262 \
  --qx 0.0 --qy 0.11321 --qz 0.0 --qw 0.99357 \
  --frame-id base_link --child-frame-id lidar \
  --ros-args -r __node:=base_to_lidar_alias_gbplanner \
  -r /tf:=/robot/tf -r /tf_static:=/robot/tf_static \
  > /tmp/gbplanner3_tf_lidar.log 2>&1 &
disown

# world → map
nohup ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 \
  --frame-id world --child-frame-id map \
  --ros-args -r __node:=world_to_map_tf_gbplanner \
  -r /tf:=/robot/tf -r /tf_static:=/robot/tf_static \
  > /tmp/gbplanner3_tf_world_map.log 2>&1 &
disown
echo "    static TF nodes started"

# Humble TF stays namespaced (/robot/tf). The /tf relay is on the Noetic
# side, inside the gbplanner container — see docker-compose.collab_qrc.yml.

echo ""
echo "==> [4/4] Start MuJoCo + Fast-LIO..."
# PREFLIGHT_KILL=0 → skip nav_test_fastlio.sh's own preflight; we already ran it.
# (Otherwise pipefail in _preflight_stop_ros2_daemon when no daemon alive aborts
#  the script silently under set -euo pipefail.)
cd "$WS_DIR"
exec env PREFLIGHT_KILL=0 "${WS_DIR}/scripts/launch/nav_test_fastlio.sh" \
  "mujoco_model_path:=${SCENE}" \
  "scene_area_m2:=${AREA}" \
  "has_wheels:=false" \
  "two_way_drive:=false" \
  "explore:=false" \
  "${NON_SCENE_ARGS[@]}"
