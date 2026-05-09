#!/usr/bin/env bash
# replay_livox_dataset.sh — replay a bag recorded by record_livox_dataset.sh.
#
# Visual parity with real_autonomy.sh: spawns the SAME RViz config the real
# robot uses (default 3d_view.rviz — has /cloud_registered + RobotModel; the
# autonomy.rviz one is also available via rviz_config=autonomy.rviz). Also
# starts robot_state_publisher + joint_state_publisher so the URDF tree
# (FL_hip, FR_thigh, livox_mid360, imu, …) is rendered on top of Fast-LIO's
# pose, even though those frames weren't in the bag. Three fixes vs. plain
# `ros2 bag play`:
#   1) Forces /tf_static back to TRANSIENT_LOCAL via --qos-profile-overrides.
#      Default replay drops it to VOLATILE, so any subscriber (including a
#      late-attached RViz) never sees the static TFs and the TF tree stays
#      broken — hence "Global Status: No tf data" in RViz.
#   2) Spawns robot_state_publisher with go2_description/xacro/robot.xacro,
#      so the URDF link tree publishes static + dynamic /tf_static + /tf
#      (the bag only carries Fast-LIO's map→camera_init→body→base_link;
#      everything below base_link came from RSP on the live robot, which
#      the bag did NOT capture).
#   3) Spawns joint_state_publisher (zero defaults) so RSP also emits the
#      DYNAMIC leg-joint frames; without this, RViz only sees the URDF's
#      static fixed joints and the legs collapse to base_link.
#
# Usage:
#   ./scripts/real/replay_livox_dataset.sh                                    # latest bag, 3d_view.rviz
#   ./scripts/real/replay_livox_dataset.sh <BAG_DIR>
#   ./scripts/real/replay_livox_dataset.sh <BAG_DIR> rviz_config=autonomy.rviz
#   ./scripts/real/replay_livox_dataset.sh <BAG_DIR> rate=2.0 loop=true
#   ./scripts/real/replay_livox_dataset.sh <BAG_DIR> rviz=false               # topic stream only
#   ./scripts/real/replay_livox_dataset.sh <BAG_DIR> rsp=false                # skip RSP (debug)
#   ./scripts/real/replay_livox_dataset.sh <BAG_DIR> start=10                 # skip first 10s
#
# Caveat: nav-stack topics referenced by autonomy.rviz / octomap.rviz
# (/robot/map, /robot/scan_3d, /robot/planned_path, …) WERE NOT recorded —
# the bag only has raw Livox + Fast-LIO output. Those displays will show
# "No data". Use 3d_view.rviz (default) for the cleanest SLAM-debug view.
#
# Stop with Ctrl+C — RViz + bag play + RSP + JSP get cleaned up.
set -eo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
REPO_ROOT="$( cd "$SCRIPT_DIR/../.." &> /dev/null && pwd )"

# ── Args ─────────────────────────────────────────────────────────────
BAG=""
RATE="1.0"
USE_RVIZ="true"
USE_RSP="true"
USE_OCTOMAP="false"  # rebuild /robot/map from /cloud_registered_body — only
                     # useful if Fast-LIO didn't drift kilometers; octomap's
                     # voxel tree (~3 km @ 0.05 m) can't index huge values.
LOOP="false"
START_OFFSET="0"
# Default to 3d_view.rviz — for SLAM/Fast-LIO drift debug it's the right
# config: shows /cloud_registered (recorded), works regardless of how far
# the pose has drifted. autonomy.rviz wants /robot/map (nav occupancy
# grid, octomap-derived) which struggles with extreme drift; switch via
# rviz_config=autonomy.rviz octomap=true if you want that view AND the
# bag was recorded indoors with bounded drift.
RVIZ_CONFIG_NAME="3d_view.rviz"

for arg in "$@"; do
  case "$arg" in
    -h|--help)
      sed -n '2,40p' "$0" ; exit 0 ;;
    rate=*)         RATE="${arg#rate=}" ;;
    rviz=*)         USE_RVIZ="${arg#rviz=}" ;;
    rsp=*)          USE_RSP="${arg#rsp=}" ;;
    octomap=*)      USE_OCTOMAP="${arg#octomap=}" ;;
    rviz_config=*)  RVIZ_CONFIG_NAME="${arg#rviz_config=}" ;;
    loop=*)         LOOP="${arg#loop=}" ;;
    start=*)        START_OFFSET="${arg#start=}" ;;
    *)
      if [[ -z "$BAG" && -d "$arg" ]]; then
        BAG="$arg"
      else
        echo "WARN: unknown arg '$arg'" >&2
      fi
      ;;
  esac
done

case "$USE_RVIZ"    in true|false) ;; *) echo "ERROR: rviz must be true|false" >&2; exit 1 ;; esac
case "$USE_RSP"     in true|false) ;; *) echo "ERROR: rsp must be true|false" >&2; exit 1 ;; esac
case "$USE_OCTOMAP" in true|false) ;; *) echo "ERROR: octomap must be true|false" >&2; exit 1 ;; esac
case "$LOOP"        in true|false) ;; *) echo "ERROR: loop must be true|false" >&2; exit 1 ;; esac

# Default: pick the latest livox_dataset_* bag.
if [[ -z "$BAG" ]]; then
  BAG="$(ls -1dt "$REPO_ROOT"/bags/livox_dataset_* 2>/dev/null | head -1)"
  if [[ -z "$BAG" ]]; then
    echo "ERROR: no livox_dataset_* bag under $REPO_ROOT/bags/. Pass <BAG_DIR>." >&2
    exit 1
  fi
  echo "[auto] picking latest bag: $BAG"
fi

# Accept a parent dir containing a rosbag2 dir.
if [[ ! -f "$BAG/metadata.yaml" ]]; then
  cand="$(find "$BAG" -maxdepth 2 -name metadata.yaml -print -quit 2>/dev/null)"
  if [[ -n "$cand" ]]; then
    BAG="$(dirname "$cand")"
  else
    echo "ERROR: no metadata.yaml under $BAG (not a finalized rosbag2 dir)." >&2
    [[ -f "$BAG/record.log" ]] && tail -20 "$BAG/record.log" >&2
    exit 1
  fi
fi

# ── Banner ───────────────────────────────────────────────────────────
echo ""
echo "################################################"
echo "  Replay (Livox dataset): $BAG"
echo "    rate=$RATE  rviz=$USE_RVIZ  loop=$LOOP  start=${START_OFFSET}s"
if [[ -f "$BAG/manifest.txt" ]]; then
  echo "  ── manifest ──"
  sed 's/^/    /' "$BAG/manifest.txt"
fi
echo "################################################"
echo ""

# ── Source ROS ───────────────────────────────────────────────────────
source /opt/ros/humble/setup.bash
[[ -f "$REPO_ROOT/install/setup.bash" ]] && source "$REPO_ROOT/install/setup.bash"

# ── tf_static QoS override (the bug fix) ─────────────────────────────
QOS_YAML="$(mktemp /tmp/replay_livox_qos.XXXXXX.yaml)"
cat > "$QOS_YAML" <<'EOF'
/tf_static:
  history: keep_all
  reliability: reliable
  durability: transient_local
EOF

# ── RViz config (use the SAME files real_autonomy.sh uses) ───────────
# Search install/ first, then src/, so the script works whether or not
# go2w_real_bringup has been built recently.
RVIZ_CONFIG=""
for cand in \
    "$REPO_ROOT/install/go2w_real_bringup/share/go2w_real_bringup/config/rviz/$RVIZ_CONFIG_NAME" \
    "$REPO_ROOT/src/go2w/go2w_real_bringup/config/rviz/$RVIZ_CONFIG_NAME"; do
  if [[ -f "$cand" ]]; then
    RVIZ_CONFIG="$cand"
    break
  fi
done
if [[ -z "$RVIZ_CONFIG" && "$USE_RVIZ" == "true" ]]; then
  echo "ERROR: RViz config '$RVIZ_CONFIG_NAME' not found under" >&2
  echo "       install/go2w_real_bringup/share/.../rviz/ or src/go2w/.../rviz/" >&2
  exit 1
fi

# ── Process tracking + cleanup ───────────────────────────────────────
RVIZ_PID=""
RSP_PID=""
OCTOMAP_PID=""
P2L_PID=""
PLAY_PID=""

cleanup_replay() {
  trap - INT TERM EXIT
  echo ""
  echo "Stopping replay..."
  if [[ -n "$PLAY_PID" ]] && kill -0 "$PLAY_PID" 2>/dev/null; then
    kill -INT "$PLAY_PID" 2>/dev/null || true
    sleep 1
    kill -9 "$PLAY_PID" 2>/dev/null || true
  fi
  for pid in "$RVIZ_PID" "$RSP_PID" "$OCTOMAP_PID" "$P2L_PID"; do
    [[ -n "$pid" ]] && kill "$pid" 2>/dev/null || true
  done
  pkill -9 -f "ros2 bag play" 2>/dev/null || true
  # description.launch.py spawns rsp under its own launch wrapper; pkill by
  # exec name catches both layers.
  pkill -9 -f "robot_state_publisher" 2>/dev/null || true
  pkill -9 -f "ros2 launch go2_description description" 2>/dev/null || true
  pkill -9 -f "octomap_server_node.*replay_livox" 2>/dev/null || true
  pkill -9 -f "pointcloud_to_laserscan_node.*replay_livox" 2>/dev/null || true
  rm -f "$QOS_YAML"
  exit 0
}
trap cleanup_replay INT TERM

# ── robot_state_publisher (URDF tree below base_link) ────────────────
# Without RSP, the bag's TF only reaches body → base_link. RSP adds the
# fixed-joint frames (trunk, imu_link, sensor mounts) below base_link by
# parsing the xacro. We DO NOT spawn joint_state_publisher: leg revolute
# frames (lf_hip / lh_thigh etc.) are dynamic and not relevant for
# Fast-LIO drift debugging — what matters is base_link / livox / IMU
# alignment, all of which are FIXED joints in the URDF.
if [[ "$USE_RSP" == "true" ]]; then
  XACRO=""
  for cand in \
      "$REPO_ROOT/install/go2_description/share/go2_description/xacro/robot.xacro" \
      "$REPO_ROOT/src/go2w/go2_description/xacro/robot.xacro"; do
    [[ -f "$cand" ]] && XACRO="$cand" && break
  done
  if [[ -z "$XACRO" ]]; then
    echo "WARN: robot.xacro not found — skipping URDF tree (TF below base_link empty)" >&2
  else
    echo "[rsp] launching go2_description description.launch.py"
    ros2 launch go2_description description.launch.py \
        use_sim_time:=true description_path:="$XACRO" \
        > /tmp/replay_livox_rsp.log 2>&1 &
    RSP_PID=$!
    echo "      RSP launch PID=$RSP_PID (log: /tmp/replay_livox_rsp.log)"
  fi
fi

# ── octomap_server: rebuild /robot/map from /cloud_registered_body ───
# Same params as real_single.launch.py:391+ (fastlio_mid360 branch).
# autonomy.rviz's BinaryMap display reads /robot/map — without this it
# stays empty since /robot/map was NOT recorded into the bag.
if [[ "$USE_OCTOMAP" == "true" ]]; then
  echo "[octomap] starting octomap_server (subscribes /cloud_registered_body, publishes /robot/map)"
  ros2 run octomap_server octomap_server_node \
      --ros-args -r __ns:=/robot -r __node:=octomap_map_gen \
      -r cloud_in:=/cloud_registered_body \
      -r projected_map:=/robot/map \
      -p use_sim_time:=true \
      -p resolution:=0.05 \
      -p frame_id:=map \
      -p base_frame_id:=base_link \
      -p sensor_model.max_range:=8.0 \
      -p sensor_model.hit:=0.8 \
      -p sensor_model.miss:=0.35 \
      -p sensor_model.min:=0.12 \
      -p sensor_model.max:=0.97 \
      -p point_cloud_min_z:=-0.40 \
      -p point_cloud_max_z:=2.00 \
      -p occupancy_min_z:=-0.17 \
      -p occupancy_max_z:=1.80 \
      -p filter_ground_plane:=true \
      -p ground_filter.distance:=0.04 \
      -p ground_filter.angle:=0.30 \
      -p ground_filter.plane_distance:=0.10 \
      -p filter_speckles:=true \
      -p latch:=true \
      -p transform_tolerance:=0.5 \
      > /tmp/replay_livox_octomap.log 2>&1 &
  OCTOMAP_PID=$!
  echo "      octomap_server PID=$OCTOMAP_PID (log: /tmp/replay_livox_octomap.log)"

  # pointcloud_to_laserscan: produces /robot/scan_3d for autonomy.rviz's
  # LaserScan display. Same params as real_bringup_core.launch.py.
  ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node \
      --ros-args -r __ns:=/robot -r __node:=pointcloud_to_laserscan \
      -r cloud_in:=/cloud_registered_body \
      -r scan:=/robot/scan_3d \
      -p use_sim_time:=true \
      -p target_frame:=base_link \
      -p transform_tolerance:=0.3 \
      -p min_height:=-0.25 \
      -p max_height:=0.60 \
      -p angle_min:=-3.14159 \
      -p angle_max:=3.14159 \
      -p angle_increment:=0.006135923151543 \
      -p scan_time:=0.1 \
      -p range_min:=0.10 \
      -p range_max:=8.0 \
      -p use_inf:=true \
      > /tmp/replay_livox_p2l.log 2>&1 &
  P2L_PID=$!
  echo "      pointcloud_to_laserscan PID=$P2L_PID"
fi

# ── Start RViz first (so it's already up when /tf_static publishes). ──
if [[ "$USE_RVIZ" == "true" ]]; then
  echo "[rviz] starting RViz with $RVIZ_CONFIG_NAME (use_sim_time=true)"
  ros2 run rviz2 rviz2 -d "$RVIZ_CONFIG" --ros-args -p use_sim_time:=true \
      > /tmp/replay_livox_rviz.log 2>&1 &
  RVIZ_PID=$!
  echo "      RViz PID=$RVIZ_PID  (log: /tmp/replay_livox_rviz.log)"
  # Give RViz a moment to subscribe before we publish /tf_static.
  sleep 3
fi

# ── Play ─────────────────────────────────────────────────────────────
PLAY_ARGS=(
  --clock
  --rate "$RATE"
  --qos-profile-overrides-path "$QOS_YAML"
)
[[ "$LOOP" == "true" ]] && PLAY_ARGS+=( --loop )
[[ "$START_OFFSET" != "0" ]] && PLAY_ARGS+=( --start-offset "$START_OFFSET" )

echo "[play] ros2 bag play ${PLAY_ARGS[*]} $BAG"
ros2 bag play "$BAG" "${PLAY_ARGS[@]}" &
PLAY_PID=$!

# Wait for play to finish (or for SIGINT trap to kick in).
wait "$PLAY_PID" 2>/dev/null || true
EXIT_CODE=$?

cleanup_replay
exit "$EXIT_CODE"
