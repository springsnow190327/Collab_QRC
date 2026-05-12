#!/usr/bin/env bash
# replay_noetic_bag.sh — laptop-side automated replay of an onboard Noetic
# record bag (the kind onboard_record_noetic.sh produces — ROS 1 .bag).
#
# Bring up in one command:
#   1. Convert .bag → ROS 2 mcap (cached; skipped if already done)
#   2. octomap_server_node                              (background)
#   3. rviz2 with a baseline Noetic-replay cfg          (background)
#   4. ros2 bag play <mcap> --clock                      (foreground)
#
# Ctrl+C → kills (2)+(3)+(4), exits cleanly.
#
# This script is sister to scripts/real/replay_bag.sh — which targets the
# Foxy real-robot rosbag2 dirs. Don't merge: input format and topic naming
# diverge.
#
# Usage:
#   ./replay_noetic_bag.sh                         # latest first_run, octomap on
#   ./replay_noetic_bag.sh tag=run2
#   ./replay_noetic_bag.sh tag=first_run octomap=false   # raw cloud + odom only
#   ./replay_noetic_bag.sh tag=first_run rate=0.5        # 0.5x replay
#   ./replay_noetic_bag.sh tag=first_run loop=true       # rosbag2 --loop
#   ./replay_noetic_bag.sh bag=/path/to/file.bag         # explicit path
#
# Requires (laptop):
#   ros-humble-octomap-server ros-humble-octomap-rviz-plugins
#   pip install --user rosbags

set -eo pipefail
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
REPO_ROOT="$( cd "$SCRIPT_DIR/../.." &> /dev/null && pwd )"

LOCAL_BAG_DIR="${LOCAL_BAG_DIR:-$HOME/Collab_QRC/bags}"
TAG="first_run"
BAG_EXPLICIT=""
RATE="1.0"
LOOP="false"
WITH_OCTOMAP="true"
RVIZ_CFG="${SCRIPT_DIR}/rviz2_replay_noetic.rviz"
OCTOMAP_RES="0.10"
OCTOMAP_MAXRANGE="8.0"
OCTOMAP_FRAME="camera_init"
OCTOMAP_CLOUD_IN="/robot/cloud_registered_body"

for arg in "$@"; do
  case "$arg" in
    tag=*)        TAG="${arg#tag=}" ;;
    bag=*)        BAG_EXPLICIT="${arg#bag=}" ;;
    rate=*)       RATE="${arg#rate=}" ;;
    loop=*)       LOOP="${arg#loop=}" ;;
    octomap=*)    WITH_OCTOMAP="${arg#octomap=}" ;;
    resolution=*) OCTOMAP_RES="${arg#resolution=}" ;;
    max_range=*)  OCTOMAP_MAXRANGE="${arg#max_range=}" ;;
    cloud_in=*)   OCTOMAP_CLOUD_IN="${arg#cloud_in=}" ;;
    *) echo "WARN: unknown arg '$arg'" >&2 ;;
  esac
done

# ── Resolve .bag path ────────────────────────────────────────────────
if [[ -n "$BAG_EXPLICIT" ]]; then
  BAG="$BAG_EXPLICIT"
  [[ -f "$BAG" ]] || { echo "ERROR: $BAG not found" >&2; exit 1; }
else
  BAG=$(ls -t "${LOCAL_BAG_DIR}"/onboard_noetic_*${TAG}*_0_0.bag 2>/dev/null | head -1)
  if [[ -z "$BAG" ]]; then
    BAG=$(ls -t "${LOCAL_BAG_DIR}"/onboard_noetic_*${TAG}*.bag 2>/dev/null \
           | grep -v '\.orig\.bag$' | head -1)
  fi
  [[ -n "$BAG" ]] || { echo "ERROR: no bag matching '${TAG}' in $LOCAL_BAG_DIR" >&2; exit 1; }
fi
echo "  bag: $BAG ($(du -h "$BAG" | cut -f1))"

# ── Convert .bag → ROS 2 mcap (skip if cached) ───────────────────────
BASE="${BAG%_0_0.bag}"
[[ "$BASE" == "$BAG" ]] && BASE="${BAG%.bag}"
MCAP_DIR="${BASE}_ros2"

if [[ ! -d "$MCAP_DIR" ]]; then
  command -v rosbags-convert &>/dev/null \
    || { echo "ERROR: rosbags-convert missing. Run: pip install --user rosbags" >&2; exit 1; }
  CHUNKS=( ${BASE}_0_*.bag )
  [[ ${#CHUNKS[@]} -eq 0 ]] && CHUNKS=( "$BAG" )
  echo "  converting ${#CHUNKS[@]} chunk(s) → $MCAP_DIR ..."
  rosbags-convert --src "${CHUNKS[@]}" --dst "$MCAP_DIR" 2>&1 | tail -3

  # Post-process metadata.yaml: rosbags 0.11.x emits offered_qos_profiles
  # as a nested YAML LIST, but ROS 2 Humble's rosbag2 reader expects a
  # serialized YAML STRING.  Empty string = use default QoS.
  echo "  patching metadata.yaml (humble QoS schema)..."
  python3 - "$MCAP_DIR/metadata.yaml" <<'PYEOF'
import sys, yaml
p = sys.argv[1]
d = yaml.safe_load(open(p))
for t in d['rosbag2_bagfile_information']['topics_with_message_count']:
    t['topic_metadata']['offered_qos_profiles'] = ""
open(p, 'w').write(yaml.safe_dump(d, sort_keys=False))
print(f"    patched {len(d['rosbag2_bagfile_information']['topics_with_message_count'])} topics")
PYEOF
else
  echo "  reusing cached $MCAP_DIR"
fi

# ── Cleanup trap ─────────────────────────────────────────────────────
BAG_PID=""; OCTO_PID=""; RVIZ_PID=""
cleanup() {
  trap - INT TERM EXIT
  echo ""
  echo "→ stopping replay session..."
  for pid in $RVIZ_PID $OCTO_PID $BAG_PID; do
    [[ -n "$pid" ]] && kill -INT "$pid" 2>/dev/null || true
  done
  sleep 1
  for pid in $RVIZ_PID $OCTO_PID $BAG_PID; do
    [[ -n "$pid" ]] && kill -9 "$pid" 2>/dev/null || true
  done
  # Belt-and-suspenders: any orphaned bag play with our mcap as arg.
  pkill -9 -f "ros2 bag play $MCAP_DIR" 2>/dev/null || true
  echo "  ✓ done"
  exit 0
}
trap cleanup INT TERM EXIT

source /opt/ros/humble/setup.bash
[[ -f "$REPO_ROOT/install/setup.bash" ]] && source "$REPO_ROOT/install/setup.bash"

# ── Generate a baseline rviz2 cfg if missing ─────────────────────────
# Minimal but useful: Grid + TF + body-frame cloud + odom + path + octomap 2D.
# Saved next to the script so the user can hand-edit and it persists.
if [[ ! -f "$RVIZ_CFG" ]]; then
  echo "  writing default rviz2 cfg → $RVIZ_CFG"
  # Minimal cfg: trajectory (Path) + voxel grid (MarkerArray) + reference Grid.
  # NO TF display, NO base_link dependency — everything lives in camera_init.
  # /occupied_cells_vis_array is octomap_server's pre-rendered cube markers,
  # no octomap_rviz_plugins/OccupancyGrid required.
  cat > "$RVIZ_CFG" <<'RVIZ_EOF'
Panels:
  - Class: rviz_common/Displays
    Name: Displays
Visualization Manager:
  Displays:
    - Class: rviz_default_plugins/Grid
      Name: Grid
      Enabled: true
      Plane Cell Count: 30
      Cell Size: 1.0
      Color: 100; 100; 100
      Alpha: 0.4
    - Class: rviz_default_plugins/Path
      Name: Trajectory
      Enabled: true
      Topic:
        Value: /robot/path
      Color: 255; 165; 0
      Line Style: Billboards
      Line Width: 0.20
      Pose Style: Axes
      Axes Length: 0.30
      Axes Radius: 0.04
    - Class: rviz_default_plugins/Odometry
      Name: OdomTrail
      Enabled: true
      Topic:
        Value: /robot/Odometry
      Shape:
        Color: 255; 50; 50
        Length: 0.40
        Radius: 0.05
      Keep: 500
      Position Tolerance: 0.001
      Angle Tolerance: 0.001
    - Class: rviz_default_plugins/MarkerArray
      Name: OctoMap-Voxels
      Enabled: true
      Topic:
        Value: /occupied_cells_vis_array
  Global Options:
    Background Color: 32; 32; 32
    Fixed Frame: camera_init
    Frame Rate: 20
  Tools:
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/FocusCamera
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 20.0
      Focal Point: {X: 0.0, Y: 0.0, Z: 0.0}
      Pitch: 0.8
      Yaw: 0.6
      Target Frame: <Fixed Frame>
RVIZ_EOF
fi

# ── Banner ──────────────────────────────────────────────────────────
echo ""
echo "################################################"
echo "  REPLAY  $(basename "$BAG")"
echo "    mcap   : $MCAP_DIR"
echo "    rate   : ${RATE}x  loop=${LOOP}  octomap=${WITH_OCTOMAP}"
[[ "$WITH_OCTOMAP" == "true" ]] && \
  echo "    octo   : res=${OCTOMAP_RES} max_range=${OCTOMAP_MAXRANGE} cloud_in=${OCTOMAP_CLOUD_IN}"
echo "  Ctrl+C → kills bag_play + octomap + rviz2 cleanly."
echo "################################################"
echo ""

# ── 1. octomap_server (start FIRST so it doesn't miss early frames) ─
if [[ "$WITH_OCTOMAP" == "true" ]]; then
  echo "[1/3] starting octomap_server..."
  ros2 run octomap_server octomap_server_node \
    --ros-args \
    -p frame_id:="$OCTOMAP_FRAME" \
    -p base_frame_id:=base_link \
    -p resolution:="$OCTOMAP_RES" \
    -p sensor_model.max_range:="$OCTOMAP_MAXRANGE" \
    -p sensor_model.hit:=0.65 \
    -p sensor_model.miss:=0.30 \
    -p occupancy_min_z:=0.10 \
    -p occupancy_max_z:=2.50 \
    -p filter_ground_plane:=false \
    -p incremental_2D_projection:=true \
    -p publish_free_space:=true \
    -p latch:=false \
    -p use_sim_time:=true \
    -r cloud_in:="$OCTOMAP_CLOUD_IN" \
    > /tmp/replay_octomap.log 2>&1 &
  OCTO_PID=$!
  echo "      PID=$OCTO_PID  log: /tmp/replay_octomap.log"
  sleep 2
fi

# ── 2. rviz2 ────────────────────────────────────────────────────────
echo "[2/3] starting rviz2..."
rviz2 -d "$RVIZ_CFG" --ros-args -p use_sim_time:=true \
  > /tmp/replay_rviz.log 2>&1 &
RVIZ_PID=$!
echo "      PID=$RVIZ_PID  log: /tmp/replay_rviz.log"
sleep 2

# ── 3. ros2 bag play (foreground so Ctrl+C lands here) ──────────────
echo "[3/3] playing bag (rate=${RATE}x)..."
echo ""
echo "  Published topics during replay:"
echo "    /robot/Odometry, /robot/cloud_registered{,_body}, /robot/path,"
[[ "$WITH_OCTOMAP" == "true" ]] && \
echo "    /projected_map (octomap 2D), /occupied_cells_vis_array (octomap 3D)"
echo "  See rviz2 window."
echo ""
# Exclude /livox/lidar: it uses livox_ros_driver2/msg/CustomMsg which has
# no ROS 2 type definition installed on this laptop. Without --exclude
# ros2 bag play prints a warning and may skip the topic.  Octomap and
# trajectory don't need it — /robot/cloud_registered_body covers viz.
PLAY_ARGS=(--clock --rate "$RATE" --topics
           /robot/cloud_registered_body /robot/cloud_registered
           /robot/Odometry /robot/path /tf /tf_static /livox/imu)
[[ "$LOOP" == "true" ]] && PLAY_ARGS+=(--loop)
ros2 bag play "$MCAP_DIR" "${PLAY_ARGS[@]}" &
BAG_PID=$!
echo "      PID=$BAG_PID"

# Wait for playback to finish; trap handles cleanup on Ctrl+C.
wait $BAG_PID 2>/dev/null || true
echo ""
echo "  bag playback finished. rviz2/octomap still alive — Ctrl+C to exit."
while kill -0 $RVIZ_PID 2>/dev/null; do sleep 1; done
