#!/usr/bin/env bash
# offline_slam.sh — re-run a SLAM algorithm on raw Livox + IMU data from an
# onboard Noetic .bag.  Unlike replay_noetic_bag.sh (which plays pre-computed
# SLAM outputs), this feeds /livox/lidar + /livox/imu into a SLAM node running
# on the laptop so you can compare algorithms (Fast-LIO2, Point-LIO, etc.)
#
# Usage:
#   ./offline_slam.sh                           # latest ops1 bag, Fast-LIO
#   ./offline_slam.sh tag=ops2
#   ./offline_slam.sh tag=ops1 slam=fast_lio rate=0.5
#   ./offline_slam.sh bag=/path/to/file.bag slam=fast_lio
#
# Supported slam= values:
#   fast_lio   — ROS 2 Fast-LIO2 (already in install/)
#   point_lio  — TODO: add when point_lio ROS 2 pkg is built
#
# Prerequisites (one-time build):
#   colcon build --symlink-install --packages-select livox_ros_driver2
#   colcon build --symlink-install --packages-select fast_lio
#
# Rate advice: use 0.5 for first runs; Fast-LIO is CPU-bound and can fall
# behind at 1.0x on a laptop — you'll see "[ WARN] IMU buffer full" if so.

set -eo pipefail
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
REPO_ROOT="$( cd "$SCRIPT_DIR/../.." &> /dev/null && pwd )"

LOCAL_BAG_DIR="${LOCAL_BAG_DIR:-$REPO_ROOT/bags}"
TAG="ops1"
BAG_EXPLICIT=""
RATE="0.5"
SLAM="fast_lio"
RVIZ="true"
PCD_SAVE="true"

for arg in "$@"; do
  case "$arg" in
    tag=*)    TAG="${arg#tag=}" ;;
    bag=*)    BAG_EXPLICIT="${arg#bag=}" ;;
    rate=*)   RATE="${arg#rate=}" ;;
    slam=*)   SLAM="${arg#slam=}" ;;
    rviz=*)   RVIZ="${arg#rviz=}" ;;
    pcd=*)    PCD_SAVE="${arg#pcd=}" ;;
    *) echo "WARN: unknown arg '$arg'" >&2 ;;
  esac
done

# ── Preflight: check livox_ros_driver2 is installed ──────────────────
if [[ ! -d "$REPO_ROOT/install/livox_ros_driver2" ]]; then
  echo "ERROR: livox_ros_driver2 not installed. Run:" >&2
  echo "  cd $REPO_ROOT" >&2
  echo "  colcon build --symlink-install --packages-select livox_ros_driver2" >&2
  exit 1
fi

# ── Preflight: check SLAM backend ────────────────────────────────────
case "$SLAM" in
  fast_lio)
    SLAM_BIN="$REPO_ROOT/install/fast_lio/lib/fast_lio/fastlio_mapping"
    SLAM_PKG="fast_lio"
    SLAM_EXE="fastlio_mapping"
    SLAM_CFG="$REPO_ROOT/install/fast_lio/share/fast_lio/config/mid360.yaml"
    # Real-robot tuned overrides (see docs/claude/noetic_fastlio_onboard.md)
    SLAM_EXTRA_PARAMS=(
      "point_filter_num:=1"
      "filter_size_surf:=0.10"
      "filter_size_map:=0.10"
      "preprocess.blind:=0.20"
      "mapping.extrinsic_est_en:=true"
      "pcd_save.pcd_save_en:=$PCD_SAVE"
      "pcd_save.interval:=20"
      "common.time_sync_en:=true"
    )
    ;;
  point_lio)
    echo "ERROR: point_lio ROS 2 package not yet built in this workspace." >&2
    echo "  When available, add its launch config to this case block." >&2
    exit 1
    ;;
  *)
    echo "ERROR: unknown slam='$SLAM'. Valid: fast_lio, point_lio" >&2
    exit 1
    ;;
esac

if [[ ! -f "$SLAM_BIN" ]]; then
  echo "ERROR: $SLAM binary not found at $SLAM_BIN" >&2
  echo "  Run: colcon build --symlink-install --packages-select $SLAM_PKG" >&2
  exit 1
fi

# ── Resolve .bag path ─────────────────────────────────────────────────
if [[ -n "$BAG_EXPLICIT" ]]; then
  BAG="$BAG_EXPLICIT"
  [[ -f "$BAG" ]] || { echo "ERROR: $BAG not found" >&2; exit 1; }
else
  BAG=$(ls -t "${LOCAL_BAG_DIR}"/onboard_noetic_*${TAG}*_0_0.bag 2>/dev/null | head -1)
  [[ -z "$BAG" ]] && BAG=$(ls -t "${LOCAL_BAG_DIR}"/onboard_noetic_*${TAG}*.bag 2>/dev/null \
                           | grep -v '\.orig\.bag$' | head -1)
  [[ -n "$BAG" ]] || { echo "ERROR: no bag matching '${TAG}' in $LOCAL_BAG_DIR" >&2; exit 1; }
fi
echo "  bag : $BAG ($(du -h "$BAG" | cut -f1))"

# ── Convert .bag → ROS 2 mcap (separate cache from replay_noetic_bag cache)
# includes /livox/lidar so the SLAM node gets raw CustomMsg
BASE="${BAG%_0_0.bag}"
[[ "$BASE" == "$BAG" ]] && BASE="${BAG%.bag}"
MCAP_DIR="${BASE}_ros2_raw"   # different suffix from replay_noetic_bag.sh's _ros2

if [[ ! -d "$MCAP_DIR" ]]; then
  command -v rosbags-convert &>/dev/null \
    || { echo "ERROR: rosbags-convert missing. Run: pip install --user rosbags" >&2; exit 1; }

  CHUNKS=( ${BASE}_0_*.bag )
  [[ ${#CHUNKS[@]} -eq 0 ]] && CHUNKS=( "$BAG" )
  echo "  converting ${#CHUNKS[@]} chunk(s) → $MCAP_DIR  (includes /livox/lidar) ..."
  rosbags-convert --src "${CHUNKS[@]}" --dst "$MCAP_DIR" 2>&1 | tail -3

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

# ── Cleanup trap ──────────────────────────────────────────────────────
SLAM_PID=""; RVIZ_PID=""
cleanup() {
  trap - INT TERM EXIT
  echo ""
  echo "→ stopping offline SLAM session..."
  [[ -n "$RVIZ_PID" ]] && kill -INT "$RVIZ_PID" 2>/dev/null || true
  # Send SIGINT to SLAM and wait up to 15 s for it to flush PCD before SIGKILL
  if [[ -n "$SLAM_PID" ]]; then
    kill -INT "$SLAM_PID" 2>/dev/null || true
    echo "  waiting for SLAM to flush PCD (up to 15 s)..."
    for i in $(seq 1 15); do
      kill -0 "$SLAM_PID" 2>/dev/null || break
      sleep 1
    done
    kill -9 "$SLAM_PID" 2>/dev/null || true
  fi
  [[ -n "$RVIZ_PID" ]] && kill -9 "$RVIZ_PID" 2>/dev/null || true
  echo "  done"
  exit 0
}
trap cleanup INT TERM EXIT

source /opt/ros/humble/setup.bash
source "$REPO_ROOT/install/setup.bash"

# ── Banner ────────────────────────────────────────────────────────────
echo ""
echo "################################################"
echo "  OFFLINE SLAM  $(basename "$BAG")"
echo "    slam   : $SLAM   rate=${RATE}x   pcd_save=${PCD_SAVE}"
echo "    mcap   : $MCAP_DIR"
echo "  Raw topics fed to SLAM: /livox/lidar  /livox/imu  /tf_static"
echo "  SLAM output:  /Odometry  /cloud_registered{,_body}  /path"
echo "  Ctrl+C → kills SLAM + rviz cleanly."
echo "################################################"
echo ""

# ── 1. SLAM node ──────────────────────────────────────────────────────
echo "[1/2] starting $SLAM ..."
EXTRA_ARGS=()
for p in "${SLAM_EXTRA_PARAMS[@]}"; do
  EXTRA_ARGS+=(-p "$p")
done
ros2 run "$SLAM_PKG" "$SLAM_EXE" \
  --ros-args \
  --params-file "$SLAM_CFG" \
  "${EXTRA_ARGS[@]}" \
  > /tmp/offline_slam_${SLAM}.log 2>&1 &
SLAM_PID=$!
echo "      PID=$SLAM_PID  log: /tmp/offline_slam_${SLAM}.log"
sleep 2

# ── 2. RViz (optional) ───────────────────────────────────────────────
if [[ "$RVIZ" == "true" ]]; then
  RVIZ_CFG="$REPO_ROOT/install/fast_lio/share/fast_lio/rviz/fastlio.rviz"
  [[ -f "$RVIZ_CFG" ]] || RVIZ_CFG=""
  echo "[2/2] starting rviz2..."
  rviz2 ${RVIZ_CFG:+-d "$RVIZ_CFG"} --ros-args -p use_sim_time:=true \
    > /tmp/offline_slam_rviz.log 2>&1 &
  RVIZ_PID=$!
  echo "      PID=$RVIZ_PID"
  sleep 2
fi

# ── 3. bag play: raw sensors only (no pre-computed SLAM outputs) ──────
echo "[3/3] playing raw sensor topics at ${RATE}x ..."
echo "  (skipping /robot/Odometry, /robot/cloud_registered*, /robot/path — SLAM recomputes these)"
echo ""

ros2 bag play "$MCAP_DIR" \
  --rate "$RATE" \
  --topics /livox/lidar /livox/imu /tf_static &
BAG_PID=$!
echo "      bag PID=$BAG_PID"

wait $BAG_PID 2>/dev/null || true
echo ""
echo "  bag playback done — sending SIGINT to SLAM to flush PCD..."

# Send SIGINT to Fast-LIO (triggers its shutdown save callback) and wait up to 20 s
if [[ -n "$SLAM_PID" ]] && kill -0 "$SLAM_PID" 2>/dev/null; then
  kill -INT "$SLAM_PID" 2>/dev/null || true
  for i in $(seq 1 20); do
    kill -0 "$SLAM_PID" 2>/dev/null || { echo "  SLAM exited after ${i}s"; break; }
    sleep 1
  done
  kill -9 "$SLAM_PID" 2>/dev/null || true
fi

# ── 4. Post-SLAM: point cloud → mesh ─────────────────────────────────
FAST_LIO_SRC="$REPO_ROOT/src/vendor/fast_lio"
PCD_PATH="$FAST_LIO_SRC/PCD/scans.pcd"
MESH_SCRIPT="$REPO_ROOT/scripts/real/pcd_to_mesh.py"
OUT_STEM="${BASE##*/}"   # basename of bag (no extension)
MESH_OUT_DIR="$REPO_ROOT/bags/meshes/${OUT_STEM}_mesh"

echo ""
echo "################################################"
# Merge interval PCDs (scans_*.pcd) into a single scans.pcd if needed
if [[ ! -f "$PCD_PATH" ]] && ls "$FAST_LIO_SRC"/PCD/scans_*.pcd >/dev/null 2>&1; then
  N=$(ls "$FAST_LIO_SRC"/PCD/scans_*.pcd | wc -l)
  echo "  merging $N interval PCDs into scans.pcd..."
  python3 - <<PYEOF
import open3d as o3d, glob
from pathlib import Path
files = sorted(glob.glob("$FAST_LIO_SRC/PCD/scans_*.pcd"),
               key=lambda p: int(Path(p).stem.split("_")[1]))
merged = o3d.geometry.PointCloud()
for f in files:
    merged += o3d.io.read_point_cloud(f)
o3d.io.write_point_cloud("$PCD_PATH", merged)
print(f"  merged {len(files)} files → {len(merged.points):,} points")
PYEOF
fi
echo "  PCD saved to: $PCD_PATH"
if [[ -f "$PCD_PATH" ]]; then
  SIZE=$(du -h "$PCD_PATH" | cut -f1)
  echo "  size: $SIZE"
  echo ""
  echo "  Running mesh conversion automatically..."
  python3 "$MESH_SCRIPT" "$PCD_PATH" --out-dir "$MESH_OUT_DIR" --nurec-collider
  echo ""
  echo "  To re-run mesh conversion manually:"
  echo "    python3 $MESH_SCRIPT $PCD_PATH --out-dir $MESH_OUT_DIR --nurec-collider"
else
  echo "  WARN: PCD file not found — pcd_save_en may be false or SLAM crashed before saving"
fi
echo "################################################"
