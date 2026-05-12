#!/usr/bin/env bash
# stream_cloud_live.sh — live point cloud stream from Jetson ROS 1 Noetic to
# laptop Open3D, without X11 forwarding or ROS on the laptop side.
#
# Architecture:
#   Jetson:                                  Laptop:
#   ┌─────────────────────────┐              ┌────────────────────────────┐
#   │ rospy.Subscriber        │              │ Open3D Visualizer          │
#   │   /robot/cloud_registered│              │   poll_events()            │
#   │     ↓ pc2.read_points   │              │   update_geometry()        │
#   │   write framed binary   │ ssh stdin    │   read framed binary       │
#   │   to stdout             ├──pipe──────→ │   from stdin               │
#   └─────────────────────────┘              └────────────────────────────┘
#
# Framing: 4-byte uint32 N (# float32s) + N*4 bytes (XYZI flat).
# Why ssh pipe instead of TCP socket? Reverse forward needs careful firewall
# config + cleanup; ssh stdin is bulletproof and gets killed with the script.
#
# Usage:
#   ./stream_cloud_live.sh                        # /robot/cloud_registered
#   ./stream_cloud_live.sh topic=/robot/cloud_registered_body
#   ./stream_cloud_live.sh decimate=5             # plot every 5th point (faster)
#
# Stop:
#   Ctrl+C in this terminal — clean shutdown on both ends.
#   Or close the Open3D window (Q or X button).

set -e
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

JETSON_USER="unitree"
JETSON_HOST="${GO2W_JETSON_IP:-192.168.123.18}"
JETSON_PASS="${JETSON_PASS:-}"
TOPIC="/robot/cloud_registered"
DECIMATE="1"     # plot every Nth point — 1 = all, 5 = 1/5 etc.

for arg in "$@"; do
  case "$arg" in
    user=*)     JETSON_USER="${arg#user=}" ;;
    host=*)     JETSON_HOST="${arg#host=}" ;;
    pass=*)     JETSON_PASS="${arg#pass=}" ;;
    topic=*)    TOPIC="${arg#topic=}" ;;
    decimate=*) DECIMATE="${arg#decimate=}" ;;
    *) echo "WARN: unknown arg '$arg'" >&2 ;;
  esac
done

[[ -z "$JETSON_PASS" ]] && { echo "ERROR: JETSON_PASS not set." >&2; exit 1; }
command -v sshpass &>/dev/null || { echo "ERROR: sshpass not installed" >&2; exit 1; }

echo ""
echo "################################################"
echo "  LIVE point cloud stream  (no X11 forwarding)"
echo "    target   : ${JETSON_USER}@${JETSON_HOST}"
echo "    topic    : $TOPIC"
echo "    decimate : 1/$DECIMATE points kept"
echo "  Ctrl+C in this terminal to stop, or close the Open3D window."
echo "################################################"
echo ""

# ── Jetson-side Python: subscribe + frame to stdout ──────────────────
# Stdout is the binary stream; stderr is logging (kept readable on laptop).
read -r -d '' JET_PY <<'PYEOF' || true
import sys, os, struct
# Scrub miniconda from python's module path (interactive .bashrc on this Jetson
# auto-activates conda base, which doesn't have rospy).
sys.path = [p for p in sys.path if 'miniconda' not in p and 'conda' not in p]
sys.path.insert(0, '/opt/ros/noetic/lib/python3/dist-packages')

import rospy, numpy as np
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2

topic = sys.argv[1]
decimate = max(1, int(sys.argv[2]))

os.environ.setdefault('ROS_MASTER_URI', 'http://192.168.123.18:11311')
os.environ.setdefault('ROS_IP', '192.168.123.18')

# disable_signals=True lets SIGINT/SIGPIPE land directly on Python so the
# script exits when ssh tears the stdout pipe (laptop side closed).
rospy.init_node('cloud_stream', anonymous=True, disable_signals=True)

# Binary protocol: write 4-byte little-endian uint32 = N floats, then N*4 bytes.
out = sys.stdout.buffer
frame_count = [0]

def cb(msg):
    pts = np.array(list(pc2.read_points(msg,
        field_names=('x','y','z','intensity'), skip_nans=True)),
        dtype=np.float32)
    if decimate > 1:
        pts = pts[::decimate]
    flat = pts.reshape(-1)
    try:
        out.write(struct.pack('<I', flat.size))
        out.write(flat.tobytes())
        out.flush()
    except BrokenPipeError:
        rospy.signal_shutdown('laptop closed pipe')
        return
    frame_count[0] += 1
    if frame_count[0] % 10 == 0:
        print(f"[jetson] {frame_count[0]} frames, last={pts.shape[0]} pts",
              file=sys.stderr, flush=True)

sub = rospy.Subscriber(topic, PointCloud2, cb, queue_size=1)
print(f"[jetson] subscribed to {topic}, decimate=1/{decimate}",
      file=sys.stderr, flush=True)

while not rospy.is_shutdown():
    rospy.rostime.wallsleep(0.05)
PYEOF

# ── Laptop-side Python: read stream + Open3D live viewer ─────────────
read -r -d '' LAPTOP_PY <<'PYEOF' || true
import sys, struct, numpy as np
import open3d as o3d

stream = sys.stdin.buffer
print("[laptop] waiting for first frame…", file=sys.stderr, flush=True)

def read_frame():
    """Returns (N,4) float32 array, or None on EOF."""
    hdr = stream.read(4)
    if len(hdr) < 4:
        return None
    n = struct.unpack('<I', hdr)[0]
    buf = bytearray(n * 4)
    view = memoryview(buf)
    got = 0
    while got < len(buf):
        chunk = stream.readinto(view[got:])
        if not chunk:
            return None
        got += chunk
    arr = np.frombuffer(buf, dtype=np.float32).reshape(-1, 4)
    return arr

def colorize(intensity):
    imin, imax = float(intensity.min()), float(intensity.max())
    if imax <= imin:
        return np.zeros((intensity.size, 3))
    inorm = (intensity - imin) / (imax - imin)
    r = np.clip(1.5 - 4*np.abs(inorm - 0.75), 0, 1)
    g = np.clip(1.5 - 4*np.abs(inorm - 0.5),  0, 1)
    b = np.clip(1.5 - 4*np.abs(inorm - 0.25), 0, 1)
    return np.stack([r, g, b], axis=-1)

# Bootstrap: read the first frame BEFORE creating the window so the cloud
# isn't a 0-point geometry (Open3D centers camera badly on that).
first = None
while first is None:
    first = read_frame()
    if first is None:
        print("[laptop] EOF before first frame — Jetson side likely failed",
              file=sys.stderr, flush=True)
        sys.exit(1)
xyz = first[:, :3].astype(np.float64)

vis = o3d.visualization.Visualizer()
vis.create_window(window_name="LIVE /robot/cloud_registered  (Ctrl+C terminal or Q window to stop)",
                  width=1280, height=800)

pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(xyz)
pcd.colors = o3d.utility.Vector3dVector(colorize(first[:, 3]).astype(np.float64))
vis.add_geometry(pcd)

frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0, origin=[0,0,0])
vis.add_geometry(frame)

# Initial view: top-down-ish.
view = vis.get_view_control()
view.set_lookat([0, 0, 0])
view.set_front([1, 1, 1])
view.set_up([0, 0, 1])
view.set_zoom(0.35)

n_frames = 1
import time
t0 = time.time()
while True:
    arr = read_frame()
    if arr is None:
        print("[laptop] stream ended", file=sys.stderr, flush=True)
        break
    pcd.points = o3d.utility.Vector3dVector(arr[:, :3].astype(np.float64))
    pcd.colors = o3d.utility.Vector3dVector(colorize(arr[:, 3]).astype(np.float64))
    vis.update_geometry(pcd)
    if not vis.poll_events():
        print("[laptop] window closed by user", file=sys.stderr, flush=True)
        break
    vis.update_renderer()
    n_frames += 1
    if n_frames % 30 == 0:
        fps = n_frames / max(0.001, time.time() - t0)
        print(f"[laptop] {n_frames} frames, {fps:.1f} fps", file=sys.stderr, flush=True)

vis.destroy_window()
PYEOF

# ── Bridge: ssh stdin to Jetson, stdout to laptop python ─────────────
SSH_OPTS=(-o StrictHostKeyChecking=accept-new -o ConnectTimeout=5
          -o ServerAliveInterval=10 -o ServerAliveCountMax=3)
# Cleanup trap — kill ssh child on Ctrl+C so the Jetson python ALSO dies
# (it will get SIGPIPE on its stdout write and signal_shutdown).
trap 'echo ""; echo "shutting down…"; kill $SSH_PID 2>/dev/null || true; exit 0' INT TERM

# `set -o pipefail` would make the whole pipeline fail when Open3D window
# closes (Python exits → SIGPIPE upstream). Keep pipefail off for this line.
set +e
sshpass -p "$JETSON_PASS" ssh "${SSH_OPTS[@]}" "${JETSON_USER}@${JETSON_HOST}" \
    "/usr/bin/python3 - '$TOPIC' '$DECIMATE'" <<<"$JET_PY" \
  | python3 -c "$LAPTOP_PY"
EXIT_CODE=$?
echo "[host] pipeline exited with code $EXIT_CODE"
