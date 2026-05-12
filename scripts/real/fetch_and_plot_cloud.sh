#!/usr/bin/env bash
# fetch_and_plot_cloud.sh — grab a snapshot of /robot/cloud_registered from
# the Jetson's running Noetic FAST-LIO2, pull it back to the laptop, and view
# it with Open3D — bypasses X11-forwarding entirely.
#
# Why this exists:  X11-forward of RViz over the USB-C dongle was rendering
# black-on-black (Mesa indirect GLX disabled on Jammy + SWrast slow enough to
# also fail in practice). Sometimes the right move is to stop fighting the
# display protocol and just ship the bytes.
#
# Flow:
#   1. SSH to Jetson; run a small rospy snippet that subscribes to the cloud
#      topic, captures N frames, saves them to /tmp/cloud_<stamp>.npz.
#   2. rsync the .npz back to the laptop.
#   3. Open Open3D GUI on the laptop to visualize.
#
# Usage:
#   ./fetch_and_plot_cloud.sh                          # 1 frame, /robot/cloud_registered
#   ./fetch_and_plot_cloud.sh frames=5                 # accumulate 5 frames into one cloud
#   ./fetch_and_plot_cloud.sh topic=/robot/cloud_registered_body
#   ./fetch_and_plot_cloud.sh out=/tmp/run3.npz
#   ./fetch_and_plot_cloud.sh frames=10 plot=false     # save only, no viewer
#
# Requires (laptop):  python3 + numpy + open3d
# Requires (Jetson):  /usr/bin/python3 (system, NOT miniconda) + rospy from Noetic

set -e
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

JETSON_USER="unitree"
JETSON_HOST="${GO2W_JETSON_IP:-192.168.123.18}"
JETSON_PASS="${JETSON_PASS:-}"
TOPIC="/robot/cloud_registered"
FRAMES="1"
OUT=""
DO_PLOT="true"

for arg in "$@"; do
  case "$arg" in
    user=*)   JETSON_USER="${arg#user=}" ;;
    host=*)   JETSON_HOST="${arg#host=}" ;;
    pass=*)   JETSON_PASS="${arg#pass=}" ;;
    topic=*)  TOPIC="${arg#topic=}" ;;
    frames=*) FRAMES="${arg#frames=}" ;;
    out=*)    OUT="${arg#out=}" ;;
    plot=*)   DO_PLOT="${arg#plot=}" ;;
    *) echo "WARN: unknown arg '$arg'" >&2 ;;
  esac
done

[[ -z "$JETSON_PASS" ]] && { echo "ERROR: JETSON_PASS not set." >&2; exit 1; }
command -v sshpass &>/dev/null || { echo "ERROR: sshpass not installed" >&2; exit 1; }

STAMP="$(date +%Y%m%d_%H%M%S)"
REMOTE_NPZ="/tmp/cloud_${STAMP}.npz"
LOCAL_NPZ="${OUT:-/tmp/cloud_${STAMP}.npz}"

# ── Jetson-side: subscribe + dump to .npz ────────────────────────────
# Inline python (heredoc) — avoids deploying a separate file.
# Uses /usr/bin/python3 explicitly because Jetson's interactive .bashrc puts
# miniconda's python3 first in PATH, which doesn't have rospy.
REMOTE_PY=$(cat <<'PYEOF'
import sys, os
# Force system python's ROS modules even if PATH is contaminated.
sys.path = [p for p in sys.path if 'miniconda' not in p and 'conda' not in p]
sys.path.insert(0, '/opt/ros/noetic/lib/python3/dist-packages')
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
import numpy as np

topic = sys.argv[1]
n_frames = int(sys.argv[2])
out_path = sys.argv[3]

os.environ.setdefault('ROS_MASTER_URI', 'http://192.168.123.18:11311')
os.environ.setdefault('ROS_IP', '192.168.123.18')

rospy.init_node('cloud_snap', anonymous=True, disable_signals=True)
frames = []

def cb(msg):
    pts = np.array(list(pc2.read_points(msg,
        field_names=('x','y','z','intensity'), skip_nans=True)),
        dtype=np.float32)
    frames.append(pts)
    rospy.loginfo(f"frame {len(frames)}/{n_frames}: {pts.shape}, frame_id={msg.header.frame_id}")
    if len(frames) >= n_frames:
        rospy.signal_shutdown('captured')

sub = rospy.Subscriber(topic, PointCloud2, cb, queue_size=1)
# Wait up to 10s for first message; rospy.spin won't block on shutdown.
t0 = rospy.Time.now()
while not rospy.is_shutdown():
    rospy.rostime.wallsleep(0.1)
    if (rospy.Time.now() - t0).to_sec() > 10 and not frames:
        rospy.logerr(f"no message on {topic} in 10s, giving up")
        sys.exit(2)

stacked = np.vstack(frames) if frames else np.zeros((0,4), dtype=np.float32)
np.savez_compressed(out_path, points=stacked, topic=topic, n_frames=len(frames))
print(f"SAVED {out_path}  shape={stacked.shape}  dtype={stacked.dtype}")
PYEOF
)

echo "[1/3] capturing $FRAMES frame(s) of $TOPIC on Jetson..."
SSH_OPTS=(-o StrictHostKeyChecking=accept-new -o ConnectTimeout=5)
# Pipe the Python source on stdin to `python3 -` — bash quoting can't mangle
# what we never put on the command line.  Args after `-` become sys.argv[1:].
sshpass -p "$JETSON_PASS" ssh "${SSH_OPTS[@]}" "${JETSON_USER}@${JETSON_HOST}" \
  "/usr/bin/python3 - '$TOPIC' '$FRAMES' '$REMOTE_NPZ'" <<EOF
$REMOTE_PY
EOF

# ── Laptop-side: rsync back ──────────────────────────────────────────
echo "[2/3] rsyncing $REMOTE_NPZ → $LOCAL_NPZ ..."
rsync -avz -e "sshpass -p $JETSON_PASS ssh ${SSH_OPTS[*]}" \
  "${JETSON_USER}@${JETSON_HOST}:${REMOTE_NPZ}" "$LOCAL_NPZ" 2>&1 | tail -3

echo "  Local file: $LOCAL_NPZ ($(du -h "$LOCAL_NPZ" | cut -f1))"

# ── Laptop-side: Open3D viewer ───────────────────────────────────────
if [[ "$DO_PLOT" != "true" ]]; then
  echo "  plot=false — file saved, viewer skipped."
  exit 0
fi

echo "[3/3] launching Open3D viewer..."
python3 - "$LOCAL_NPZ" <<'PYEOF'
import sys, numpy as np
import open3d as o3d

data = np.load(sys.argv[1])
pts = data['points']
xyz = pts[:, :3]
intensity = pts[:, 3]
print(f"loaded {xyz.shape[0]:,} points from topic {data['topic']}, "
      f"x range [{xyz[:,0].min():.2f}, {xyz[:,0].max():.2f}], "
      f"y [{xyz[:,1].min():.2f}, {xyz[:,1].max():.2f}], "
      f"z [{xyz[:,2].min():.2f}, {xyz[:,2].max():.2f}]")

# Color by intensity (rainbow). Normalize to 0..1 then map.
imin, imax = float(intensity.min()), float(intensity.max())
if imax > imin:
    inorm = (intensity - imin) / (imax - imin)
else:
    inorm = np.zeros_like(intensity)
# Manual rainbow: red → yellow → green → cyan → blue
r = np.clip(1.5 - 4 * np.abs(inorm - 0.75), 0, 1)
g = np.clip(1.5 - 4 * np.abs(inorm - 0.5),  0, 1)
b = np.clip(1.5 - 4 * np.abs(inorm - 0.25), 0, 1)
colors = np.stack([r, g, b], axis=-1)

pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(xyz.astype(np.float64))
pcd.colors = o3d.utility.Vector3dVector(colors.astype(np.float64))

# Add an axis at origin for orientation reference (1m frame).
frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0, origin=[0,0,0])

print("Open3D viewer: drag to orbit, scroll to zoom, hold Shift+drag to pan, Q or Esc to close.")
o3d.visualization.draw_geometries([pcd, frame],
    window_name=f"{data['topic']} — {xyz.shape[0]:,} pts",
    width=1200, height=800)
PYEOF
