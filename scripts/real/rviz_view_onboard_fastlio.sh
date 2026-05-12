#!/usr/bin/env bash
# rviz_view_onboard_fastlio.sh — view the Jetson's ROS 1 Noetic FAST-LIO2
# output on the laptop, via ssh -X (X11 forwarding over ethernet).
#
# Why X-forwarding rather than ros1_bridge → rviz2?
#   - Laptop is Jammy + Humble only; Noetic doesn't install cleanly on Jammy.
#   - ros1_bridge + rviz2 path requires translating PointCloud2 message defs
#     + maintaining a bridge running on Jetson; ssh -X is one command.
#   - Network cost is similar (RViz over X11 ≈ topic streams over DDS), but
#     X11 lets us reuse the upstream HKU loam_livox.rviz config unchanged.
#
# Trade-offs:
#   - RViz UI feels slightly laggy over ethernet (still usable for SLAM
#     inspection; not great for clicking through dense menus).
#   - X11 rendering happens on the laptop GPU; Jetson only ships drawing
#     commands. Heavy point cloud display can saturate the Mid-360 stream.
#
# Usage (on the LAPTOP):
#   ./rviz_view_onboard_fastlio.sh
#   ./rviz_view_onboard_fastlio.sh host=192.168.123.18 user=unitree
#   ./rviz_view_onboard_fastlio.sh cfg=/path/to/custom.rviz
#
# Ctrl+C closes RViz cleanly; Jetson SLAM keeps running.

set -e

JETSON_USER="unitree"
JETSON_HOST="${GO2W_JETSON_IP:-192.168.123.18}"
JETSON_PASS="${JETSON_PASS:-}"
# Default RViz config: HKU fast_lio's loam_livox.rviz (already on Jetson after
# noetic_fastlio_ws build — points to map=camera_init frame, shows point cloud
# + odom path).
RVIZ_CFG=""

for arg in "$@"; do
  case "$arg" in
    user=*)  JETSON_USER="${arg#user=}" ;;
    host=*)  JETSON_HOST="${arg#host=}" ;;
    pass=*)  JETSON_PASS="${arg#pass=}" ;;
    cfg=*)   RVIZ_CFG="${arg#cfg=}" ;;
    *) echo "WARN: unknown arg '$arg'" >&2 ;;
  esac
done

if [[ -z "$JETSON_PASS" ]]; then
  echo "ERROR: JETSON_PASS not set." >&2
  echo "  Either: export JETSON_PASS=<robot-password>" >&2
  echo "  Or:     pass=<password> $0 ..." >&2
  exit 1
fi

command -v sshpass &>/dev/null || { echo "ERROR: sshpass not installed (apt install sshpass)" >&2; exit 1; }

if ! ping -c 1 -W 2 "$JETSON_HOST" &>/dev/null; then
  echo "ERROR: Cannot reach $JETSON_HOST. Check Ethernet." >&2
  exit 1
fi

# X11 needs DISPLAY set + xhost permissive enough for remote.
if [[ -z "${DISPLAY:-}" ]]; then
  echo "ERROR: \$DISPLAY not set on laptop. Run from a graphical session." >&2
  exit 1
fi
# Allow X connections from Jetson; idempotent.
xhost +SI:localuser:"$(whoami)" &>/dev/null || true

echo ""
echo "################################################"
echo "  Remote RViz (Jetson → laptop X11)"
echo "    target  : ${JETSON_USER}@${JETSON_HOST}"
echo "    DISPLAY : $DISPLAY (laptop)"
echo "    cfg     : ${RVIZ_CFG:-<HKU fast_lio default>}"
echo "  RViz is launched on the Jetson; rendering happens on the laptop."
echo "  Ctrl+C here closes RViz only — Jetson SLAM stack keeps running."
echo "################################################"
echo ""

# -Y = trusted X11 forwarding. -X (untrusted) blocks many GL extensions
# under modern xorg-server (XSECURITY restrictions), making rviz/Ogre crash
# on Grid display or other GL2+ primitives. -Y bypasses that.
SSH_OPTS=(-Y -o StrictHostKeyChecking=accept-new -o ConnectTimeout=5
          -o ServerAliveInterval=30 -o ServerAliveCountMax=3
          -o ForwardX11Timeout=4w)

# Build remote command. The Jetson's onboard_fastlio_noetic.sh sources
# /opt/ros/noetic and the workspace devel/setup.bash already in its own
# shell — but this ssh is a fresh shell, so source everything explicitly.
REMOTE_CMD='
# Strip miniconda from PATH (auto-activated by .bashrc — would shadow rviz).
if [[ -n "${CONDA_PREFIX:-}" ]] || echo "$PATH" | grep -q miniconda; then
  type conda 2>/dev/null | head -1 | grep -q function && conda deactivate 2>/dev/null || true
  export PATH="$(echo "$PATH" | tr ":" "\n" | grep -vE "(miniconda|conda)" | tr "\n" ":" | sed "s/:$//")"
  unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PROMPT_MODIFIER CONDA_SHLVL CONDA_PYTHON_EXE
fi
unset PYTHONPATH PYTHONHOME

source /opt/ros/noetic/setup.bash
source ~/noetic_fastlio_ws/devel/setup.bash
export ROS_MASTER_URI=http://192.168.123.18:11311
export ROS_IP=192.168.123.18

# Force Mesa software rasterization. Why not LIBGL_ALWAYS_INDIRECT=1 (the
# textbook X11-forwarding fix)? Because modern Ubuntu (incl. Jammy on the
# laptop side) ships with indirect GLX DISABLED by default — RViz then
# renders to a frame that the X server silently drops, producing a black
# viewport with FPS counter > 0 (no crash, no error). LIBGL_ALWAYS_SOFTWARE=1
# bypasses this: Mesa swrast renders into a CPU buffer on the Jetson, then
# XPutImage ships finished pixels to the laptop. Slow (5-15 fps for our
# pointcloud) but works on ANY X server config.
# If this is too sluggish, the right answer is NoMachine instead of X11-fwd.
export LIBGL_ALWAYS_SOFTWARE=1
# Belt-and-suspenders for Ogre RTT path on swrast.
export OGRE_RTT_MODE=Copy

# Pick config: explicit override, or HKU upstream default.
RVIZ_CFG="'"$RVIZ_CFG"'"
[[ -z "$RVIZ_CFG" ]] && RVIZ_CFG="$(rospack find fast_lio)/rviz_cfg/loam_livox.rviz"
echo "  Using RViz cfg: $RVIZ_CFG"
echo "  LIBGL_ALWAYS_SOFTWARE=1 (Mesa swrast — pixels XPutImage'd to laptop)"

# Launch RViz; -d loads the config. Foreground — closes when user exits.
rviz -d "$RVIZ_CFG"
'

exec sshpass -p "$JETSON_PASS" ssh "${SSH_OPTS[@]}" "${JETSON_USER}@${JETSON_HOST}" "$REMOTE_CMD"
