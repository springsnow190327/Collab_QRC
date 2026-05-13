#!/usr/bin/env bash
# stream_map_live.sh — live 2-panel map viewer over ssh binary pipe.
#
# Wireless-stable: viewer process stays alive across WiFi drops.
# A named FIFO (/tmp/go2_map_stream.fifo) decouples the viewer from SSH:
#
#   Jetson (ROS 1 Noetic):                   Laptop:
#   ┌──────────────────────────┐             ┌───────────────────────────┐
#   │ onboard_projector.py     │             │ local_viewer.py           │
#   │   /robot/cloud_registered│  ssh pipe   │   Left : current scan     │
#   │   /robot/Odometry        ├──► FIFO ───▶│   Right: accumulated map  │
#   │   /pci_command_path      │             │   Path : planned waypoints│
#   └──────────────────────────┘             └───────────────────────────┘
#
#   When WiFi drops SSH exits → FIFO gets EOF → viewer shows "reconnecting…"
#   → shell loop retries SSH every 3 s → viewer resumes, map state preserved.
#
# Binary protocol (see onboard_projector.py for full spec):
#   'C' 0x43  Cloud XY (Z-filtered, decimated)   N*2 float32 pairs
#   'P' 0x50  Pose x,y,yaw                        3   float32
#   'T' 0x54  Path waypoints XY                   N*2 float32 pairs
#
# Usage:
#   ./stream_map_live.sh                        # defaults
#   ./stream_map_live.sh decimate=10            # fewer points (faster on WiFi)
#   ./stream_map_live.sh z_min=0.05 z_max=1.2  # custom Z filter
#   ./stream_map_live.sh topic=/robot/cloud_registered_body
#   ./stream_map_live.sh host=192.168.123.18 pass=123
#
# Stop: Ctrl+C or close the matplotlib window.

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

PROJECTOR="$SCRIPT_DIR/onboard_projector.py"
VIEWER="$SCRIPT_DIR/local_viewer.py"
FIFO="/tmp/go2_map_stream.fifo"

JETSON_USER="unitree"
JETSON_HOST="${GO2W_JETSON_IP:-192.168.123.18}"
JETSON_PASS="${JETSON_PASS:-}"
CLOUD_TOPIC="/robot/cloud_registered"
ODOM_TOPIC="/robot/Odometry"
DECIMATE="5"
Z_MIN="0.10"
Z_MAX="1.80"
RETRY_DELAY=3   # seconds between reconnect attempts

for arg in "$@"; do
  case "$arg" in
    user=*)    JETSON_USER="${arg#user=}" ;;
    host=*)    JETSON_HOST="${arg#host=}" ;;
    pass=*)    JETSON_PASS="${arg#pass=}" ;;
    topic=*)   CLOUD_TOPIC="${arg#topic=}" ;;
    odom=*)    ODOM_TOPIC="${arg#odom=}" ;;
    decimate=*)DECIMATE="${arg#decimate=}" ;;
    z_min=*)   Z_MIN="${arg#z_min=}" ;;
    z_max=*)   Z_MAX="${arg#z_max=}" ;;
    retry=*)   RETRY_DELAY="${arg#retry=}" ;;
    *) echo "WARN: unknown arg '$arg'" >&2 ;;
  esac
done

# ── Preflight checks ──────────────────────────────────────────────────────────
[[ -z "$JETSON_PASS" ]] && {
  echo "ERROR: JETSON_PASS not set." >&2
  echo "  Either: export JETSON_PASS=123  (or pass=123 on cmdline)" >&2
  exit 1
}
command -v sshpass &>/dev/null || {
  echo "ERROR: sshpass not installed (apt install sshpass)" >&2
  exit 1
}
[[ -f "$PROJECTOR" ]] || { echo "ERROR: $PROJECTOR not found." >&2; exit 1; }
[[ -f "$VIEWER"    ]] || { echo "ERROR: $VIEWER not found."    >&2; exit 1; }

echo ""
echo "################################################"
echo "  Live map viewer  (stream_map_live.sh)"
echo "    Jetson   : ${JETSON_USER}@${JETSON_HOST}"
echo "    cloud    : $CLOUD_TOPIC"
echo "    odom     : $ODOM_TOPIC"
echo "    decimate : 1/$DECIMATE   z=[${Z_MIN}, ${Z_MAX}] m"
echo "    fifo     : $FIFO"
echo "  Left : current scan + pose trail"
echo "  Right: accumulated map grid + planned path"
echo "  Ctrl+C or close window to stop."
echo "################################################"
echo ""

SSH_OPTS=(
  -o StrictHostKeyChecking=accept-new
  -o ConnectTimeout=8
  -o ServerAliveInterval=10
  -o ServerAliveCountMax=3
)

# ── Create FIFO ───────────────────────────────────────────────────────────────
[[ -p "$FIFO" ]] || mkfifo "$FIFO"

# ── Launch viewer (once) ──────────────────────────────────────────────────────
python3 "$VIEWER" --fifo "$FIFO" &
VIEWER_PID=$!

# ── Cleanup on exit ───────────────────────────────────────────────────────────
cleanup() {
  echo ""
  echo "[stream_map_live] shutting down…"
  kill "$VIEWER_PID" 2>/dev/null || true
  rm -f "$FIFO"
  exit 0
}
trap cleanup INT TERM EXIT

# ── Reconnect loop ────────────────────────────────────────────────────────────
ATTEMPT=0
while kill -0 "$VIEWER_PID" 2>/dev/null; do
  ATTEMPT=$((ATTEMPT + 1))
  echo "[stream_map_live] SSH attempt $ATTEMPT → ${JETSON_USER}@${JETSON_HOST}"

  # SSH stdout → FIFO.  This open() on FIFO blocks until viewer opens its end,
  # so on first iteration it waits for viewer startup (usually <1 s).
  # On viewer EOF-reopen the FIFO reader is already waiting so this proceeds.
  sshpass -p "$JETSON_PASS" ssh "${SSH_OPTS[@]}" \
      "${JETSON_USER}@${JETSON_HOST}" \
      "python3 - \
        cloud_topic=${CLOUD_TOPIC} \
        odom_topic=${ODOM_TOPIC} \
        decimate=${DECIMATE} \
        z_min=${Z_MIN} \
        z_max=${Z_MAX} \
        ros_master=http://${JETSON_HOST}:11311 \
        ros_ip=${JETSON_HOST}" \
      < "$PROJECTOR" \
      > "$FIFO" 2>/dev/null
  EC=$?

  # Viewer closed its end (user shut the window) — SIGPIPE/141 on next write
  if ! kill -0 "$VIEWER_PID" 2>/dev/null; then
    echo "[stream_map_live] viewer closed — done"
    break
  fi

  echo "[stream_map_live] SSH exited (code $EC) — retry in ${RETRY_DELAY}s…"
  sleep "$RETRY_DELAY"
done

echo "[stream_map_live] exited"
