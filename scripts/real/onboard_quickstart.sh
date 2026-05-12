#!/usr/bin/env bash
# onboard_quickstart.sh — one-button laptop launcher for the Jetson's Noetic
# FAST-LIO2 stack.  Brings up SLAM remotely, prints live health stats on the
# laptop terminal, and — critically — when you Ctrl+C here, it SHUTS DOWN
# the remote stack cleanly.  No more orphan rosbag / fastlio processes
# surviving an ssh drop or terminal close.
#
# Flow:
#   1. Ping Jetson, confirm link.
#   2. SSH in: ensure 192.168.123.100/24 is on eth0 (Mid-360 bind — lost on
#      reboot; pre-cache sudo with hardcoded factory password).
#   3. Launch onboard_fastlio_noetic.sh on the Jetson (nohup, detached — so
#      if our ssh drops mid-session, SLAM keeps running).
#   4. Wait until /robot/Odometry has a publisher.
#   5. Status loop: every 4s ssh in, sample Odometry+IMU hz, print one line.
#   6. On Ctrl+C / TERM / EXIT: ssh in and run
#         onboard_fastlio_noetic.sh stop
#      Retry 3x on flaky ssh.  If all 3 fail, print the manual stop command.
#
# Usage:
#   ./onboard_quickstart.sh                       # default 192.168.123.18, pass 123
#   ./onboard_quickstart.sh host=192.168.123.18 user=unitree pass=123
#   ./onboard_quickstart.sh attach                # just attach to existing stack
#
# Sister: pull_and_view_bag.sh runs after this session ends.

set -eo pipefail

JETSON_USER="unitree"
JETSON_HOST="${GO2W_JETSON_IP:-192.168.123.18}"
JETSON_PASS="${JETSON_PASS:-123}"
LIVOX_HOST_IP="192.168.123.100"
ATTACH_ONLY="false"

for arg in "$@"; do
  case "$arg" in
    user=*)  JETSON_USER="${arg#user=}" ;;
    host=*)  JETSON_HOST="${arg#host=}" ;;
    pass=*)  JETSON_PASS="${arg#pass=}" ;;
    attach)  ATTACH_ONLY="true" ;;
    *) echo "WARN: unknown arg '$arg'" >&2 ;;
  esac
done

command -v sshpass &>/dev/null || { echo "ERROR: sshpass not installed (apt install sshpass)" >&2; exit 1; }

SSH_OPTS=(-o StrictHostKeyChecking=accept-new -o ConnectTimeout=5
          -o ServerAliveInterval=15 -o ServerAliveCountMax=3)
SSH="sshpass -p $JETSON_PASS ssh ${SSH_OPTS[*]} ${JETSON_USER}@${JETSON_HOST}"

# ── Shutdown trap — fires on Ctrl+C, SIGTERM, or normal exit ─────────
# Idempotent: SHUTDOWN_DONE guard so EXIT after INT doesn't double-shutdown.
SHUTDOWN_DONE=0
shutdown_remote() {
  [[ $SHUTDOWN_DONE -eq 1 ]] && return
  SHUTDOWN_DONE=1
  echo ""
  echo "→ Stopping SLAM stack on Jetson..."
  local attempt
  for attempt in 1 2 3; do
    if sshpass -p "$JETSON_PASS" ssh "${SSH_OPTS[@]}" \
        "${JETSON_USER}@${JETSON_HOST}" \
        '~/noetic_fastlio_ws/scripts/onboard_fastlio_noetic.sh stop' \
        2>&1 | tail -5; then
      echo "  ✓ Remote stack stopped."
      return 0
    fi
    echo "  retry ${attempt}/3 (ssh failed) ..."
    sleep 2
  done
  echo "  ✗ Could not reach Jetson to shut down. Run manually:"
  echo "      ssh ${JETSON_USER}@${JETSON_HOST} \\"
  echo "        '~/noetic_fastlio_ws/scripts/onboard_fastlio_noetic.sh stop'"
}
trap shutdown_remote INT TERM EXIT

# ── 1. Reachability ──────────────────────────────────────────────────
echo ""
echo "################################################"
echo "  Onboard FAST-LIO2 quickstart"
echo "    target : ${JETSON_USER}@${JETSON_HOST}"
echo "    mode   : $([[ "$ATTACH_ONLY" == "true" ]] && echo attach || echo "fresh start")"
echo "  Ctrl+C here → stops Jetson stack cleanly."
echo "################################################"
echo ""
echo "[1/4] ping Jetson..."
if ! ping -c 2 -W 2 "$JETSON_HOST" &>/dev/null; then
  echo "  ✗ ${JETSON_HOST} unreachable. Check USB-C dongle (often goes" >&2
  echo "    NO_CARRIER on this rig — replug, or reboot laptop)." >&2
  SHUTDOWN_DONE=1   # nothing to stop
  exit 1
fi
echo "  ✓ link OK"

# ── 2. Start (unless attach) ─────────────────────────────────────────
if [[ "$ATTACH_ONLY" != "true" ]]; then
  echo "[2/4] preflight cleanup + NIC bind + launch stack..."
  # Preflight cleanup is delegated to the launcher's own _kill_noetic_stack.
  # WHY:  the entire heredoc body becomes the remote bash's argv, so any
  # `pkill -9 -f <pattern>` whose pattern matches a string LITERALLY PRESENT
  # in this heredoc (e.g. "onboard_fastlio_noetic.sh", "roslaunch fast_lio",
  # "fastlio_mapping") kills the bash running the heredoc.  Self-fragging.
  # The launcher script doesn't have this issue because its argv doesn't
  # contain those patterns (they're literals INSIDE the script file, not
  # in argv) — so we delegate cleanup entirely to it.
  $SSH "
    # Mid-360 secondary IP (idempotent).
    if ! ip -4 addr show eth0 | grep -q ' ${LIVOX_HOST_IP}/'; then
      echo '${JETSON_PASS}' | sudo -S ip addr add ${LIVOX_HOST_IP}/24 dev eth0 \\
        2>&1 | sed 's/^/    /'
    else
      echo '    NIC bind already in place'
    fi

    # Kick the launcher detached. setsid -f forks into a new session,
    # nohup catches SIGHUP, </dev/null + redirected stdout fully detaches.
    # The launcher's startup runs _kill_noetic_stack itself, so any
    # previous instance's nodes get cleaned up before fresh ones boot.
    setsid -f nohup ~/noetic_fastlio_ws/scripts/onboard_fastlio_noetic.sh \\
      </dev/null >/tmp/onboard_launch.log 2>&1
    echo '    launcher kicked (preflight cleanup inside launcher itself)'
  "
else
  echo "[2/4] attach mode — skipping launch"
fi

# ── 3. Wait until SLAM publishes ─────────────────────────────────────
echo "[3/4] waiting for /robot/Odometry publisher..."
READY=0
for i in $(seq 1 30); do
  if $SSH "source /opt/ros/noetic/setup.bash; \
           export ROS_MASTER_URI=http://${JETSON_HOST}:11311; \
           rostopic info /robot/Odometry 2>/dev/null | grep -q '^ \* /robot/'" \
       &>/dev/null; then
    READY=1
    break
  fi
  printf "\r  ...%2ds" "$i"
  sleep 1
done
echo ""
if [[ "$READY" -ne 1 ]]; then
  echo "  ✗ /robot/Odometry never came up. Recent launch log:" >&2
  $SSH 'tail -30 /tmp/onboard_launch.log' 2>/dev/null | sed 's/^/    /'
  exit 1
fi
echo "  ✓ SLAM publishing"

# ── 4. Status loop ───────────────────────────────────────────────────
echo "[4/4] live status — Ctrl+C to stop"
echo ""
echo "    Record:  ssh ${JETSON_USER}@${JETSON_HOST} \\"
echo "             ~/noetic_fastlio_ws/scripts/onboard_record_noetic.sh tag=run1"
echo "    Stream:  JETSON_PASS=${JETSON_PASS} ./scripts/real/stream_cloud_live.sh"
echo ""

while true; do
  STATS=$($SSH "
    source /opt/ros/noetic/setup.bash
    export ROS_MASTER_URI=http://${JETSON_HOST}:11311
    ODOM=\$(timeout 2 rostopic hz /robot/Odometry 2>&1 | awk '/average rate/{print \$3; exit}')
    IMU=\$(timeout 2 rostopic hz /livox/imu 2>&1 | awk '/average rate/{print \$3; exit}')
    POS=\$(timeout 1 rostopic echo -n 1 --noarr /robot/Odometry 2>/dev/null \
          | awk '/position:/{getline; printf \"x=%+.2f \", \$2; getline; printf \"y=%+.2f \", \$2; getline; printf \"z=%+.2f\", \$2; exit}')
    # Recording detection: ACTIVE if a rosbag record process is alive AND
    # a .bag.active file exists. If only the file exists (no process),
    # that's a stale orphan from a crashed previous run.
    if pgrep -f 'rosbag record' >/dev/null 2>&1; then
      ACTIVE=\$(ls /home/unitree/bags/*.bag.active 2>/dev/null | head -1)
      if [[ -n \"\$ACTIVE\" ]]; then
        SZ=\$(du -h \"\$ACTIVE\" 2>/dev/null | cut -f1)
        REC=\"REC [\$SZ]\"
      else
        REC='REC [warming]'
      fi
    else
      REC='idle'
    fi
    BAGS=\$(ls /home/unitree/bags/*.bag 2>/dev/null | wc -l)
    echo \"\$REC | odom=\${ODOM:-?}Hz  imu=\${IMU:-?}Hz  pos[\$POS]  saved=\${BAGS}bags\"
  " 2>/dev/null) || STATS="ssh failed (link flap?)"
  printf "\r[%s] %-90s" "$(date +%H:%M:%S)" "$STATS"
  sleep 4
done
