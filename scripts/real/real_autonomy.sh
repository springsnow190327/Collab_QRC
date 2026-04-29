#!/bin/bash
# real_autonomy.sh — single entry point for real Go2W autonomy.
#
# Flag surface:
#   robot={go2w|go2}                     default: go2w   (go2 = no-wheel Unitree Go2)
#   connect={ethernet|webrtc}            default: ethernet
#   slam={auto|carto_l1|fastlio_mid360}  default: auto
#                                        (auto → ping-probe Mid-360 at
#                                         $GO2W_MID360_IP (default
#                                         192.168.123.120); if reachable
#                                         select fastlio_mid360, else
#                                         carto_l1. Detection runs after
#                                         ensure_link so the dongle is up.
#                                         Override GO2W_MID360_IP or set
#                                         slam=carto_l1 explicitly to skip.)
#   nav={nav2_mppi|cfpa2|tare|tare_real|far}    default: nav2_mppi
#                                        nav2_mppi = Nav2 SmacPlannerHybrid + MPPI +
#                                                    behavior_server + stuck_watchdog +
#                                                    fast_lio_tf_adapter
#                                                    (production stack since 2026-04-29).
#                                        cfpa2     = legacy default_nav.py (Python A* +
#                                                    D* Lite + recovery).
#                                        tare_real = real CMU TARE → localPlanner
#                                                    direct (FAR unwired, CFPA2 off)
#   mapper={scan|carto_binary|carto_2d}  default: carto_2d
#                                        (carto_2d = Cartographer 2D grid + binarizer,
#                                         best for occupancy + dynamic obstacles;
#                                         scan = simple_scan_mapper, no free-space
#                                         carving, kept for fallback / FAR runs)
#   oa={true|false}                      default: true    (Unitree api_id=1003)
#   execute={true|false}                 default: true    (false = dry-run, sport API disconnected)
#   rviz={true|false}                    default: true    (RViz 2D top-down)
#   rviz_config={autonomy|cartographer|cartographer_grid|octomap}.rviz
#                                        default: autonomy.rviz
#   rviz_3d={true|false}                 default: true    (second RViz window,
#                                        3D perspective with octomap voxels +
#                                        registered cloud + red robot triangle;
#                                        spawns a viz-only octomap_server when
#                                        slam=carto_l1)
#   carto_mode={2d|3d}                   default: 2d      (carto_l1 only; auto-forced
#                                        to 2d when mapper=carto_2d)
#
# Examples:
#   ./real_autonomy.sh                                         # default Go2W stack
#   ./real_autonomy.sh robot=go2                               # Go2 (no-wheel)
#   ./real_autonomy.sh slam=fastlio_mid360 nav=far
#   ./real_autonomy.sh nav=tare mapper=carto_2d
#   ./real_autonomy.sh stop                                    # kill everything

set -e
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
REPO_ROOT="$( cd "$SCRIPT_DIR/../.." &> /dev/null && pwd )"

# ── Cleanup function ─────────────────────────────────────────────────
# Used by both the "stop" subcommand and the SIGINT/SIGTERM trap that fires
# on Ctrl+C. ros2 launch is unreliable about propagating SIGINT to its
# ExecuteProcess children (Python helpers especially linger), so we
# explicitly walk the process tree.
_kill_autonomy_stack() {
  # Catch the launch wrapper itself first so it doesn't respawn children
  # while we're killing them.
  pkill -9 -f "ros2 launch go2w_real_bringup" 2>/dev/null || true
  pkill -9 -f "real_single.launch.py\|real_single_tare_real.launch.py" 2>/dev/null || true
  for p in \
      cartographer_node cartographer_occupancy transform_everything \
      default_nav cfpa2 carto_odom_bridge twist_bridge \
      cmd_vel_activity_mux cmd_vel_to_sport octomap_server \
      elevation_to_occupancy simple_scan_mapper probability_grid_binarizer \
      frontier_3d_markers pointcloud_to_laserscan fastlio_mapping \
      tare_planner_node waypoint_mux far_planner \
      livox_ros_driver2_node \
      controller_server planner_server behavior_server bt_navigator \
      lifecycle_manager_navigation lifecycle_manager \
      fast_lio_tf_adapter cfpa2_to_nav2_bridge path_relay stuck_watchdog \
      cfpa2_single_robot_node \
      wall_collision_checker autonomy_enabler supervisor_panic_node \
      exploration_metrics_logger scan_rear_filter \
      joy_node teleop_node \
      static_transform_publisher; do
    pkill -9 -f "$p" 2>/dev/null || true
  done
  killall -9 rviz2 2>/dev/null || true
  (ros2 daemon stop &>/dev/null &); sleep 1
  pkill -9 -f _ros2_daemon 2>/dev/null || true
  # Clear FastRTPS shared-memory residue so the next launch comes up clean.
  rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true
}

# ── Defaults ──────────────────────────────────────────────────────────
ROBOT="go2w"
CONNECT="ethernet"
SLAM="auto"
NAV="nav2_mppi"
MAPPER="carto_2d"
OA="true"
EXECUTE="true"
RVIZ="true"
RVIZ_CONFIG="autonomy.rviz"
RVIZ_3D="true"
CARTO_MODE="2d"

# ── Parse key=value args (and "stop") ─────────────────────────────────
for arg in "$@"; do
  case "$arg" in
    stop)
      echo "Stopping all real-robot autonomy processes..."
      _kill_autonomy_stack
      echo "Done."
      exit 0
      ;;
    robot=*)   ROBOT="${arg#robot=}" ;;
    connect=*) CONNECT="${arg#connect=}" ;;
    slam=*)    SLAM="${arg#slam=}" ;;
    nav=*)     NAV="${arg#nav=}" ;;
    mapper=*)  MAPPER="${arg#mapper=}" ;;
    oa=*)      OA="${arg#oa=}" ;;
    execute=*) EXECUTE="${arg#execute=}" ;;
    rviz=*)    RVIZ="${arg#rviz=}" ;;
    rviz_config=*) RVIZ_CONFIG="${arg#rviz_config=}" ;;
    rviz_3d=*) RVIZ_3D="${arg#rviz_3d=}" ;;
    carto_mode=*) CARTO_MODE="${arg#carto_mode=}" ;;
    *) echo "WARN: unknown arg '$arg'" >&2 ;;
  esac
done

# ── Validate ──────────────────────────────────────────────────────────
case "$ROBOT"   in go2w|go2) ;; *) echo "ERROR: robot must be go2w|go2" >&2; exit 1 ;; esac
case "$CONNECT" in ethernet|webrtc) ;; *) echo "ERROR: connect must be ethernet|webrtc" >&2; exit 1 ;; esac
case "$SLAM"    in auto|carto_l1|fastlio_mid360) ;; *) echo "ERROR: slam must be auto|carto_l1|fastlio_mid360" >&2; exit 1 ;; esac
case "$NAV"     in nav2_mppi|cfpa2|tare|tare_real|far) ;; *) echo "ERROR: nav must be nav2_mppi|cfpa2|tare|tare_real|far" >&2; exit 1 ;; esac
case "$MAPPER"  in scan|carto_binary|carto_2d) ;; *) echo "ERROR: mapper must be scan|carto_binary|carto_2d" >&2; exit 1 ;; esac
case "$OA"      in true|false) ;; *) echo "ERROR: oa must be true|false" >&2; exit 1 ;; esac
case "$EXECUTE" in true|false) ;; *) echo "ERROR: execute must be true|false" >&2; exit 1 ;; esac
case "$RVIZ"    in true|false) ;; *) echo "ERROR: rviz must be true|false" >&2; exit 1 ;; esac
case "$RVIZ_3D" in true|false) ;; *) echo "ERROR: rviz_3d must be true|false" >&2; exit 1 ;; esac
case "$CARTO_MODE" in 2d|3d) ;; *) echo "ERROR: carto_mode must be 2d|3d" >&2; exit 1 ;; esac

# ── Connection (Ethernet is validated here; WebRTC path exits early) ──
if [[ "$CONNECT" == "webrtc" ]]; then
  exec "$SCRIPT_DIR/connect_webrtc.sh"
fi
source "$SCRIPT_DIR/connect_ethernet.sh"
ensure_link
setup_cyclonedds_ethernet

# ── LiDAR autodetection ──────────────────────────────────────────────
# Only runs when user requested slam=auto. Uses detect_lidar() from
# connect_ethernet.sh (ping-based, ~1s timeout). Falls back to carto_l1
# silently if Mid-360 is absent.
SLAM_AUTO_USED="false"
if [[ "$SLAM" == "auto" ]]; then
  SLAM_AUTO_USED="true"
  DETECTED="$(detect_lidar)"
  case "$DETECTED" in
    mid360) SLAM="fastlio_mid360" ;;
    l1)     SLAM="carto_l1" ;;
    *)      echo "WARN: detect_lidar returned '$DETECTED'; defaulting to carto_l1" >&2
            SLAM="carto_l1" ;;
  esac
fi

# ── Source workspace ─────────────────────────────────────────────────
source /opt/ros/humble/setup.bash
[[ -f "$REPO_ROOT/install/setup.bash" ]] && source "$REPO_ROOT/install/setup.bash"

# ── Map flag → launch ────────────────────────────────────────────────
case "$NAV" in
  # Default since 2026-04-29: Nav2 + SmacPlannerHybrid + MPPI +
  # behavior_server + lifecycle_manager + fast_lio_tf_adapter +
  # stuck_watchdog + cfpa2_to_nav2_bridge. See
  # docs/claude/nav2_mppi_journey.md for the full stack rationale.
  nav2_mppi) LAUNCH="real_single.launch.py";      NAV_BACKEND="nav2_mppi" ;;
  cfpa2)     LAUNCH="real_single.launch.py";      NAV_BACKEND="reactive" ;;
  far)       LAUNCH="real_single.launch.py";      NAV_BACKEND="far" ;;
  tare)      LAUNCH="real_single_tare.launch.py"; NAV_BACKEND="reactive" ;;
  # Real CMU TARE → localPlanner direct (FAR unwired, watchdog armed).
  # Full CMU autonomy stack; CFPA2 is disabled. See
  # src/go2w/go2w_real_bringup/launch/real_single_tare_real.launch.py.
  tare_real) LAUNCH="real_single_tare_real.launch.py"; NAV_BACKEND="far" ;;
esac

BANNER_MODE="LIVE"
[[ "$EXECUTE" == "false" ]] && BANNER_MODE="DRY-RUN (sport API disconnected)"

echo ""
echo "################################################"
echo "  Unitree REAL autonomy   [$BANNER_MODE]"
SLAM_BANNER="$SLAM ($CARTO_MODE)"
[[ "$SLAM_AUTO_USED" == "true" ]] && SLAM_BANNER="$SLAM (autodetected from $MID360_IP)"

echo "    robot   : $ROBOT"
echo "    connect : $CONNECT"
echo "    slam    : $SLAM_BANNER"
echo "    nav     : $NAV ($NAV_BACKEND)"
echo "    mapper  : $MAPPER"
echo "    oa      : $OA"
echo "    execute : $EXECUTE"
echo "  Launch    : go2w_real_bringup $LAUNCH"
echo "  Stop      : Ctrl+C  or  scripts/real/real_autonomy.sh stop"
echo "################################################"
echo ""

# Run launch as a background child so we can intercept Ctrl+C.
# Without this trap, a Ctrl+C only kills the foreground ros2 launch wrapper
# while several ExecuteProcess children (Python helpers, Nav2 nodes spawned
# in lifecycle groups) survive — leading to "two parallel launches" the next
# time the user starts the stack. Trapping INT/TERM and routing through
# _kill_autonomy_stack guarantees a clean tree teardown.
ros2 launch go2w_real_bringup "$LAUNCH" \
  robot_model:="$ROBOT" \
  slam:="$SLAM" \
  carto_mode:="$CARTO_MODE" \
  nav_backend:="$NAV_BACKEND" \
  map_backend:="$MAPPER" \
  obstacle_avoidance:="$OA" \
  execute_controller:="$EXECUTE" \
  rviz:="$RVIZ" \
  rviz_config:="$RVIZ_CONFIG" \
  rviz_3d:="$RVIZ_3D" &
LAUNCH_PID=$!

cleanup_on_signal() {
  trap - INT TERM EXIT  # avoid re-entrance
  echo ""
  echo "Caught interrupt — tearing down autonomy stack..."
  _kill_autonomy_stack
  echo "Done."
  exit 0
}
trap cleanup_on_signal INT TERM

# Block until launch exits or a signal comes in. `wait` is interruptible
# by SIGINT (unlike `tail -f` style waits), which lets the trap run.
wait "$LAUNCH_PID"
EXIT_CODE=$?
trap - INT TERM
exit "$EXIT_CODE"
