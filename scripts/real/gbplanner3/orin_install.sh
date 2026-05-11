#!/usr/bin/env bash
# Native install of GBPlanner3 on Jetson Orin (JP 5.x / Ubuntu 20.04 Focal / ARM64).
#
# Co-exists with existing ROS 2 Foxy install. No Docker.
#
# Run on Orin via SSH. Assumes Foxy + Fast-LIO already there per CLAUDE.md
# "onboard SLAM split (2026-04-30)".
#
# Total: ~30-40 min on Orin AGX 32GB, ~50-90 min on Orin NX 16GB.

set -euo pipefail

GBPLANNER_WS="${GBPLANNER_WS:-$HOME/gbplanner3_ws}"
UAS_REPO_URL="https://github.com/ntnu-arl/unified_autonomy_stack.git"
PARALLEL_JOBS="${PARALLEL_JOBS:-4}"     # Orin: don't push past 4, will OOM in heavy translation units

echo "==> Step 1: verify Focal + ARM64"
if ! grep -q 'focal' /etc/lsb-release; then
    echo "ERROR: Not Ubuntu 20.04 Focal. This script targets JetPack 5.x."
    exit 1
fi
[ "$(uname -m)" = "aarch64" ] || { echo "ERROR: Not ARM64"; exit 1; }

echo "==> Step 2: add ROS 1 Noetic apt source (alongside existing Foxy)"
if [ ! -f /etc/apt/sources.list.d/ros-noetic.list ]; then
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" \
        > /etc/apt/sources.list.d/ros-noetic.list'
    curl -sSL 'https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc' \
        | sudo apt-key add -
fi
sudo apt-get update

echo "==> Step 3: install ros-noetic-desktop + build tools"
sudo apt-get install -y --no-install-recommends \
    ros-noetic-desktop \
    python3-catkin-tools \
    python3-osrf-pycommon \
    python3-vcstool \
    libgoogle-glog-dev \
    libspdlog-dev \
    libpcap-dev \
    git \
    git-lfs \
    ros-noetic-octomap-msgs \
    ros-noetic-octomap-ros \
    ros-noetic-joy \
    ros-noetic-twist-mux \
    ros-noetic-interactive-marker-twist-server \
    ros-noetic-tf-conversions \
    ros-noetic-geographic-msgs \
    ros-noetic-diagnostic-updater \
    ros-noetic-urdf

echo "==> Step 4: install ros-foxy-ros1-bridge (ARM64 Focal binary exists, verified)"
sudo apt-get install -y --no-install-recommends ros-foxy-ros1-bridge

echo "==> Step 5: rosdep init (if not yet)"
if [ ! -d /etc/ros/rosdep/sources.list.d ]; then
    sudo rosdep init
fi
rosdep update --include-eol-distros

echo "==> Step 6: set up gbplanner3 workspace at $GBPLANNER_WS"
mkdir -p "$GBPLANNER_WS/src"
cd "$GBPLANNER_WS"

# Pull UAS to get manifest files (we only use the .repos manifests, not Docker)
if [ ! -d "$HOME/unified_autonomy_stack" ]; then
    git clone --depth 1 "$UAS_REPO_URL" "$HOME/unified_autonomy_stack"
fi

echo "==> Step 7: convert SSH URLs to HTTPS in manifest, then vcs import"
MANIFEST="$HOME/unified_autonomy_stack/repos/ws_gbplanner.repos"
HTTPS_MANIFEST="/tmp/ws_gbplanner_https.repos"
sed 's|git@github.com:|https://github.com/|g; s|\.git$|.git|g' "$MANIFEST" > "$HTTPS_MANIFEST"
vcs import --recursive src < "$HTTPS_MANIFEST"

# Also pull pci_general's bridge config + ROS 1 launcher
# (these come via robot_bringup.repos but are needed for launch files)
ROBOT_BRINGUP_MANIFEST="/tmp/robot_bringup_https.repos"
sed 's|git@github.com:|https://github.com/|g' \
    "$HOME/unified_autonomy_stack/repos/robot_bringup.repos" > "$ROBOT_BRINGUP_MANIFEST"
vcs import src < "$ROBOT_BRINGUP_MANIFEST" || \
    echo "WARNING: robot_bringup import partial; verify https URL is accessible"

echo "==> Step 8: rosdep install missing deps"
source /opt/ros/noetic/setup.bash
rosdep install --from-paths src --ignore-src -r -y \
    --skip-keys="gazebo11 libgazebo11-dev" \
    || echo "WARNING: some deps may be missing; will retry during build"

echo "==> Step 9: catkin build (Release, $PARALLEL_JOBS jobs)"
catkin config --extend /opt/ros/noetic -DCMAKE_BUILD_TYPE=Release
catkin build -j"$PARALLEL_JOBS" --no-status

echo ""
echo "==> SUCCESS. gbplanner3 binaries at: $GBPLANNER_WS/devel/lib/gbplanner"
echo ""
echo "To use in this shell:"
echo "  source $GBPLANNER_WS/devel/setup.bash"
echo ""
echo "Verify install:"
echo "  rospack find gbplanner"
echo "  rosnode info /gbplanner_node    # (after roscore + gbplanner launch)"
echo ""
echo "Next: launch with our Collab_QRC Fast-LIO inputs:"
echo "  bash $(dirname $0)/orin_launch_gbplanner.sh"
