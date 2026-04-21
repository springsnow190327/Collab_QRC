#!/usr/bin/env bash
# QRC_demo workspace setup for Ubuntu 22.04 + ROS 2 Humble.
#
# Usage:
#   chmod +x setup.sh
#   ./setup.sh          # full install (ROS 2 + conda + deps + build)
#   ./setup.sh --deps   # system + ROS deps only, skip build
#   ./setup.sh --build  # skip installs, just build workspace
#
# Requires: Ubuntu 22.04 (Jammy), sudo access, internet.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONDA_ENV_NAME="cmu_env"
PYTHON_VERSION="3.10"

# ── Parse args ────────────────────────────────────────────────────────────────
INSTALL_DEPS=true
DO_BUILD=true
for arg in "$@"; do
  case "$arg" in
    --deps)  DO_BUILD=false ;;
    --build) INSTALL_DEPS=false ;;
    --help|-h)
      echo "Usage: ./setup.sh [--deps | --build]"
      echo "  (no args)  Full install: system deps + conda + ROS 2 + build"
      echo "  --deps     Install dependencies only, skip build"
      echo "  --build    Build workspace only, skip dependency install"
      exit 0
      ;;
  esac
done

log() { echo -e "\n\033[1;36m>>> $1\033[0m"; }

# ── 1. System packages ───────────────────────────────────────────────────────
if $INSTALL_DEPS; then

log "Installing system packages"
sudo apt-get update
sudo apt-get install -y --no-install-recommends \
  build-essential cmake git curl wget gnupg lsb-release software-properties-common \
  python3-pip python3-colcon-common-extensions python3-rosdep python3-vcstool \
  libglfw3-dev libx11-dev xorg-dev libopencv-dev libpcl-dev libeigen3-dev \
  libboost-all-dev libgflags-dev libgoogle-glog-dev \
  python3-numpy python3-scipy python3-pytest

# ── 2. ROS 2 Humble ──────────────────────────────────────────────────────────
if ! dpkg -s ros-humble-ros-base &>/dev/null; then
  log "Installing ROS 2 Humble"
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
  sudo apt-get update
fi

log "Installing ROS 2 Humble packages"
sudo apt-get install -y --no-install-recommends \
  ros-humble-ros-base \
  ros-humble-rviz2 \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-controller-manager \
  ros-humble-hardware-interface \
  ros-humble-pluginlib \
  ros-humble-realtime-tools \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-robot-localization \
  ros-humble-urdf \
  ros-humble-joint-state-publisher \
  ros-humble-tf2-tools \
  ros-humble-tf2-ros \
  ros-humble-tf2-geometry-msgs \
  ros-humble-tf2-sensor-msgs \
  ros-humble-pcl-ros \
  ros-humble-pcl-conversions \
  ros-humble-perception-pcl \
  ros-humble-cv-bridge \
  ros-humble-vision-opencv \
  ros-humble-pointcloud-to-laserscan \
  ros-humble-cartographer \
  ros-humble-cartographer-ros \
  ros-humble-joy \
  ros-humble-teleop-twist-joy \
  ros-humble-gazebo-ros \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-launch-xml \
  ros-humble-launch-yaml \
  ros-humble-rmw-fastrtps-cpp \
  ros-humble-fastrtps

# Initialize rosdep
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  log "Initializing rosdep"
  sudo rosdep init || true
fi
rosdep update --include-eol-distros || true

# ── 3. Conda / Micromamba ─────────────────────────────────────────────────────
if ! command -v micromamba &>/dev/null && ! command -v conda &>/dev/null; then
  log "Installing micromamba"
  curl -Ls https://micro.mamba.pm/api/micromamba/linux-64/latest \
    | tar -xvj -C /usr/local/bin --strip-components=1 bin/micromamba
  eval "$(micromamba shell hook -s bash)"
  micromamba shell init -s bash -p ~/micromamba
  echo 'eval "$(micromamba shell hook -s bash)"' >> ~/.bashrc
fi

# Activate shell hook
if command -v micromamba &>/dev/null; then
  eval "$(micromamba shell hook -s bash)" || true
fi

if ! micromamba env list 2>/dev/null | grep -q "${CONDA_ENV_NAME}" && \
   ! conda env list 2>/dev/null | grep -q "${CONDA_ENV_NAME}"; then
  log "Creating conda environment: ${CONDA_ENV_NAME}"
  micromamba create -n "${CONDA_ENV_NAME}" -c conda-forge \
    "python=${PYTHON_VERSION}" \
    numpy scipy matplotlib \
    -y
fi

log "Installing Python packages in ${CONDA_ENV_NAME}"
if command -v micromamba &>/dev/null; then
  eval "$(micromamba shell hook -s bash)"
  micromamba activate "${CONDA_ENV_NAME}"
elif command -v conda &>/dev/null; then
  source "$(conda info --base)/etc/profile.d/conda.sh"
  conda activate "${CONDA_ENV_NAME}"
fi

pip install --quiet --upgrade \
  mujoco \
  opencv-python-headless \
  Pillow \
  pyyaml \
  setuptools

# ── 4. rosdep install ─────────────────────────────────────────────────────────
log "Installing workspace rosdep dependencies"
source /opt/ros/humble/setup.bash
cd "${SCRIPT_DIR}"

# Ignore vendor packages that have custom build requirements
rosdep install --from-paths src \
  --ignore-src -r -y \
  --skip-keys="mujoco_ros2_control livox_ros_driver2" || true

fi  # end INSTALL_DEPS

# ── 5. Build workspace ───────────────────────────────────────────────────────
if $DO_BUILD; then

log "Building workspace"
cd "${SCRIPT_DIR}"

# Activate environment
if command -v micromamba &>/dev/null; then
  eval "$(micromamba shell hook -s bash)"
  micromamba activate "${CONDA_ENV_NAME}" 2>/dev/null || true
elif command -v conda &>/dev/null; then
  source "$(conda info --base)/etc/profile.d/conda.sh"
  conda activate "${CONDA_ENV_NAME}" 2>/dev/null || true
fi
source /opt/ros/humble/setup.bash

# Ignore catkin workspace
touch src/mtare_ros1_ws/COLCON_IGNORE 2>/dev/null || true

colcon build --symlink-install \
  --cmake-args -DPython3_EXECUTABLE="$(which python3)" \
  --parallel-workers "$(nproc)"

log "Build complete"
echo ""
echo "Source the workspace:"
echo "  source install/setup.bash"
echo ""
echo "Run the single-robot VLM demo:"
echo "  ./scripts/vlm_demo_mujoco.sh"
echo ""
echo "Run the dual-robot door task demo:"
echo "  ./scripts/door_demo_mujoco.sh"

fi  # end DO_BUILD
