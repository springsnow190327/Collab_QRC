#!/usr/bin/env bash
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
LOCAL_ROOT="${LOCAL_GTSAM_ROOT:-${WS_DIR}/.local_deps/gtsam_humble}"
LOCAL_PREFIX="${LOCAL_GTSAM_PREFIX:-${LOCAL_ROOT}/extract/opt/ros/humble}"
DEB_DIR="${LOCAL_ROOT}/debs"
SRC_DIR="${LOCAL_ROOT}/src"
BUILD_DIR="${LOCAL_ROOT}/build"

mkdir -p "${DEB_DIR}" "${LOCAL_ROOT}/extract"

try_sudo_apt() {
  if sudo -n true >/dev/null 2>&1; then
    sudo apt-get update
    sudo apt-get install -y libgtsam-dev || sudo apt-get install -y ros-humble-gtsam
    sudo ldconfig
    return 0
  fi
  return 1
}

install_ros_deb_local() {
  (
    cd "${DEB_DIR}"
    apt-get download ros-humble-gtsam
  )
  dpkg-deb -x "${DEB_DIR}"/ros-humble-gtsam_*_amd64.deb "${LOCAL_ROOT}/extract"
}

build_source_local() {
  mkdir -p "${SRC_DIR}" "${BUILD_DIR}" "${LOCAL_PREFIX}"
  if [[ ! -d "${SRC_DIR}/gtsam" ]]; then
    for ref in 4.2.0 4.2 4.1.1; do
      if git clone --depth 1 --branch "${ref}" https://github.com/borglab/gtsam.git "${SRC_DIR}/gtsam"; then
        break
      fi
      rm -rf "${SRC_DIR}/gtsam"
    done
    if [[ ! -d "${SRC_DIR}/gtsam" ]]; then
      echo "ERROR: failed to clone a supported GTSAM release tag" >&2
      return 1
    fi
  fi
  cmake -S "${SRC_DIR}/gtsam" -B "${BUILD_DIR}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="${LOCAL_PREFIX}" \
    -DGTSAM_BUILD_PYTHON=OFF \
    -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
    -DGTSAM_BUILD_TESTS=OFF \
    -DGTSAM_WITH_TBB=OFF
  cmake --build "${BUILD_DIR}" --parallel "$(nproc)"
  cmake --install "${BUILD_DIR}"
}

if ! try_sudo_apt; then
  if ! install_ros_deb_local; then
    build_source_local
  fi
fi

LOCAL_GTSAM_PREFIX="${LOCAL_PREFIX}" "${WS_DIR}/scripts/setup/check_gtsam_backend.sh"
