#!/usr/bin/env bash
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
LOCAL_GTSAM_PREFIX="${LOCAL_GTSAM_PREFIX:-${WS_DIR}/.local_deps/gtsam_humble/extract/opt/ros/humble}"

json_bool() {
  if [[ "$1" == "true" ]]; then
    printf 'true'
  else
    printf 'false'
  fi
}

python_gtsam=false
cpp_ldconfig=false
cpp_cmake=false
cpp_headers=false
cpp_libraries=false

if python3 -c "import gtsam" >/tmp/collab_qrc_gtsam_python.txt 2>/tmp/collab_qrc_gtsam_python.err; then
  python_gtsam=true
fi

if ldconfig -p 2>/dev/null | grep -qi 'libgtsam'; then
  cpp_ldconfig=true
fi

if { [[ -f "${LOCAL_GTSAM_PREFIX}/include/gtsam/slam/BetweenFactor.h" ]] \
  && [[ -f "${LOCAL_GTSAM_PREFIX}/include/gtsam/nonlinear/LevenbergMarquardtOptimizer.h" ]]; } \
  || { [[ -f /usr/include/gtsam/slam/BetweenFactor.h ]] \
  && [[ -f /usr/include/gtsam/nonlinear/LevenbergMarquardtOptimizer.h ]]; } \
  || { [[ -f /usr/local/include/gtsam/slam/BetweenFactor.h ]] \
  && [[ -f /usr/local/include/gtsam/nonlinear/LevenbergMarquardtOptimizer.h ]]; }; then
  cpp_headers=true
fi

if find "${LOCAL_GTSAM_PREFIX}" -path '*/libgtsam.so*' -type f -print -quit 2>/dev/null | grep -q . \
  || find /usr /usr/local -path '*/libgtsam.so*' -type f -print -quit 2>/dev/null | grep -q . \
  || [[ "${cpp_ldconfig}" == "true" ]]; then
  cpp_libraries=true
fi

CHECK_DIR="$(mktemp -d /tmp/collab_qrc_gtsam_cmake.XXXXXX)"
trap 'rm -rf "${CHECK_DIR}"' EXIT
cat >"${CHECK_DIR}/CMakeLists.txt" <<'EOF_CMAKE'
cmake_minimum_required(VERSION 3.16)
project(check_gtsam CXX)
find_package(GTSAM REQUIRED CONFIG)
add_executable(check_gtsam main.cpp)
target_link_libraries(check_gtsam gtsam)
EOF_CMAKE
cat >"${CHECK_DIR}/main.cpp" <<'EOF_CPP'
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
int main() { return 0; }
EOF_CPP

export CMAKE_PREFIX_PATH="${LOCAL_GTSAM_PREFIX}:${CMAKE_PREFIX_PATH:-}"
if cmake -S "${CHECK_DIR}" -B "${CHECK_DIR}/build" \
  >/tmp/collab_qrc_gtsam_cmake.txt 2>/tmp/collab_qrc_gtsam_cmake.err; then
  if cmake --build "${CHECK_DIR}/build" -j2 \
    >>/tmp/collab_qrc_gtsam_cmake.txt 2>>/tmp/collab_qrc_gtsam_cmake.err; then
    cpp_cmake=true
  fi
fi

cpp_gtsam=false
if [[ "${cpp_cmake}" == "true" && "${cpp_headers}" == "true" && "${cpp_libraries}" == "true" ]]; then
  cpp_gtsam=true
fi

recommended_backend="g2o_export_only"
blocker=""
if [[ "${cpp_gtsam}" == "true" ]]; then
  recommended_backend="gtsam_cpp"
elif [[ "${python_gtsam}" == "true" ]]; then
  recommended_backend="gtsam_python"
else
  blocker="python_gtsam_not_found;gtsam_cpp_not_found"
fi

cat <<EOF
{
  "schema": "gtsam_backend_check/v2",
  "python_gtsam": $(json_bool "${python_gtsam}"),
  "cpp_gtsam": $(json_bool "${cpp_gtsam}"),
  "cpp_gtsam_ldconfig": $(json_bool "${cpp_ldconfig}"),
  "cpp_gtsam_cmake": $(json_bool "${cpp_cmake}"),
  "cpp_gtsam_headers": $(json_bool "${cpp_headers}"),
  "cpp_gtsam_libraries": $(json_bool "${cpp_libraries}"),
  "local_gtsam_prefix": "${LOCAL_GTSAM_PREFIX}",
  "recommended_backend": "${recommended_backend}",
  "blocker": "${blocker}"
}
EOF
