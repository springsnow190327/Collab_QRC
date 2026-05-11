#!/usr/bin/env bash
# Source this file from the workspace root before running launch files or demos.

_COLLAB_QRC_NOUNSET=0
case "$-" in
  *u*) _COLLAB_QRC_NOUNSET=1; set +u ;;
esac

_COLLAB_QRC_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/humble/setup.bash

_collab_qrc_prepend_unique() {
  local var_name="$1"
  local value="$2"
  [ -n "${value}" ] || return 0
  [ -e "${value}" ] || return 0
  eval "case \":\${${var_name}:-}:\" in *\":${value}:\"*) return 0 ;; esac"
  eval "export ${var_name}=\"${value}:\${${var_name}:-}\""
}

_COLLAB_QRC_XACRO_PREFIX="${_COLLAB_QRC_DIR}/.local_deps/ros-humble-xacro/opt/ros/humble"
if [ ! -d /opt/ros/humble/share/xacro ] && [ -d "${_COLLAB_QRC_XACRO_PREFIX}/share/xacro" ]; then
  _collab_qrc_prepend_unique AMENT_PREFIX_PATH "${_COLLAB_QRC_XACRO_PREFIX}"
  _collab_qrc_prepend_unique CMAKE_PREFIX_PATH "${_COLLAB_QRC_XACRO_PREFIX}"
  _collab_qrc_prepend_unique PATH "${_COLLAB_QRC_XACRO_PREFIX}/bin"
  _collab_qrc_prepend_unique PYTHONPATH "${_COLLAB_QRC_XACRO_PREFIX}/local/lib/python3.10/dist-packages"
fi

_COLLAB_QRC_ROS_RUNTIME="${_COLLAB_QRC_DIR}/.local_deps/ros_runtime"
_COLLAB_QRC_ROS_RUNTIME_PREFIX="${_COLLAB_QRC_ROS_RUNTIME}/opt/ros/humble"
if [ -d "${_COLLAB_QRC_ROS_RUNTIME_PREFIX}/share" ]; then
  _collab_qrc_prepend_unique AMENT_PREFIX_PATH "${_COLLAB_QRC_ROS_RUNTIME_PREFIX}"
  _collab_qrc_prepend_unique CMAKE_PREFIX_PATH "${_COLLAB_QRC_ROS_RUNTIME_PREFIX}"
  _collab_qrc_prepend_unique PATH "${_COLLAB_QRC_ROS_RUNTIME_PREFIX}/bin"
  _collab_qrc_prepend_unique LD_LIBRARY_PATH "${_COLLAB_QRC_ROS_RUNTIME_PREFIX}/lib"
  _collab_qrc_prepend_unique LD_LIBRARY_PATH "${_COLLAB_QRC_ROS_RUNTIME}/usr/lib"
  _collab_qrc_prepend_unique LD_LIBRARY_PATH "${_COLLAB_QRC_ROS_RUNTIME}/usr/lib/x86_64-linux-gnu"
  _collab_qrc_prepend_unique PYTHONPATH "${_COLLAB_QRC_ROS_RUNTIME_PREFIX}/local/lib/python3.10/dist-packages"
fi

# nav2 humble runtime — extracted via `apt download` into .local_deps/nav2_humble
# (no-sudo path; see scripts/ops/install_nav2_local.sh if it ever needs to be
# reproduced). Activates ros-humble-nav2-* (mppi, smac, bringup, lifecycle,
# behaviors, bt_navigator, etc.) so the nav2_mppi backend works without
# `sudo apt install`. CLAUDE.md 2026-04-29 marks nav2_mppi as the production
# stack, but the system apt is unmodified — this prefix layers it in.
_COLLAB_QRC_NAV2_HUMBLE="${_COLLAB_QRC_DIR}/.local_deps/nav2_humble/extract"
_COLLAB_QRC_NAV2_HUMBLE_PREFIX="${_COLLAB_QRC_NAV2_HUMBLE}/opt/ros/humble"
if [ -d "${_COLLAB_QRC_NAV2_HUMBLE_PREFIX}/share" ]; then
  _collab_qrc_prepend_unique AMENT_PREFIX_PATH "${_COLLAB_QRC_NAV2_HUMBLE_PREFIX}"
  _collab_qrc_prepend_unique CMAKE_PREFIX_PATH "${_COLLAB_QRC_NAV2_HUMBLE_PREFIX}"
  _collab_qrc_prepend_unique PATH "${_COLLAB_QRC_NAV2_HUMBLE_PREFIX}/bin"
  _collab_qrc_prepend_unique LD_LIBRARY_PATH "${_COLLAB_QRC_NAV2_HUMBLE_PREFIX}/lib"
  _collab_qrc_prepend_unique LD_LIBRARY_PATH "${_COLLAB_QRC_NAV2_HUMBLE_PREFIX}/lib/x86_64-linux-gnu"
  _collab_qrc_prepend_unique PYTHONPATH "${_COLLAB_QRC_NAV2_HUMBLE_PREFIX}/local/lib/python3.10/dist-packages"
  _collab_qrc_prepend_unique PYTHONPATH "${_COLLAB_QRC_NAV2_HUMBLE_PREFIX}/lib/python3.10/site-packages"
fi
unset _COLLAB_QRC_NAV2_HUMBLE_PREFIX
unset _COLLAB_QRC_NAV2_HUMBLE

# Local CUDA 12.4 toolkit installed via runfile in $HOME/cuda-12.4
# (no-sudo, see docs/3DGS_INTEGRATION.md). Required by gsplat for
# JIT-compiling its sm_89 kernels on RTX 4070 Ada. Activate only if
# the directory exists so machines without it stay unaffected.
if [ -d "${HOME}/cuda-12.4/bin" ]; then
  export CUDA_HOME="${HOME}/cuda-12.4"
  _collab_qrc_prepend_unique PATH "${CUDA_HOME}/bin"
  _collab_qrc_prepend_unique LD_LIBRARY_PATH "${CUDA_HOME}/lib64"
  # Default arch list to RTX 4070 Ada. Override by setting before sourcing.
  export TORCH_CUDA_ARCH_LIST="${TORCH_CUDA_ARCH_LIST:-8.9}"
fi

_COLLAB_QRC_CONDA_SITE=""
if [ -n "${CONDA_PREFIX:-}" ]; then
  _COLLAB_QRC_CONDA_SITE="${CONDA_PREFIX}/lib/python3.10/site-packages"
elif [ -d "${HOME}/micromamba/envs/cmu_env/lib/python3.10/site-packages" ]; then
  _COLLAB_QRC_CONDA_SITE="${HOME}/micromamba/envs/cmu_env/lib/python3.10/site-packages"
fi
_collab_qrc_prepend_unique PYTHONPATH "${_COLLAB_QRC_CONDA_SITE}"

source "${_COLLAB_QRC_DIR}/install/setup.bash"

unset _COLLAB_QRC_CONDA_SITE
unset _COLLAB_QRC_ROS_RUNTIME_PREFIX
unset _COLLAB_QRC_ROS_RUNTIME
unset _COLLAB_QRC_XACRO_PREFIX
unset _COLLAB_QRC_DIR
unset -f _collab_qrc_prepend_unique
if [ "${_COLLAB_QRC_NOUNSET}" = 1 ]; then
  set -u
fi
unset _COLLAB_QRC_NOUNSET
