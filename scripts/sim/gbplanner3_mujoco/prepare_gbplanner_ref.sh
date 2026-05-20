#!/usr/bin/env bash
# Prepare the Unified Autonomy Stack GBPlanner workspace for Collab_QRC.
#
# The Collab_QRC benchmark keeps MuJoCo, Fast-LIO, Nav2 MPPI, safety, and
# metrics in this repo.  GBPlanner2/3 run as upstream ROS1 high-level planners
# in the UAS Docker workspace; this helper only switches the upstream source
# ref and rebuilds when the ref changes.
set -euo pipefail

version="${1:-${GBPLANNER_VERSION:-gbplanner3}}"
case "${version}" in
  gbplanner2|gbplanner3) ;;
  *)
    echo "ERROR: GBPlanner version must be gbplanner2 or gbplanner3 (got '${version}')" >&2
    exit 2
    ;;
esac

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
collab_root="$(cd "${script_dir}/../../.." && pwd)"
uas_root="${UAS_REPO_ROOT:-$HOME/Research/uas_deploy/unified_autonomy_stack}"
gbplanner_src="${GBPLANNER_SRC:-${uas_root}/workspaces/ws_gbplanner/src/exploration/gbplanner_ros}"
adaptive_obb_src="${GBPLANNER_ADAPTIVE_OBB_SRC:-${uas_root}/workspaces/ws_gbplanner/src/exploration/adaptive_obb_ros}"
pci_general_src="${GBPLANNER_PCI_GENERAL_SRC:-${uas_root}/workspaces/ws_gbplanner/src/exploration/pci_general}"
stamp_file="${GBPLANNER_BUILD_STAMP:-${uas_root}/workspaces/ws_gbplanner/.collab_qrc_gbplanner_ref}"
pci_general_statevec_patch="${collab_root}/scripts/sim/gbplanner3_mujoco/patches/pci_general_statevec.patch"
pci_general_current_vel_patch="${collab_root}/scripts/sim/gbplanner3_mujoco/patches/pci_general_current_vel.patch"

case "${version}" in
  gbplanner2)
    target_ref="${GBPLANNER2_REF:-origin/gbplanner2}"
    local_branch="${GBPLANNER2_LOCAL_BRANCH:-collab_qrc_gbplanner2}"
    adaptive_obb_ref="${GBPLANNER2_ADAPTIVE_OBB_REF:-origin/master}"
    adaptive_obb_branch="${GBPLANNER2_ADAPTIVE_OBB_LOCAL_BRANCH:-collab_qrc_gbplanner2}"
    ;;
  gbplanner3)
    target_ref="${GBPLANNER3_REF:-origin/gbplanner3_test}"
    local_branch="${GBPLANNER3_LOCAL_BRANCH:-collab_qrc_gbplanner3}"
    adaptive_obb_ref="${GBPLANNER3_ADAPTIVE_OBB_REF:-origin/gbplanner3}"
    adaptive_obb_branch="${GBPLANNER3_ADAPTIVE_OBB_LOCAL_BRANCH:-collab_qrc_gbplanner3}"
    ;;
esac

[[ -d "${uas_root}" ]] || {
  echo "ERROR: UAS_REPO_ROOT not found: ${uas_root}" >&2
  exit 1
}
[[ -d "${gbplanner_src}/.git" ]] || {
  echo "ERROR: GBPlanner source checkout not found: ${gbplanner_src}" >&2
  exit 1
}
[[ -d "${adaptive_obb_src}/.git" ]] || {
  echo "ERROR: adaptive_obb_ros source checkout not found: ${adaptive_obb_src}" >&2
  exit 1
}
command -v git >/dev/null 2>&1 || {
  echo "ERROR: git command not found" >&2
  exit 1
}
command -v docker >/dev/null 2>&1 || {
  echo "ERROR: docker command not found" >&2
  exit 1
}

if ! git -C "${gbplanner_src}" diff --quiet --ignore-submodules --; then
  echo "ERROR: GBPlanner source has unstaged tracked changes:" >&2
  git -C "${gbplanner_src}" status --short >&2
  echo "Refusing to switch branches because that would risk losing local work." >&2
  exit 1
fi
if ! git -C "${gbplanner_src}" diff --cached --quiet --ignore-submodules --; then
  echo "ERROR: GBPlanner source has staged tracked changes:" >&2
  git -C "${gbplanner_src}" status --short >&2
  echo "Refusing to switch branches because that would risk losing local work." >&2
  exit 1
fi
if ! git -C "${adaptive_obb_src}" diff --quiet --ignore-submodules --; then
  echo "ERROR: adaptive_obb_ros source has unstaged tracked changes:" >&2
  git -C "${adaptive_obb_src}" status --short >&2
  echo "Refusing to switch branches because that would risk losing local work." >&2
  exit 1
fi
if ! git -C "${adaptive_obb_src}" diff --cached --quiet --ignore-submodules --; then
  echo "ERROR: adaptive_obb_ros source has staged tracked changes:" >&2
  git -C "${adaptive_obb_src}" status --short >&2
  echo "Refusing to switch branches because that would risk losing local work." >&2
  exit 1
fi

if ! git -C "${gbplanner_src}" fetch origin gbplanner2 gbplanner3_test; then
  echo "WARN: git fetch failed; using locally available refs if present." >&2
fi
if ! git -C "${adaptive_obb_src}" fetch origin master gbplanner3; then
  echo "WARN: adaptive_obb_ros git fetch failed; using locally available refs if present." >&2
fi

target_sha="$(git -C "${gbplanner_src}" rev-parse --verify "${target_ref}^{commit}")"
adaptive_obb_sha="$(git -C "${adaptive_obb_src}" rev-parse --verify "${adaptive_obb_ref}^{commit}")"
current_sha="$(git -C "${gbplanner_src}" rev-parse HEAD)"
if [[ "${current_sha}" != "${target_sha}" ]]; then
  echo "Switching GBPlanner source to ${version}: ${target_ref} (${target_sha})"
  git -C "${gbplanner_src}" checkout -B "${local_branch}" "${target_sha}"
else
  echo "GBPlanner source already at ${version}: ${target_ref} (${target_sha})"
fi
current_adaptive_obb_sha="$(git -C "${adaptive_obb_src}" rev-parse HEAD)"
if [[ "${current_adaptive_obb_sha}" != "${adaptive_obb_sha}" ]]; then
  echo "Switching adaptive_obb_ros to ${version}: ${adaptive_obb_ref} (${adaptive_obb_sha})"
  git -C "${adaptive_obb_src}" checkout -B "${adaptive_obb_branch}" "${adaptive_obb_sha}"
else
  echo "adaptive_obb_ros already at ${version}: ${adaptive_obb_ref} (${adaptive_obb_sha})"
fi

if [[ -d "${pci_general_src}/.git" ]]; then
  pci_header="${pci_general_src}/include/pci_general/pci_general.h"
  if [[ -f "${pci_header}" ]] && ! grep -q "typedef Eigen::Matrix<double, 5, 1> StateVec" "${pci_header}"; then
    [[ -f "${pci_general_statevec_patch}" ]] || {
      echo "ERROR: missing pci_general compatibility patch: ${pci_general_statevec_patch}" >&2
      exit 1
    }
    echo "Applying Collab_QRC pci_general StateVec compatibility patch"
    git -C "${pci_general_src}" apply "${pci_general_statevec_patch}"
  fi
  if [[ -f "${pci_header}" ]] && ! grep -q "current_vel_" "${pci_header}"; then
    [[ -f "${pci_general_current_vel_patch}" ]] || {
      echo "ERROR: missing pci_general compatibility patch: ${pci_general_current_vel_patch}" >&2
      exit 1
    }
    echo "Applying Collab_QRC pci_general current_vel_ compatibility patch"
    git -C "${pci_general_src}" apply "${pci_general_current_vel_patch}"
  fi
fi

stamp_payload="${version} ${target_sha} adaptive_obb ${adaptive_obb_sha}"
if [[ "${GBPLANNER_FORCE_BUILD:-0}" == "1" ]]; then
  echo "GBPLANNER_FORCE_BUILD=1; rebuilding GBPlanner workspace"
elif [[ -f "${stamp_file}" && "$(cat "${stamp_file}")" == "${stamp_payload}" && -d "${uas_root}/workspaces/ws_gbplanner/build" && -d "${uas_root}/workspaces/ws_gbplanner/devel" ]]; then
  echo "GBPlanner workspace already built for ${stamp_payload}"
  exit 0
fi

echo "Building UAS GBPlanner workspace for ${stamp_payload}"
rm -f "${stamp_file}"
(
  cd "${uas_root}"
  docker compose -f docker-compose.build.yml --profile build up \
    --remove-orphans \
    --exit-code-from build_gbplanner \
    build_gbplanner
)
mkdir -p "$(dirname "${stamp_file}")"
printf '%s\n' "${stamp_payload}" >"${stamp_file}"
echo "GBPlanner workspace build stamp: ${stamp_file}"
