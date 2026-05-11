#!/usr/bin/env bash
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
DEST="${SLAM_BACKEND_SOURCE_DIR:-${WS_DIR}/external}"
mkdir -p "${DEST}"

fetch_repo() {
  local name="$1"
  local url="$2"
  local dir="${DEST}/${name}"
  if [[ -d "${dir}/.git" ]]; then
    git -C "${dir}" fetch --depth 1 origin
    git -C "${dir}" reset --hard FETCH_HEAD
  else
    git clone --depth 1 "${url}" "${dir}"
  fi
}

fetch_repo "Swarm-LIO2" "https://github.com/hku-mars/Swarm-LIO2.git"
fetch_repo "dynamic_lio" "https://github.com/ZikangYuan/dynamic_lio.git"
fetch_repo "ERASOR" "https://github.com/LimHyungTae/ERASOR.git"

cat > "${DEST}/BACKENDS.md" <<EOF
# External SLAM Backends

- Swarm-LIO2: ${DEST}/Swarm-LIO2
- Dynamic-LIO: ${DEST}/dynamic_lio
- ERASOR: ${DEST}/ERASOR

These sources are intentionally kept under external/ as host-local clones and are not committed.
EOF
