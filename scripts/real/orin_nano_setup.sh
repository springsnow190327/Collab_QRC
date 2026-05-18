#!/usr/bin/env bash
# orin_nano_setup.sh — first-time bring-up for the Orin Nano Super HIL target.
#
# Run ON the Jetson (e.g. piped over SSH from the desktop):
#   ssh johnpork233@192.168.55.49 'bash -s' < scripts/real/orin_nano_setup.sh
#
# Sets the Jetson up to host the autonomy half of the Collab_QRC stack while
# the desktop runs MuJoCo + sensor publishers. Idempotent: safe to re-run.
#
# What it does (in order):
#   1. nvpmodel -m 0  +  jetson_clocks      (MAXN Super, 25W, lock all clocks)
#   2. swap status check                    (8GB shared mem on this board is tight)
#   3. CUDA paths in ~/.bashrc              (nvcc, lib, include)
#   4. apt: ROS 2 Humble nav/grid_map/tf2   (needed by elevation_mapping_cupy + Nav2)
#   5. pip: PyTorch for JetPack 6.2         (from jetson-ai-lab wheels, CUDA 12.6)
#   6. pip: CuPy 12.x                       (elevation_map raycast core)
#   7. pip: jetson-stats                    (jtop dashboard)
#   8. Smoke: torch.cuda.is_available() && cupy.cuda.runtime.getDeviceCount()>0
#
# What it does NOT do (deliberate):
#   - Change hostname (cheatsheet recommends but breaks current SSH; do manually)
#   - Install Tailscale (cheatsheet recommends but non-blocking for HIL)
#   - rsync the workspace (use deploy_to_orin_nano.sh from the desktop)
#   - build colcon (deploy script triggers that on a separate pass)
#
# Tunables via env:
#   SKIP_APT=1     skip apt install step (use if already done)
#   SKIP_TORCH=1   skip PyTorch install
#   SKIP_CUPY=1    skip CuPy install
#   PYTORCH_INDEX  override pip index (default https://pypi.jetson-ai-lab.dev/jp6/cu126)

set -e

# ─────────────────────────────────────────────────────────────────────
# 0. Preflight
# ─────────────────────────────────────────────────────────────────────
echo ""
echo "################################################"
echo "  Orin Nano Super — first-time HIL setup"
echo "  hostname : $(hostname)"
echo "  L4T      : $(cat /etc/nv_tegra_release 2>/dev/null | head -1 | cut -c1-60)"
echo "  Date     : $(date -u +%Y-%m-%dT%H:%M:%SZ)"
echo "################################################"

if [[ "$(uname -m)" != "aarch64" ]]; then
  echo "ERROR: not aarch64 — this script is for the Jetson, not the host." >&2
  exit 1
fi

PYTORCH_INDEX="${PYTORCH_INDEX:-https://pypi.jetson-ai-lab.dev/jp6/cu126}"

# ─────────────────────────────────────────────────────────────────────
# 1. Power mode — Orin Nano Super modes (verified 2026-05-18 on this unit):
#      0 = 15W   (default)
#      1 = 25W
#      2 = MAXN_SUPER  ← what we want for HIL workload
#    Use SUDO_PASS env to drive `sudo -S` non-interactively (script is
#    typically run piped over SSH so there's no TTY to prompt).
# ─────────────────────────────────────────────────────────────────────
# sudo wrapper — uses SUDO_PASS env via `sudo -S` for non-interactive runs
# (typical when piped over SSH). Falls back to plain sudo if SUDO_PASS unset.
SUDO() {
  if [[ -n "${SUDO_PASS:-}" ]]; then
    echo "$SUDO_PASS" | command sudo -S -p '' "$@"
  else
    command sudo "$@"
  fi
}
echo ""
echo "── 1. Power mode → MAXN_SUPER (mode 2) ──"
echo "  before: $(SUDO nvpmodel -q 2>/dev/null | grep -E 'NV Power Mode' | head -1)"
SUDO nvpmodel -m 2
SUDO jetson_clocks
echo "  after : $(SUDO nvpmodel -q 2>/dev/null | grep -E 'NV Power Mode' | head -1)"
echo "  jetson_clocks locked"

# ─────────────────────────────────────────────────────────────────────
# 2. Swap — Orin Nano 8GB needs swap to survive colcon + nvblox link
# ─────────────────────────────────────────────────────────────────────
echo ""
echo "── 2. Swap status ──"
free -h | grep -E "Swap"
swap_mb=$(free -m | awk '/Swap:/ {print $2}')
if [[ "$swap_mb" -lt 4000 ]]; then
  echo "  WARNING: swap < 4 GB. Recommend (cheatsheet §10):"
  echo "    sudo systemctl disable nvzramconfig"
  echo "    sudo fallocate -l 8G /swapfile && sudo chmod 600 /swapfile"
  echo "    sudo mkswap /swapfile && sudo swapon /swapfile"
  echo "    echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab"
  echo "  (skipping auto-config — destructive, do it manually if you want)"
else
  echo "  swap OK (${swap_mb} MB)"
fi

# ─────────────────────────────────────────────────────────────────────
# 3. CUDA paths
# ─────────────────────────────────────────────────────────────────────
echo ""
echo "── 3. CUDA in ~/.bashrc ──"
if ! grep -q "cuda/bin" ~/.bashrc; then
  cat >> ~/.bashrc <<'EOF'

# Added by orin_nano_setup.sh
export PATH=/usr/local/cuda/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH
export CUDA_HOME=/usr/local/cuda
EOF
  echo "  appended CUDA paths"
else
  echo "  already present"
fi
export PATH=/usr/local/cuda/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH
echo "  nvcc: $(nvcc --version 2>/dev/null | tail -1)"

# ─────────────────────────────────────────────────────────────────────
# 4. apt deps (ROS 2 grid_map / nav2 / tf2 / python-build)
# ─────────────────────────────────────────────────────────────────────
if [[ "${SKIP_APT:-0}" != "1" ]]; then
  echo ""
  echo "── 4. apt: ROS 2 deps for trav pipeline + Nav2 ──"
  SUDO apt-get update -qq
  SUDO apt-get install -y --no-install-recommends \
    ros-humble-grid-map \
    ros-humble-grid-map-msgs \
    ros-humble-grid-map-core \
    ros-humble-grid-map-ros \
    ros-humble-grid-map-filters \
    ros-humble-grid-map-cv \
    ros-humble-grid-map-visualization \
    ros-humble-filters \
    ros-humble-nav2-bringup \
    ros-humble-nav2-mppi-controller \
    ros-humble-nav2-smac-planner \
    ros-humble-tf-transformations \
    ros-humble-pointcloud-to-laserscan \
    python3-pip python3-dev \
    libeigen3-dev libpcl-dev \
    > /tmp/orin_apt.log 2>&1 || { echo "apt install failed — see /tmp/orin_apt.log"; tail -20 /tmp/orin_apt.log; exit 1; }
  echo "  apt deps installed (log: /tmp/orin_apt.log)"
else
  echo "── 4. apt deps — SKIPPED (SKIP_APT=1) ──"
fi

# ─────────────────────────────────────────────────────────────────────
# 5. PyTorch for JetPack 6.2 (CUDA 12.6 / aarch64)
# ─────────────────────────────────────────────────────────────────────
if [[ "${SKIP_TORCH:-0}" != "1" ]]; then
  echo ""
  echo "── 5. PyTorch (Jetson wheel) ──"
  # NOTE: PyTorch is OPTIONAL for the HIL stack as of 2026-05-18 — we patched
  # elevation_mapping.py to honor ELEVATION_MAPPING_FORCE_CUPY=1, which uses
  # the pure-cupy CNN backend that ran 2.60 ms / 384 Hz on this unit. Set
  # SKIP_TORCH=1 if pypi.jetson-ai-lab.dev DNS doesn't resolve on your network.
  if python3 -c "import torch; assert torch.cuda.is_available()" 2>/dev/null; then
    echo "  already installed and CUDA-enabled — skipping"
  else
    echo "  installing torch + torchvision from ${PYTORCH_INDEX} ..."
    pip3 install --user --no-cache-dir \
      --index-url "$PYTORCH_INDEX" \
      torch torchvision \
      2>&1 | tail -10 \
      || echo "  WARN: PyTorch install failed — non-blocking, cupy backend will be used"
  fi
else
  echo "── 5. PyTorch — SKIPPED (SKIP_TORCH=1) — cupy backend will be used ──"
fi

# ─────────────────────────────────────────────────────────────────────
# 6. CuPy (CUDA 12.x runtime)
# ─────────────────────────────────────────────────────────────────────
if [[ "${SKIP_CUPY:-0}" != "1" ]]; then
  echo ""
  echo "── 6. CuPy + numpy/scipy pin ──"
  # cupy 14.x strictly requires numpy>=2.0 (verified 2026-05-18 on this unit).
  # But JetPack 6.2's system scipy at /usr/lib/python3/dist-packages was built
  # against numpy 1.x → ABI break ('_ARRAY_API not found') on import. Fix by
  # shadowing system scipy with a numpy-2-compatible scipy in --user site.
  pip3 install --user --no-cache-dir \
    cupy-cuda12x \
    'numpy>=2,<2.6' \
    'scipy>=1.13' \
    2>&1 | tail -5
else
  echo "── 6. CuPy — SKIPPED (SKIP_CUPY=1) ──"
fi

# ─────────────────────────────────────────────────────────────────────
# 7. jetson-stats (jtop dashboard)
# ─────────────────────────────────────────────────────────────────────
echo ""
echo "── 7. jetson-stats ──"
if ! command -v jtop &>/dev/null; then
  SUDO pip3 install -U jetson-stats --break-system-packages 2>&1 | tail -3
  SUDO systemctl restart jtop.service 2>/dev/null || true
  echo "  jtop installed — log out and back in before first use"
else
  echo "  already installed: $(jtop --version 2>&1 | head -1)"
fi

# ─────────────────────────────────────────────────────────────────────
# 8. Smoke test — verify GPU access from Python
# ─────────────────────────────────────────────────────────────────────
echo ""
echo "── 8. Smoke test ──"
python3 - <<'EOF'
import sys
ok = True

try:
    import torch
    print(f"  torch  : {torch.__version__}")
    print(f"  CUDA   : {torch.cuda.is_available()}, device: {torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'NONE'}")
    if not torch.cuda.is_available():
        ok = False
except Exception as e:
    print(f"  torch  : FAIL — {e}")
    ok = False

try:
    import cupy as cp
    n_dev = cp.cuda.runtime.getDeviceCount()
    props = cp.cuda.runtime.getDeviceProperties(0)
    print(f"  cupy   : {cp.__version__}, devices: {n_dev}, name: {props['name'].decode()}")
    # Quick mat-mul to make sure JIT works
    a = cp.random.rand(512, 512).astype(cp.float32)
    b = a @ a
    cp.cuda.Stream.null.synchronize()
    print(f"  cupy mm: OK (shape {b.shape}, dtype {b.dtype})")
except Exception as e:
    print(f"  cupy   : FAIL — {e}")
    ok = False

sys.exit(0 if ok else 1)
EOF

echo ""
echo "################################################"
echo "  Setup done."
echo ""
echo "  Next from desktop:"
echo "    ./scripts/real/deploy_to_orin_nano.sh"
echo ""
echo "  Then Phase 1 of docs/claude/orin_nano_hil_runbook.md"
echo "################################################"
