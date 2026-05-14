# cmu_env Python deps for elevation_mapping_cupy

Added to the existing `cmu_env` micromamba env during Phase 2 of the trav_grid rewrite. None of these are in `pyproject.toml`/`environment.yaml` for the env (it's been built up incrementally over multiple projects); recording here so the env can be rebuilt or audited.

## Added by pip

| Package | Version | Reason |
|---|---|---|
| `cupy-cuda13x` | 14.0.1 | Elevation map GPU kernels. Replaces `cupy-cuda12x`. Either build links system `libnvrtc.so.12` so the cu13 vs cu12 wheel name barely matters; only the NVRTC wheel below matters. |
| `nvidia-cuda-nvrtc-cu12` | 12.9.86 (bumped from 12.6.77) | Blackwell sm_120 support in NVRTC's `--gpu-architecture`. cupy's `_get_max_compute_capability` reads NVRTC version: 12.0–12.7 → max sm_90 (rejects sm_120); 12.8 → sm_120; 12.9+ → sm_121. **torch 2.7.1 pins this at 12.6.77**, ignore pip's dep-conflict warning — torch runs sm_120 via forward-compat PTX JIT regardless. See [feedback_cupy_blackwell_nvrtc.md](../../../.claude/projects/-home-hanszhu-Research-Collab-QRC/memory/feedback_cupy_blackwell_nvrtc.md). |
| `simple-parsing` | latest | `elevation_mapping_cupy/parameter.py` dataclass serialisation. Listed in upstream `requirements.txt`. |
| `ruamel.yaml` | latest | `elevation_mapping_cupy/plugins/plugin_manager.py` plugin yaml loader. Not in upstream requirements.txt; missing from the apt rosdep set. |
| `shapely` | latest | `elevation_mapping_cupy/traversability_polygon.py` polygon clipping. |
| `scikit-learn` | latest | optional plugins (kdtree etc.). Installing prevents downstream import errors. |
| `ros2-numpy` | 0.0.5 | `elevation_mapping_cupy/elevation_mapping_node.py` line 16, `rnp.numpify(PointCloud2)` and `rnp.point_cloud2.get_xyz_points`. The pip wheel pins `numpy==1.24.2` — that constraint is **ignored** (no pip resolve actually enforces it at runtime). Numpy stays at 2.2.5 and ros2_numpy still works for our PointCloud2 usage path. |
| `transforms3d` | latest | `tf_transformations.quaternion_matrix` calls into transforms3d. Not in upstream requirements.txt. |

## Cosmetic warning to ignore — cv_bridge ABI

When `elevation_mapping_node.py` imports `from cv_bridge import CvBridge`, the import line in `/opt/ros/humble/local/lib/python3.10/dist-packages/cv_bridge/__init__.py` prints to stderr:

```
A module that was compiled using NumPy 1.x cannot be run in NumPy 2.2.5...
Traceback ... AttributeError: _ARRAY_API not found
```

The traceback is **swallowed inside the cv_bridge module init** — execution continues. `CvBridge()` instantiates fine. Only `cv_bridge.imgmsg_to_cv2(...)` would crash, and that code path is reached only when **semantic camera input** is enabled in `elevation_mapping.yaml`. We never enable that; our only input is `cloud_registered_body` (Mid-360 PointCloud2 from Point-LIO). So this warning is cosmetic for our pipeline.

If we ever need real cv_bridge functionality, the proper fix is to source-rebuild `cv_bridge` against numpy 2 in cmu_env, or downgrade cmu_env numpy to <2 (which breaks cupy — see the cupy memory file).

## What `numpy<2` would break in cmu_env

Confirmed empirically during Phase 2:
- `cupy-cuda13x 14.0.1` requires `numpy>=2.0,<2.6` (hard import-time check)
- `opencv-python 4.13.0.92`, `opencv-python-headless 4.13.0.92` require `numpy>=2`
- `robocasa 1.0.0` pins `numpy==2.2.5`
- Streamlit + protobuf have unrelated constraint failures (pre-existing)

So `numpy<2` is **not a workable env-wide option** as long as cupy is needed for elevation_mapping.

## Rebuild recipe

```bash
micromamba activate cmu_env
pip install \
  simple-parsing ruamel.yaml shapely scikit-learn transforms3d ros2-numpy
# Blackwell sm_120 — must upgrade NVRTC after any torch reinstall:
pip install "nvidia-cuda-nvrtc-cu12==12.9.86"
# cupy: cu12x or cu13x doesn't matter much (both link system libnvrtc.so.12),
# but cu13x matches the driver bound (580.142 = CUDA 13).
pip uninstall -y cupy-cuda12x
pip install cupy-cuda13x
```

Then sanity-check:
```bash
python3 -c "
import cupy as cp
import cupy.cuda.compiler as cc
print('arch:', cc._get_arch())  # expect 120 on 5090
a = cp.arange(1_000_000, dtype=cp.float32)
print('sum:', float((a*2.0+1.0).sum()))
"
```

Expected: `arch: 120` and a numerical sum (not a `CUDA_ERROR_NO_BINARY_FOR_GPU` crash).
