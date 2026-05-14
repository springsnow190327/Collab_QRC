# ETH Elevation Mapping — Design Reference

**Purpose.** Before replacing the in-tree 2D projection in `nvblox_frontend/mapper_node.cpp` with `elevation_mapping_cupy` + `grid_map` filter chain (see [plans/2026-05-14-trav-grid-rewrite.md](plans/2026-05-14-trav-grid-rewrite.md)), document the math model and design intent of the upstream pipeline. Source material verified against the canonical papers and the implementations themselves; line-pointers given where they exist.

The reason this matters: after [`trav_grid_math_model_critique.md`](trav_grid_math_model_critique.md) identified 6 mathematical defects in our hand-rolled projection, fixing Problems 1+2 in commit `8d4a7ba` (lowest-stable H + plane-fit slope) was not sufficient — ramp/leak/noise symptoms persisted. That implied the **architecture** (3-state output, last-write-wins persistence, no probabilistic fusion) was the problem, not the per-step formulas. ETH's pipeline is what 6 ad-hoc patches were converging toward, badly. This doc captures it cleanly so the rewrite is principled rather than another patch.

## Upstream sources

- Miki, Wellhausen, Grandia, Jenelten, Homberger, Hutter — *Elevation Mapping for Locomotion and Navigation using GPU*, IROS 2022 ([arXiv:2204.12876](https://arxiv.org/abs/2204.12876)). The GPU rewrite (`elevation_mapping_cupy`).
- Fankhauser, Bloesch, Hutter — *Probabilistic Terrain Mapping for Mobile Robots with Uncertain Localization*, RA-L 2018. Full probabilistic foundation with pose-covariance propagation.
- Fankhauser, Bloesch, Gehring, Hutter, Siegwart — *Robot-Centric Elevation Mapping with Uncertainty Estimates*, CLAWAR 2014. The original robot-centric formulation.
- Fankhauser & Hutter — *A Universal Grid Map Library*, in ROS — The Complete Reference Vol. 1, Springer 2016. The `grid_map` library + filter chain abstraction.
- Reference implementations: [`leggedrobotics/elevation_mapping_cupy`](https://github.com/leggedrobotics/elevation_mapping_cupy), [`ANYbotics/grid_map`](https://github.com/ANYbotics/grid_map), [`leggedrobotics/traversability_estimation`](https://github.com/leggedrobotics/traversability_estimation).

## Stage A — per-cell probabilistic height fusion (`elevation_mapping_cupy`)

Each map cell stores two scalars: estimated height $\hat h$ and variance $\sigma^2$. This is a 1D Kalman filter on height. It is **not** log-odds occupancy — log-odds is the right tool for "is there matter here", while elevation mapping answers "what is the ground height here, and how sure are we". Our nvblox layer still does log-odds carving in 3D; elevation mapping consumes the 3D-carved point cloud and produces the 2.5D height estimate from it.

### A.1 — measurement variance per point

`cupy` GPU kernel `add_points_kernel` (in `custom_kernels.py`):

$$
v_z \;=\; k_{\text{noise}} \cdot z^2
$$

where $z$ is the **range** of the point in sensor frame, and $k_{\text{noise}}$ is `sensor_noise_factor` (default ≈ 0.05). Quadratic in range — far points get more variance.

The 2018 RA-L paper (Fankhauser et al.) gives the full propagation:

$$
\Sigma_p \;=\; J_S \, \Sigma_S \, J_S^\top \;+\; J_R \, \Sigma_R \, J_R^\top \;+\; J_r \, \Sigma_r \, J_r^\top
$$

with $\Sigma_S$ sensor noise, $\Sigma_R$ robot rotation covariance, $\Sigma_r$ robot translation covariance, and $J_*$ the Jacobians transforming a sensor-frame point into the map frame. Only the $z$-component is retained for the 1D Kalman fusion. The cupy implementation collapses this to the quadratic-in-range form on the assumption that the legged-robot state estimator handles the rotation/translation noise upstream — for us on Point-LIO this assumption is roughly justified; if SLAM drift is significant we can plug the full $\Sigma_R, \Sigma_r$ back in later.

### A.2 — Kalman fusion update

Verbatim from the `add_points_kernel` CUDA source (variables renamed for clarity):

$$
\hat h^+ \;=\; \frac{\sigma_p^2 \cdot \hat h^- \;+\; \sigma^{2-} \cdot z}{\sigma^{2-} + \sigma_p^2}
\qquad
\sigma^{2+} \;=\; \frac{\sigma_p^2 \cdot \sigma^{2-}}{\sigma_p^2 + \sigma^{2-}}
$$

This is the standard 1D Kalman update: posterior mean is a precision-weighted average of prior $\hat h^-$ and measurement $z$; posterior variance is the harmonic mean of the two variances (always smaller than either). A long sequence of low-variance measurements drives $\sigma^2 \to 0$ and locks the cell; a single noisy measurement barely moves the mean.

### A.3 — Mahalanobis outlier gate

```cuda
if (abs(map_h - z) > map_v * mahalanobis_thresh) {
    atomicAdd(&map[get_map_idx(idx, 1)], outlier_variance);
}
```

If the new measurement disagrees with the prior by more than $\tau_{\text{mahal}} \cdot \sigma^{2-}$, do **not** fuse — instead inflate the cell variance by `outlier_variance`. Effect: a single outlier doesn't poison $\hat h$, but it weakens the cell's confidence so subsequent measurements can recover. This is the principled replacement for our `cls_persist_` last-write-wins logic.

### A.4 — motion / drift update

When the robot pose increment $\Delta x$ arrives, the cupy implementation rigid-translates the map array on GPU (`MoveMap`) and inflates the variance of every cell by a drift term $\Sigma_{\Delta x}$. Cells that have not been observed for a while accumulate variance and become "stale" — controlled by `max_variance` (the upper clamp on $\sigma^2$). Beyond `max_variance` they are effectively treated as unknown again.

Robot-centric (vs. world-fixed) is a deliberate design choice from Fankhauser 2014: it acknowledges that lidar SLAM drifts, and that maintaining a globally consistent map across a 100 m traverse is impossible without loop closure. Locally accurate around the robot is what locomotion control needs; far-away cells can drift without affecting the next foothold.

Our current code uses a world-fixed grid; we should preserve that for compatibility with Nav2 StaticLayer and CFPA2 BFS but accept the limitation that long-range mapping will drift with SLAM error.

## Stage B — filter chain (`grid_map` + `traversability_estimation`)

After the elevation map publishes `elevation` (and `variance`) layers, `grid_map_filters` runs a configurable pipeline. The canonical chain (from `filters_demo_filter_chain.yaml`):

```
elevation
  → InpaintFilter            → elevation_inpainted    # fill holes
  → MeanInRadiusFilter       → elevation_smooth       # low-pass
  → NormalVectorsFilter      → normal_vectors_{x,y,z} # surface normals
  → MathExpression: acos(nz) → slope
  → MathExpression: |e - es| → roughness
  → MathExpression: weighted → traversability  ∈ [0,1]
```

Then a separate `StepFilter` runs on `elevation` to detect cliffs/walls. The four filters are described below.

### B.1 — NormalVectorsFilter (PCA-based plane fit)

The most important single difference vs. our current code. For each cell $(i,j)$, collect 3D points $\{(x_k, y_k, H[k])\}$ in a circular world-radius window. Compute the covariance matrix

$$
C \;=\; \frac{1}{N}\sum_k p_k p_k^\top \;-\; \bar p \, \bar p^\top
$$

Eigendecompose $C = \sum_i \lambda_i \mathbf{e}_i \mathbf{e}_i^\top$ with $\lambda_1 \le \lambda_2 \le \lambda_3$. The **surface normal at the cell is $\mathbf{n} = \mathbf{e}_1$** — the eigenvector of the smallest eigenvalue. Flip it so $n_z > 0$.

Degeneracy detection comes for free: when the points are nearly co-linear (an edge / cliff / wall), $\lambda_2 / \lambda_3 \to 0$ → there is no well-defined plane normal → the filter falls back to $\mathbf{n} = \hat z$. This is the right answer: "we don't know the slope here, defer to other filters" — and `StepFilter` will pick up the cliff via height discontinuity.

This is fundamentally the right thing because our Problem 2 (slope baseline crossing discontinuities) is exactly the failure mode it avoids by construction.

Implementation: `grid_map_filters/src/NormalVectorsFilter.cpp`. Two algorithms — `area` (the PCA above) and `raster` (3×3 finite-difference gradient with z-up). Use `area` for terrain because `raster` re-introduces the baseline-crossing issue.

### B.2 — SlopeFilter

`traversability_estimation_filters/src/SlopeFilter.cpp`:

$$
\text{slope}(i,j) \;=\; \arccos(n_z(i,j))
$$

$$
\text{score}_{\text{slope}}(i,j) \;=\;
\begin{cases}
1 \;-\; \dfrac{\text{slope}}{\sigma_{\text{crit}}} & \text{slope} < \sigma_{\text{crit}} \\
0 & \text{slope} \ge \sigma_{\text{crit}}
\end{cases}
$$

with $\sigma_{\text{crit}} = \pi/4$ (45°) by default. Linear soft cost — 25° ramp gets score ≈ 0.44, not 0 or 1. Nav2 can use this gradient information to *prefer* flatter routes without categorically forbidding moderate slopes.

### B.3 — RoughnessFilter

`traversability_estimation_filters/src/RoughnessFilter.cpp` — RMS of plane residuals:

$$
\text{roughness}(i,j) \;=\; \sqrt{\frac{1}{N-1}\sum_k d_k^2}
\qquad
d_k = (p_k - \bar p) \cdot \mathbf{n}
$$

$d_k$ is the signed perpendicular distance from cell $k$ to the fitted plane. Score $= 1 - \text{roughness}/r_{\text{crit}}$ clamped to $[0,1]$, $r_{\text{crit}} \approx 0.05$ m (default 0.3 in upstream — looser).

The same PCA from NormalVectorsFilter gives both slope and roughness — zero additional cost. A 25° clean ramp has slope = 0.44 rad and roughness ≈ 0 m → score (slope) = 0.44, score (roughness) ≈ 1.0; the robot can traverse it with moderate cost. A flat but rocky patch has slope ≈ 0 but roughness ≈ 0.05 m → high cost via roughness.

### B.4 — StepFilter

`traversability_estimation_filters/src/StepFilter.cpp` uses a two-window algorithm:

- **inner window** (radius $r_1$, e.g. 0.10 m): $\text{step}_1(i,j) = \max_W H - \min_W H$
- **outer window** (radius $r_2$, e.g. 0.20 m): count cells $n$ where $\text{step}_1 > s_{\text{crit}}$
- score: $\text{score}_{\text{step}} = 1 - \min\!\bigl(s_{\text{crit}},\, \tfrac{n}{n_{\text{crit}}} s_{\text{crit}}\bigr) / s_{\text{crit}}$

with $s_{\text{crit}} \approx 0.15$ m. Intent: a single high-difference cell shouldn't veto — the outer-window count requires the discontinuity to extend over multiple cells. This filters speckle noise but still hard-vetos walls.

### B.5 — combined cost

The demo chain combines slope and roughness linearly:

$$
T(i,j) \;=\; 0.5\!\left(1 - \tfrac{\text{slope}}{0.6}\right) \;+\; 0.5\!\left(1 - \tfrac{\text{roughness}}{0.1}\right)
$$

then clamps to $[0,1]$. Step is typically a separate hard-veto layer that overwrites $T$ to 0 if exceeded.

A continuous $T \in [0, 254]$ goes to Nav2 directly via `Costmap2D`; planners can use the gradient for cost-aware A*/MPPI. We currently throw all of this away by collapsing to 3-state `{UNK, FREE, OCC}`.

## How this discriminates wall / cliff / ramp

The geometry/filter table:

| Geometry            | slope                                | roughness | step      | Verdict                                  |
|---------------------|--------------------------------------|-----------|-----------|------------------------------------------|
| Flat floor          | ≈0                                   | ≈0        | 0         | $T \approx 1$, freely traversable        |
| 25° ramp            | 0.44 rad (linear cost 0.27)          | ≈0        | tiny      | $T \approx 0.64$, traversable w/ cost    |
| 45° steep ramp      | 0.79 rad                             | ≈0        | tiny      | $T = 0$, not traversable                 |
| Vertical wall       | π/2 (cost 1.0)                       | ≈0        | large     | step veto, $T = 0$                       |
| Cliff edge          | PCA $\lambda_2/\lambda_3 \to 0$ → $\mathbf{n} = \hat z$ → slope undefined | — | large     | step veto, $T = 0$, slope abstains       |
| Rocky patch         | ≈0                                   | high      | medium    | roughness cost dominates                 |
| Single noise pulse  | ≈0                                   | small     | inner large but outer $n < n_{\text{crit}}$ | passes, no veto |

The wall-vs-ramp distinction is **carried by step**, not slope. A wall and a vertical cliff look the same to slope (π/2 or undefined) but they look the same way to step (large) — and step is the right signal for both. A clean ramp has slope ≠ 0 but step ≈ one-cell rise, so step does not fire. This is exactly the discrimination our 5-cell baseline slope was botching.

## Contrast with current `mapper_node.cpp`

| Concern               | Current                            | ETH pipeline                                  | Impact of the gap                                                |
|-----------------------|------------------------------------|-----------------------------------------------|------------------------------------------------------------------|
| Height representation | $H = \max\{z:\text{OCC}\}$         | Kalman-fused $\hat h$ + $\sigma^2$            | No outlier rejection; overhangs replace ground in $H$            |
| Slope                 | 5-cell finite difference           | PCA over circular window                      | Baseline crosses discontinuities → ramp edges → false OCC        |
| Roughness             | not computed                       | RMS plane residual                            | Can't grade "rocky but traversable" vs "smooth and steep"        |
| Step                  | 1-cell max diff                    | two-window with $n_{\text{crit}}$             | Single-cell noise produces spurious step vetoes                  |
| Output                | 3-state $\{-1, 0, 100\}$           | continuous $[0, 254]$                         | Nav2 / CFPA2 lose gradient info; no preference for easier terrain |
| Persistence           | last-write-wins on `cls`           | Kalman accumulation w/ Mahalanobis gate       | Single noisy frame flips a cell forever                          |
| Sensor blind zone     | 3 m flood-fill hack                | analytic by FOV (computed per cell from pose) | Heuristic; over- or under-fills depending on robot pose          |
| Pose model            | world-fixed grid                   | robot-centric w/ drift inflation              | Long-range drift not modeled; we accept this for Nav2 compat     |

The six in-tree patches are independently each correct attacks on one of these gaps. The reason fixing two of them was not enough: **the gaps are coupled.** A clean $H$ from Problem 1 still feeds a 5-cell baseline slope that can cross a fixed-but-now-correct $H$ discontinuity in Problem 2; the corrected slope still feeds a 3-state output that throws away the soft cost — so Nav2 sees the same 0/100 it always did, just with slightly different boundaries. Roughness was never wired in, so a noisy-but-flat patch still confuses the categorical output. Without a probabilistic update, transient noise re-injects through `cls_persist_` every frame.

Replacing the whole stage with `elevation_mapping_cupy + grid_map_filters` addresses every gap at once with one composed pipeline, and inherits ~10 years of ETH RSL field-hardening on legged robots.

## Robot-centric vs world-fixed — a design tension we must resolve

Fankhauser's 2014 paper makes a strong case for robot-centric mapping: lidar SLAM drifts, so the only metrically reliable region of a map is the local neighborhood. Cells far behind the robot accumulate drift faster than they accumulate new measurements and should be allowed to become unknown again.

Our consumers (Nav2 StaticLayer, CFPA2 frontier BFS) assume a world-fixed map. The cupy node can publish either; we'll use its world-fixed mode for compatibility and accept that on long traverses the far-end of the map will drift. Once we move to gbplanner3-style local-planner-only navigation, robot-centric becomes more natural.

## Implementation notes for the rewrite

1. `elevation_mapping_cupy` ROS 2 branch exists ([leggedrobotics/elevation_mapping_cupy#ros2](https://github.com/leggedrobotics/elevation_mapping_cupy)) — vendor it under `src/vendor/` rather than relying on apt, since the upstream still lists it as experimental.
2. The CuPy GPU kernels need CUDA 12.x (we have 12.6 from nvblox). Verify cupy installs into `cmu_env` cleanly on the 5090 (Blackwell sm_120) — this is the highest-risk bring-up step; Phase 2 of the plan has a bail-out gate.
3. Use the `area` algorithm in `NormalVectorsFilter` with radius ≈ 0.25 m (covers ~5×5 cells at 0.10 m resolution). Smaller windows under-sample, larger windows over-smooth the ramp edge.
4. Outlier `mahalanobis_thresh` = 2.5 σ is a good starting point; lower it (1.5–2.0) if dynamic Mid-360 returns from the simulated robot legs cause persistent variance.
5. Subscribe `elevation_mapping_cupy` to `/robot/cloud_registered_body` (Mid-360 deskewed by Point-LIO into base_link). The node will TF to `map`. **Do not** feed raw `/livox/lidar` — it's not deskewed.
6. Keep nvblox 3D Bayesian carving running alongside; it produces the `voxels_3d` topic CFPA2's 3D frontier extractor depends on (the `extract_3d_frontiers` z-band filtered cluster + farthest-frontier-voxel centroid logic is independent of the 2D projection and should stay).
7. Use the `grid_map_to_occupancy_grid` adapter (Phase 5 of the plan) to publish the same `/robot/traversability_grid` topic on the same TRANSIENT_LOCAL QoS that `mapper_node` did — Nav2 StaticLayer and CFPA2 then need zero changes.

## Open questions for the rewrite

- **CFPA2 frontier extraction inputs.** CFPA2's 3D frontier extractor reads `voxels_3d` (from nvblox), not the 2D projection. Confirmed independent. But CFPA2's *reachability check* (`_distance_transform`) reads the 2D `/robot/traversability_grid` — switching to the new pipeline changes what "reachable" means. The continuous-cost layer threshold (free / occ) controls this; pick a starting `occ_thresh` that approximately matches the current 3-state semantics, then tune.
- **Frame coupling.** elevation_mapping_cupy expects a `map → base_link` chain. Our setup publishes that via `fast_lio_tf_adapter` (single ownership rule #16 in CLAUDE.md). Make sure the elevation_mapping node's TF buffer subscribes to the namespaced `/robot/tf` (not global `/tf`) — same rule #10 trap.
- **GPU contention.** nvblox already uses the GPU. cupy elevation_mapping will too. Need a quick `nvidia-smi dmon` check that combined utilization stays under 100% and we don't get kernel-launch stalls.

## See also

- [`trav_grid_math_model_critique.md`](trav_grid_math_model_critique.md) — the 6-defect critique that motivated this rewrite
- [`plans/2026-05-14-trav-grid-rewrite.md`](plans/2026-05-14-trav-grid-rewrite.md) — the actual execution plan
- [`3d_frontier_debugging.md`](3d_frontier_debugging.md) — the day-long debug history that led to the critique
- [`3d_explore_pipeline.md`](3d_explore_pipeline.md) — end-to-end pipeline doc the rewritten Stage 4 plugs into
