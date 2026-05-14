# Debugging Insight — Traversability-Grid Math Model Critique

**Context.** 2026-05-14, after a day of patching `mapper_node.cpp` to fix wall-leak, ramp-as-obstacle, and noise issues in the 2D `/robot/traversability_grid` published by `nvblox_frontend`. Each individual fix worked locally but the overall map quality stayed poor, with bugs surfacing in new forms each iteration. This document steps back and writes out the actual math model the code is implementing so the architectural deficiencies are visible rather than chasing them through symptom-by-symptom patches.

## TL;DR

The current 2D-projection model has **6 independent mathematical/structural defects**. Three of them (wrong surface definition, naive slope baseline, categorical median) are by themselves enough to produce the symptoms we keep patching. The proposed replacement is a 2.5D elevation map + grid_map traversability filter chain, which is the standard ETH RSL / NTNU pipeline; it produces a **continuous cost** instead of a 3-state OccupancyGrid and integrates `nvblox`'s 3D Bayesian carving correctly.

## Current model (what `publish_traversability` actually computes)

For each 2D cell $(i, j) \in \mathbb{Z}^2$ with world coords $(x_w, y_w)$, define the z-column $C_{ij} = \{(x_w, y_w, z) : z \in [z_{\min}, z_{\max}]\}$ and let $v(x_w, y_w, z) \in \{\text{OCC}, \text{FREE}, \text{UNK}\}$ be the nvblox classification at voxel center $z$ (OCC when log-odds $> \theta_{\text{occ}}$, FREE when $< 0$).

### 1. Surface height (Pass 1)

$$
H[i,j] \;=\; \max\bigl\{ z : v(x_w, y_w, z) = \text{OCC} \bigr\} \quad \text{(NaN if empty)}
$$

### 2. Free-bit bitmap

$$
\text{free\_bits}[i,j] \;=\; \bigl\{ k : v\bigl(x_w, y_w,\, z_{\min} + (k + 0.5)\,v_s\bigr) = \text{FREE} \bigr\}
$$

### 3. Clearance pass (Pass 2)

$$
\text{cls}[i,j] \;=\; \text{OCC} \quad \text{if} \quad \exists\, z \in (H,\, H + h_{\text{clear}}] : v(x_w, y_w, z) = \text{OCC}
$$

### 4. Classify

$$
\text{cls}[i,j] \;=\;
\begin{cases}
0 \;(\text{FREE}) & H \neq \text{NaN},\; \text{not clearance-blocked} \\[2pt]
0 \;(\text{FREE}) & H = \text{NaN},\; k_{\min}(\text{free\_bits}) \cdot v_s \le z_{\text{grounded}} \\[2pt]
-1 \;(\text{UNK}) & \text{otherwise}
\end{cases}
$$

### 5. Slope / step filter

$$
\text{cls}[i,j] \to \text{OCC} \quad \text{if} \quad
\underbrace{\max_{n \in \mathcal{N}_1} |H[n] - H[i,j]| > \tau_{\text{step}}}_{\text{step (adjacent)}}
\;\;\lor\;\;
\underbrace{\max_{n \in \mathcal{N}_{5}} \frac{|H[n] - H[i,j]|}{5 v_s} > \tan \sigma_{\max}}_{\text{slope (5-cell baseline)}}
$$

with $\tau_{\text{step}} = 0.20\text{ m}$, $\sigma_{\max} = 30°$, $v_s = 0.10\text{ m}$.

### 6. Median + flood-fill + persistence

- `cls` median over a $3{\times}3$ window, with OCC preserved (never demoted).
- Blind disk: within a 3 m radius from the robot, propagate FREE into UNK via 4-connected flood-fill seeded from existing FREE.
- Persistence: $\text{cls\_persist}[k] \leftarrow \text{cls}[k]$ whenever $\text{cls}[k] \neq -1$ ("latest non-UNK wins").

---

## The 6 fundamental defects

### ❌ Problem 1: $H = \max\{z : \text{OCC}\}$ is the wrong surface definition

The robot stands on the **lowest stable supporting surface with clearance above it**, not the highest occupied voxel. With $\max$, any overhang/ceiling/light fixture in the same column becomes the "surface" and the entire column is misclassified. Any scene with overhanging structure breaks.

**Correct definition (octomap_server's `projected_map` already uses this):**

$$
H[i,j] \;=\; \min\bigl\{ z : v_{\text{OCC}}(z) \;\land\; \forall z' \in (z,\, z + h_r] : v_{\neg\text{OCC}}(z') \bigr\}
$$

where $h_r$ = robot height + clearance.

### ❌ Problem 2: Slope baseline crosses discontinuities

Computing $|H[i+5] - H[i]| / (5 v_s)$ does not require $H[i+1..i+4]$ to exist as a continuous surface. On a ramp's $y$-edge, the 5-cells-away neighbour is the floor 0.5 m below the ramp surface; the slope reads $1.0 = 45° > \tan 30°$ and the ramp side gets flipped OCC. **This is a cliff, not a slope.** The 1-cell step filter would catch it, but it doesn't reach 5 cells.

**Correct: local-plane fit.** On the $5{\times}5$ window of valid $H$ values, least-squares fit $z = a x + b y + c$:

$$
\nabla H = (a, b), \quad \text{slope} = \arctan \lVert \nabla H \rVert
$$

$$
\text{roughness} = \frac{1}{|W|} \sum_{w \in W} \bigl(H[w] - (a x_w + b y_w + c)\bigr)^2
$$

If the plane residual is large, the surface isn't a plane — fall back to step-only classification rather than reporting a meaningless slope.

### ❌ Problem 3: Categorical median is not a meaningful operator

`cls ∈ {-1, 0, 100}`; sorting and taking the middle element treats UNK and OCC as ordered along the same scale. They aren't — UNK is *uncertainty*, OCC is *known obstacle*. The window $\{-1,-1,0,0,0,0,100,100,100\}$ has median $0$, which is wrong by any semantics.

**Correct: morphological operators on each class separately.**

- **OCC erosion** — drop isolated noise:
  $\text{cls}[i,j] = \text{OCC}$ only when $|\{n \in \mathcal{N}_8 : \text{cls}[n] = \text{OCC}\}| \geq 2$.
- **FREE dilation into UNK only** — fill blind zones without overwriting OCC:
  $\text{cls}[i,j] \leftarrow \text{FREE}$ when $\exists n \in \mathcal{N}_4 : \text{cls}[n] = \text{FREE}$ and current cell is UNK.

### ❌ Problem 4: A 3-state output is throwing information away

The classifier internally has surface height $H$, log-odds variance, slope angle, step magnitude, clearance margin, observation count. We collapse all of it into one of $\{\text{UNK}, \text{FREE}, \text{OCC}\}$. Nav2's planner sees `0` or `100` and cannot express "slightly steep ramp, traverse at reduced speed" or "high-uncertainty region, prefer detour".

**Correct: continuous costmap (uint8, 0–254).** Nav2's `nav2_costmap_2d::Costmap2D` natively supports it:

$$
\text{cost}[i,j] \;=\; \mathbb{1}_{\text{obstacle}} \cdot 254 \;+\; \mathbb{1}_{\text{traversable}} \cdot \text{cost}_{\text{trav}}\bigl(H, \text{slope}, \text{rough}, \sigma_H^2\bigr)
$$

with $\text{cost}_{\text{trav}}$ a smooth function of slope and roughness.

### ❌ Problem 5: Persistence is "last write wins"

```cpp
if (cls[k] != -1) cls_persist_[k] = cls[k];
```

A single noisy frame flips a cell OCC; the next frame flips it FREE; the persistent grid oscillates. **The correct accumulation is log-odds:**

$$
L[i,j]_{t+1} \;=\; L[i,j]_t \;+\; \log \frac{p(z_t \mid \text{occ})}{p(z_t \mid \text{free})}
$$

threshold once for display. This is exactly what nvblox is **already doing in 3D**, and we threw the log-odds away during the 2D projection just to reinvent a worse persistence layer on top.

### ❌ Problem 6: Blind-disk fill is a hack on top of the sensor model

The Mid-360 geometric blind zone is fully determined by sensor pose and FOV (−7° to +52°):

$$
\text{Cell}(i,j) \text{ in blind zone} \iff \text{ray from sensor to } (x_w, y_w, z_{\text{floor}}) \text{ is below } -7° \text{ V-FOV}
$$

We have the sensor pose and FOV; we can mark blind cells analytically. The 3 m disk + flood-fill heuristic is approximating something we could compute exactly.

---

## Proposal — replace, don't keep patching

The pipeline we want is **2.5D elevation mapping with traversability cost**, the standard ETH RSL / NTNU UAS approach (`elevation_mapping_cupy` on ANYmal; `grid_map` filter chain in gbplanner3). Every step is principled and has RSS/ICRA-level work behind it.

Components:

1. **nvblox keeps doing 3D Bayesian carving** — this part is mathematically correct, don't touch it.
2. **`elevation_mapping_cupy`** extracts an elevation grid $H(x,y)$ with per-cell variance $\sigma_H^2(x,y)$.
3. **`grid_map` filter chain** computes slope, roughness, step magnitude → traversability cost via a documented formula.
4. **Publish a `nav2_costmap_2d` costmap** (uint8 0–254) consumed directly by Nav2 and CFPA2, in place of the 3-state OccupancyGrid.

This replaces all 6 ad-hoc fixes with one well-typed pipeline. The infrastructure is already present in `scripts/sim/gbplanner3_mujoco/config/collab_qrc_go2/elevation_mapping_config.yaml` — it just isn't wired into the 3D-exploration launch yet.

### Minimum-cost validation path

Before committing to the full rewrite, two cheap experiments to confirm the diagnosis:

1. **Inside the current `mapper_node`**, fix Problems 1 + 2 only (`H = lowest stable surface`, slope via plane fit). If the ramp and noise issues clear up, that's evidence the model — not the parameters — was the problem.
2. If Step 1 still leaves bad behaviour, swap straight to `grid_map` + `elevation_mapping_cupy` and drop the custom 2D-projection code path entirely.

## Files touched by the current model

- [src/collaborative_exploration/nvblox_frontend/src/mapper_node.cpp](../../src/collaborative_exploration/nvblox_frontend/src/mapper_node.cpp) — all 6 stages live here
- [src/go2w/go2_gazebo_sim/launch/nav_test_3d_explore.launch.py](../../src/go2w/go2_gazebo_sim/launch/nav_test_3d_explore.launch.py) — passes `slope_max_deg`, `step_max_m`, `robot_clearance_m`, `trav_xy_extent_m`
- [docs/claude/3d_frontier_debugging.md](3d_frontier_debugging.md) — the symptom-by-symptom patch history that motivated this rewrite
- [docs/claude/3d_explore_pipeline.md](3d_explore_pipeline.md) — end-to-end pipeline this model lives in (stage 4 of 9)
