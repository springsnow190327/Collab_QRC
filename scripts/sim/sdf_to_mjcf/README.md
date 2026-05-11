# SDF → MJCF converter for SubT cave_sim worlds

Convert Gazebo SDF static-mesh worlds (from `ntnu-arl/subt_cave_sim`) to
MuJoCo MJCF so we can run gbplanner3 / nav stack tests in Collab_QRC's
existing MuJoCo pipeline instead of standing up Gazebo Harmonic.

## Scope (intentional)

- ✅ Static `<model>` with one or more single-mesh `<link>` elements (the
  entire SubT cave_sim corpus fits this — every model is `<static>true</static>`
  with one DAE mesh)
- ✅ Composing a `<world>.sdf` from multiple `<include>` model instances
  (e.g. `urban_circuit_01.sdf`)
- ✅ Auto-converts DAE → STL via trimesh, including DAE `<unit>` scaling
- ❌ Joints, actuators, sensors, articulated bodies (not present in cave_sim)
- ❌ PBR materials / textures (we only need collision geometry for nav)

## Quick test (validated on Collab_QRC laptop, 2026-05-11)

```bash
SUBT=~/Research/uas_deploy/unified_autonomy_stack/workspaces/ws_sim/src/subt_cave_sim

# Single model: a 2-story industrial warehouse with internal stairs
micromamba run -n cmu_env python3 ./sdf_to_mjcf.py model \
  "$SUBT/models/Urban 2 Story" \
  --out out/urban_2story.xml

# Single model: just a stairwell (cleaner stair-climb baseline)
micromamba run -n cmu_env python3 ./sdf_to_mjcf.py model \
  "$SUBT/models/Urban Stairwell Platform Centered" \
  --out out/stairwell.xml

# Single model: vertical shaft (33 m of pure vertical exploration)
micromamba run -n cmu_env python3 ./sdf_to_mjcf.py model \
  "$SUBT/models/Cave Vertical Shaft" \
  --out out/vertical_shaft.xml

# Full world (multi-include): urban circuit, restricted to indoor multi-story bits
micromamba run -n cmu_env python3 ./sdf_to_mjcf.py world \
  "$SUBT/worlds/urban_circuit_01.sdf" \
  --models-root "$SUBT/models" \
  --filter-include "Urban 2 Story" "Urban Stairwell" "Urban Service Room" \
  --out out/urban_indoor.xml
```

## Test results (all 3 single-model conversions)

| Model | extent (m) | z range | Tris | STL size |
|---|---|---|---|---|
| Urban 2 Story | 40 × 40 × 21 | -1.36 → +20.0 | 117k | 5.7 MB |
| Urban Stairwell Platform | 39 × 40 × 18 | +0.93 → +19.1 | 17.5k | 857 KB |
| Cave Vertical Shaft | 33 × 29 × 34 | -0.7 → +33.0 | (varies) | 856 KB |

All three load cleanly in MuJoCo 3.6.0:
```python
import mujoco
m = mujoco.MjModel.from_xml_path('out/urban_2story.xml')
# ✓ no errors, no scale warnings, no NaN inertia
```

## Wiring into Collab_QRC's existing MuJoCo path

1. Convert the model (above).
2. Move the MJCF + asset dir into the worlds tree:
   ```bash
   cp -r out/urban_2story.xml* \
     /home/hanszhu/Research/Collab_QRC/src/go2w/go2_gazebo_sim/worlds/
   ```
3. Wrap with Go2 spawn — write a small parent MJCF that includes the world
   and adds the Go2 robot at a known floor-level spawn pose:
   ```xml
   <mujoco model="urban_2story_with_go2">
     <include file="urban_2story.xml"/>
     <worldbody>
       <body name="go2_base" pos="0 -15 -0.9">
         <include file="../models/go2.xml"/>
       </body>
     </worldbody>
   </mujoco>
   ```
   (Adjust spawn pose based on where the floor is inside this specific
   building — for Urban 2 Story, floor is at z ≈ -1.36, so robot base
   ~ -0.9 is good for Go2's standing height ~ 0.45m.)
4. Add a launch arg:
   ```bash
   ./scripts/launch/nav_test_go2.sh world:=urban_2story
   ```

## How it works

```
sdf_to_mjcf.py model <dir>      sdf_to_mjcf.py world <file>
       │                              │
       ▼                              ▼
parse_model_sdf()             parse_world_sdf()
  ├─ <pose> → Pose             ├─ each <include>:
  ├─ <link><collision|visual>  │    name + pose + uri="model://X"
  │    <mesh><uri>             │    → parse_model_sdf(models_root/X)
  └─ returns: list[MeshGeom]   └─ returns: list[ModelInstance]
       │                              │
       └──────────┬───────────────────┘
                  ▼
          emit_mjcf()
            │
            ├─ export_mesh_to_stl() per unique mesh
            │    ├─ trimesh.load(scene mode, preserves hierarchy)
            │    ├─ _detect_dae_unit() ← parses <unit meter="0.01"/>
            │    ├─ apply_scale(meter_per_unit)  ← cm → m
            │    └─ export as STL
            └─ write MJCF XML with:
                 <asset><mesh name=".." file=".." refpos=0 refquat=identity/>
                 <worldbody>
                   <body pos=.. quat=..>
                     <geom type="mesh" mesh=".."/>
                   </body>
```

## Known quirks

1. **DAE `<unit>`** — SubT DAE files have `<unit meter="0.010000" name="centimeter"/>`.
   trimesh's `force='mesh'` silently drops this unit conversion. We
   work around by parsing it manually and calling `apply_scale()` ourselves.

2. **MuJoCo internal mesh re-framing** — When you read `m.mesh_vert` from a
   loaded model, MuJoCo has already centered the mesh on its center-of-mass
   and aligned principal axes. The **world-frame** bbox of the mesh is
   recovered by applying the geom's `pos` + `quat` to those internal verts.
   Adding `refpos="0 0 0" refquat="1 0 0 0"` to the mesh asset is a no-op
   for storage but documents the intent.

3. **Texture warnings** — DAE files reference texture .jpg files that
   sometimes don't exist alongside the mesh. We swallow these warnings
   because they don't affect collision geometry. If you want textured
   visuals, you'll need to pull the texture files separately.

4. **Pose composition is approximate for non-trivial RPY** — `Pose.compose`
   sums RPY rather than doing full rotation-matrix composition. This is
   correct for the common SubT case (mostly identity rotations + yaw-only
   world placement) but would be wrong for arbitrary stacked rotations.
   Fix when you encounter a model that breaks this assumption.

5. **`world` mode is untested at scale** — Single-model conversion is
   solid. World-level conversion (multi-include with poses) is implemented
   but I haven't run it against a full `urban_circuit_01.sdf` yet
   (~300 model includes, will take minutes). Smoke-test with a filter first.

## Other SubT cave_sim worlds worth converting

```
worlds/darpa_cave_01.sdf       — DARPA cave circuit (sloped tunnels, no stairs)
worlds/darpa_cave_02.world     — variant
worlds/darpa_cave_03.world     — variant
worlds/niosh_osrf.world        — NIOSH safety research mine reconstruction
worlds/pittsburgh_mine.sdf     — Pittsburgh mine (real-data, ~10MB DAE)
worlds/urban_circuit_01.sdf    — Urban circuit (multi-floor + stairs) ← test 3D
```

For gbplanner3 validation against the **same world NTNU used**, focus on
`urban_circuit_01.sdf` (multi-floor) and `darpa_cave_02.world` (rugged
3D terrain).

## License

Outputs are derivative of:
- subt_cave_sim © Open Robotics, Apache-2.0
- Collab_QRC → Collab_QRC license
The converter itself (this directory) is fine to relicense as part of Collab_QRC.
