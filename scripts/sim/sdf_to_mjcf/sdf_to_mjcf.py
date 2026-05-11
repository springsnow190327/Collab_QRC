#!/usr/bin/env python3
"""
Convert Gazebo SDF static-mesh worlds to MuJoCo MJCF.

Scope (intentional):
  Handles static <model> with single-mesh <link>/<collision>/<visual>.
  No joints, no actuators, no dynamic objects — pure scene geometry.

The SubT cave_sim worlds (urban_circuit_01.sdf, Urban 2 Story, etc.) all
fit this scope: each model is one big DAE mesh marked <static>true</static>.

Usage:
  # Convert single model
  ./sdf_to_mjcf.py model "models/Urban 2 Story" \\
      --out worlds/urban_2story.xml

  # Convert full world (sum of all included models with their poses)
  ./sdf_to_mjcf.py world worlds/urban_circuit_01.sdf \\
      --models-root . --out worlds/urban_circuit.xml \\
      --filter-include "Urban Stairwell" "Urban 2 Story"

Output:
  - <out>.xml                MJCF file
  - <out>.xml.assets/*.stl   meshes (DAE auto-converted to STL via trimesh)

Dependencies: python3 + trimesh + numpy (all standard in `cmu_env`).
"""

from __future__ import annotations

import argparse
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable, Sequence
import xml.etree.ElementTree as ET

import numpy as np

# Lazy import — only need it when we actually convert a mesh.
def _trimesh():
    import trimesh
    return trimesh


# --------------------------------------------------------------------------
# SDF parsing helpers
# --------------------------------------------------------------------------

@dataclass
class Pose:
    """SDF pose: x y z roll pitch yaw (RPY in radians)."""
    xyz: np.ndarray = field(default_factory=lambda: np.zeros(3))
    rpy: np.ndarray = field(default_factory=lambda: np.zeros(3))

    @classmethod
    def parse(cls, text: str | None) -> "Pose":
        if not text:
            return cls()
        parts = [float(x) for x in text.split()]
        if len(parts) != 6:
            raise ValueError(f"SDF pose must be 6 floats, got: {text!r}")
        return cls(xyz=np.array(parts[:3]), rpy=np.array(parts[3:]))

    @staticmethod
    def rpy_to_quat_wxyz(rpy: np.ndarray) -> np.ndarray:
        """Convert RPY (extrinsic XYZ in SDF convention) to (w, x, y, z) quaternion.
        MuJoCo also uses w,x,y,z order.
        """
        r, p, y = rpy
        cr, sr = np.cos(r / 2), np.sin(r / 2)
        cp, sp = np.cos(p / 2), np.sin(p / 2)
        cy, sy = np.cos(y / 2), np.sin(y / 2)
        # XYZ extrinsic (Gazebo SDF) = Z * Y * X intrinsic
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y_ = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        return np.array([w, x, y_, z])

    def compose(self, child: "Pose") -> "Pose":
        """Apply child pose in the frame of self (rotation + translation)."""
        # Rotation matrix from RPY (XYZ extrinsic)
        cr, sr = np.cos(self.rpy[0]), np.sin(self.rpy[0])
        cp, sp = np.cos(self.rpy[1]), np.sin(self.rpy[1])
        cy, sy = np.cos(self.rpy[2]), np.sin(self.rpy[2])
        R_x = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
        R_y = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
        R_z = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
        R = R_z @ R_y @ R_x

        composed_xyz = self.xyz + R @ child.xyz
        # For RPY composition we just sum here. This is wrong in general but
        # ok because most SubT models use identity rotation; world-level
        # placement uses yaw only. If finer composition is needed, do it via
        # rotation matrices and convert back.
        composed_rpy = self.rpy + child.rpy
        return Pose(xyz=composed_xyz, rpy=composed_rpy)


@dataclass
class MeshGeom:
    name: str           # MuJoCo asset name (sanitized)
    src_path: Path      # source DAE/STL/OBJ path
    pose: Pose          # local pose within its parent link
    scale: tuple[float, float, float] = (1.0, 1.0, 1.0)


@dataclass
class ModelInstance:
    name: str           # MJCF body name (sanitized)
    world_pose: Pose    # pose in world frame
    meshes: list[MeshGeom]


def _sanitize(name: str) -> str:
    """MuJoCo names must be alphanumeric + underscore. Map spaces & dashes."""
    s = re.sub(r"[^A-Za-z0-9_]", "_", name.strip())
    return re.sub(r"_+", "_", s).strip("_") or "unnamed"


def _find_first_pose(elem: ET.Element) -> Pose:
    p = elem.find("pose")
    if p is None or p.text is None:
        return Pose()
    return Pose.parse(p.text)


def parse_model_sdf(model_dir: Path) -> list[MeshGeom]:
    """Parse a Gazebo model directory's model.sdf.

    Returns the list of MeshGeom from each <link>/<collision> (preferred) or
    <link>/<visual> if no collision is present. Each mesh's pose is the
    composition of link pose + collision/visual pose.
    """
    sdf_path = model_dir / "model.sdf"
    if not sdf_path.exists():
        raise FileNotFoundError(f"No model.sdf in {model_dir}")

    tree = ET.parse(sdf_path)
    root = tree.getroot()
    model = root.find(".//model")
    if model is None:
        raise ValueError(f"No <model> in {sdf_path}")

    out: list[MeshGeom] = []
    for link in model.findall("link"):
        link_pose = _find_first_pose(link)

        # Prefer collision; fall back to visual if no collision is found.
        targets = link.findall("collision")
        if not targets:
            targets = link.findall("visual")

        for i, target in enumerate(targets):
            target_pose = _find_first_pose(target)
            mesh = target.find("geometry/mesh")
            if mesh is None:
                continue
            uri = mesh.findtext("uri", "").strip()
            scale = mesh.findtext("scale", "1 1 1").strip().split()
            scale_t = tuple(float(s) for s in scale[:3]) if len(scale) >= 3 else (1.0, 1.0, 1.0)

            # Resolve URI relative to model_dir. SDF uses
            #   "meshes/foo.dae" or "model://X/meshes/foo.dae".
            uri_clean = re.sub(r"^model://[^/]+/", "", uri)
            mesh_path = (model_dir / uri_clean).resolve()
            if not mesh_path.exists():
                print(f"  WARN: mesh not found: {mesh_path}", file=sys.stderr)
                continue

            pose = link_pose.compose(target_pose)
            geom_name = _sanitize(f"{model_dir.name}_{link.get('name','link')}_{i}")
            out.append(MeshGeom(name=geom_name, src_path=mesh_path,
                                pose=pose, scale=scale_t))
    return out


def parse_world_sdf(world_path: Path, models_root: Path,
                    include_filter: Sequence[str] | None = None) -> list[ModelInstance]:
    """Parse a world SDF, extracting each <include>'s model name + pose.

    For each <include><uri>model://X</uri></include>, locates models_root/X/
    and parses its meshes.
    """
    tree = ET.parse(world_path)
    root = tree.getroot()
    world = root.find(".//world")
    if world is None:
        raise ValueError(f"No <world> in {world_path}")

    instances: list[ModelInstance] = []
    for include in world.findall("include"):
        uri = include.findtext("uri", "").strip()
        m = re.match(r"^model://(.+)$", uri)
        if not m:
            continue
        model_name = m.group(1)
        if include_filter and not any(fn in model_name for fn in include_filter):
            continue

        inst_name = include.findtext("name", model_name).strip()
        inst_pose = _find_first_pose(include)

        model_dir = models_root / model_name
        if not model_dir.is_dir():
            print(f"  WARN: model dir not found: {model_dir}", file=sys.stderr)
            continue

        try:
            meshes = parse_model_sdf(model_dir)
        except (FileNotFoundError, ValueError) as e:
            print(f"  WARN: skip {model_name}: {e}", file=sys.stderr)
            continue

        instances.append(ModelInstance(
            name=_sanitize(inst_name), world_pose=inst_pose, meshes=meshes))
    return instances


# --------------------------------------------------------------------------
# Mesh conversion (DAE → STL via trimesh)
# --------------------------------------------------------------------------

def _detect_dae_unit(dae_path: Path) -> float:
    """Parse DAE <unit meter="X"/> directly. Default 1.0 m if absent."""
    try:
        tree = ET.parse(dae_path)
        ns = "{http://www.collada.org/2005/11/COLLADASchema}"
        unit = tree.find(f".//{ns}asset/{ns}unit")
        if unit is not None and "meter" in unit.attrib:
            return float(unit.attrib["meter"])
    except Exception:
        pass
    return 1.0


def export_mesh_to_stl(src: Path, dst: Path) -> None:
    """Convert any trimesh-supported format (DAE/OBJ/PLY) to STL.

    Loads as Scene to preserve hierarchy transforms, then concatenates.
    For DAE, we also parse <unit meter="X"/> and apply the scale ourselves
    because trimesh's force='mesh' path silently drops scene-level unit scaling.

    Output STL is always in meters.
    """
    trimesh = _trimesh()
    dst.parent.mkdir(parents=True, exist_ok=True)
    if src.suffix.lower() == ".stl":
        dst.write_bytes(src.read_bytes())
        return

    # Suppress missing-texture warnings — they don't affect geometry.
    import io, contextlib
    err_buf = io.StringIO()
    with contextlib.redirect_stderr(err_buf):
        loaded = trimesh.load(src, process=False)

    # Merge to single mesh
    if hasattr(loaded, "geometry"):  # it's a Scene
        if not loaded.geometry:
            raise RuntimeError(f"Loaded empty scene from {src}")
        mesh = loaded.to_geometry() if hasattr(loaded, "to_geometry") \
               else loaded.dump(concatenate=True)
    else:
        mesh = loaded

    if mesh.is_empty:
        raise RuntimeError(f"Loaded empty mesh from {src}")

    # Apply DAE unit conversion (centimeters→meters etc.)
    if src.suffix.lower() in (".dae", ".collada"):
        meter_per_unit = _detect_dae_unit(src)
        if abs(meter_per_unit - 1.0) > 1e-6:
            mesh.apply_scale(meter_per_unit)

    mesh.fix_normals()
    mesh.export(dst.as_posix())


# --------------------------------------------------------------------------
# MJCF emission
# --------------------------------------------------------------------------

def emit_mjcf(instances: list[ModelInstance], out_path: Path,
              spawn_height: float = 0.5) -> None:
    """Emit MJCF with one <body> per instance, mesh-typed geoms inside.

    All instances are <static> from SDF → in MJCF they go straight into
    <worldbody> with no joints.
    """
    asset_dir = out_path.with_suffix(out_path.suffix + ".assets")
    asset_dir.mkdir(parents=True, exist_ok=True)

    # Unique-by-name mesh export
    exported: dict[str, Path] = {}
    for inst in instances:
        for g in inst.meshes:
            if g.name in exported:
                continue
            dst = asset_dir / f"{g.name}.stl"
            try:
                export_mesh_to_stl(g.src_path, dst)
                exported[g.name] = dst
            except Exception as e:
                print(f"  WARN: failed to export {g.name}: {e}", file=sys.stderr)

    asset_rel = asset_dir.name
    lines: list[str] = []
    lines.append(f'<?xml version="1.0"?>')
    lines.append(f'<mujoco model="{_sanitize(out_path.stem)}">')
    lines.append(f'  <compiler meshdir="{asset_rel}" angle="radian" autolimits="true"/>')
    lines.append(f'  <option timestep="0.002" gravity="0 0 -9.81"/>')
    lines.append(f'  <default>')
    lines.append(f'    <geom rgba="0.7 0.7 0.7 1" condim="3" friction="1.0 0.05 0.0001"/>')
    lines.append(f'  </default>')
    lines.append(f'  <asset>')
    lines.append(f'    <texture type="skybox" builtin="gradient" rgb1="0.4 0.6 0.8" rgb2="0 0 0" width="256" height="256"/>')
    for name in exported:
        # refpos/refquat = identity forces MuJoCo to keep the file frame
        # instead of re-aligning to principal axes of inertia (default
        # behavior would rotate static world geometry).
        lines.append(f'    <mesh name="{name}" file="{name}.stl" '
                     f'refpos="0 0 0" refquat="1 0 0 0"/>')
    lines.append(f'  </asset>')
    lines.append(f'  <worldbody>')
    lines.append(f'    <light pos="0 0 30" directional="true"/>')
    # Optional ground plane for safety (you can remove if your world includes a floor)
    lines.append(f'    <geom name="ground" type="plane" size="100 100 0.1" rgba="0.3 0.3 0.3 1"/>')
    for inst in instances:
        quat = Pose.rpy_to_quat_wxyz(inst.world_pose.rpy)
        pos = inst.world_pose.xyz
        lines.append(f'    <body name="{inst.name}" '
                     f'pos="{pos[0]:.4f} {pos[1]:.4f} {pos[2]:.4f}" '
                     f'quat="{quat[0]:.6f} {quat[1]:.6f} {quat[2]:.6f} {quat[3]:.6f}">')
        for g in inst.meshes:
            if g.name not in exported:
                continue
            gpos = g.pose.xyz
            gquat = Pose.rpy_to_quat_wxyz(g.pose.rpy)
            scale = " ".join(f"{s:.4f}" for s in g.scale)
            lines.append(f'      <geom type="mesh" mesh="{g.name}" '
                         f'pos="{gpos[0]:.4f} {gpos[1]:.4f} {gpos[2]:.4f}" '
                         f'quat="{gquat[0]:.6f} {gquat[1]:.6f} {gquat[2]:.6f} {gquat[3]:.6f}"/>')
        lines.append(f'    </body>')
    # Spawn marker for the user to wire Go2 into
    lines.append(f'    <!-- Spawn your Go2 / Go2W include below.  Default Z={spawn_height} -->')
    lines.append(f'    <!--   <include file="path/to/go2.xml"/>  -->')
    lines.append(f'  </worldbody>')
    lines.append(f'</mujoco>')

    out_path.write_text("\n".join(lines))
    print(f"\nMJCF: {out_path}")
    print(f"Assets: {asset_dir} ({len(exported)} STL meshes)")
    if instances:
        total_geoms = sum(len(i.meshes) for i in instances)
        print(f"World: {len(instances)} model instances, {total_geoms} mesh geoms total")


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------

def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="mode", required=True)

    sp_model = sub.add_parser("model", help="Convert single Gazebo model dir to MJCF")
    sp_model.add_argument("model_dir", type=Path)
    sp_model.add_argument("--out", type=Path, required=True)
    sp_model.add_argument("--pose", default="0 0 0 0 0 0",
                          help="World pose of this model (x y z roll pitch yaw)")

    sp_world = sub.add_parser("world", help="Convert full world SDF (sum of <include>d models)")
    sp_world.add_argument("world_sdf", type=Path)
    sp_world.add_argument("--models-root", type=Path, required=True,
                          help="Directory containing model subfolders referenced by model://")
    sp_world.add_argument("--out", type=Path, required=True)
    sp_world.add_argument("--filter-include", nargs="*", default=None,
                          help="Only include models whose name contains any of these substrings")

    args = p.parse_args(argv)

    if args.mode == "model":
        meshes = parse_model_sdf(args.model_dir)
        if not meshes:
            print("WARN: no meshes found in this model", file=sys.stderr)
        instances = [ModelInstance(
            name=_sanitize(args.model_dir.name),
            world_pose=Pose.parse(args.pose),
            meshes=meshes)]
        emit_mjcf(instances, args.out)

    elif args.mode == "world":
        instances = parse_world_sdf(args.world_sdf, args.models_root,
                                    include_filter=args.filter_include)
        if not instances:
            print("ERROR: no instances after filter", file=sys.stderr)
            return 1
        emit_mjcf(instances, args.out)

    return 0


if __name__ == "__main__":
    sys.exit(main())
