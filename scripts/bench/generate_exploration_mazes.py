#!/usr/bin/env python3
"""Generate deterministic MJCF maze variants for exploration benchmarks.

The generator keeps the robot, assets, sensors, and actuator sections from
``demo3_mixed.xml`` intact and replaces only the world obstacle layout between
the outer-wall marker and the first robot body.  This preserves robot dynamics
and sensor configuration while giving the benchmark multiple maze topologies.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import random
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable


SCENE_AREA_M2 = 384.0
BOUNDS = {"x_min": 0.0, "x_max": 24.0, "y_min": -8.0, "y_max": 8.0}
WALL_HALF_THICKNESS_M = 0.075
WALL_THICKNESS_M = WALL_HALF_THICKNESS_M * 2.0
MIN_GATE_WIDTH_M = 2.0
TARGET_CORRIDOR_WIDTH_M = 2.2
SPAWN_POSES = {
    "robot_a": {"x": 4.0, "y": 2.0, "yaw": 0.0},
    "robot_b": {"x": 4.0, "y": -6.0, "yaw": 0.0},
}


@dataclass(frozen=True)
class GeneratedScene:
    scene_path: Path
    metadata_path: Path
    metadata: dict


def _box(name: str, x: float, y: float, sx: float, sy: float,
         *, z: float = 0.5, sz: float = 0.5, material: str = "divider_mat") -> str:
    return (
        f'    <geom name="{name}" type="box" pos="{x:.3f} {y:.3f} {z:.3f}"\n'
        f'          size="{sx:.3f} {sy:.3f} {sz:.3f}" material="{material}" class="world"/>'
    )


def _h_wall(name: str, x_min: float, x_max: float, y: float, *, material: str = "divider_mat") -> str:
    if x_max <= x_min:
        raise ValueError(f"invalid horizontal wall '{name}': x_max <= x_min")
    return _box(
        name,
        (x_min + x_max) * 0.5,
        y,
        (x_max - x_min) * 0.5,
        WALL_HALF_THICKNESS_M,
        material=material,
    )


def _v_wall(name: str, x: float, y_min: float, y_max: float, *, material: str = "divider_mat") -> str:
    if y_max <= y_min:
        raise ValueError(f"invalid vertical wall '{name}': y_max <= y_min")
    return _box(
        name,
        x,
        (y_min + y_max) * 0.5,
        WALL_HALF_THICKNESS_M,
        (y_max - y_min) * 0.5,
        material=material,
    )


def _cylinder(name: str, x: float, y: float, radius: float,
              *, height: float = 0.5, material: str = "obstacle_mat") -> str:
    return (
        f'    <geom name="{name}" type="cylinder" pos="{x:.3f} {y:.3f} 0.500"\n'
        f'          size="{radius:.3f} {height:.3f}" material="{material}" class="world"/>'
    )


def _clutter_box(name: str, x: float, y: float, sx: float, sy: float, yaw: float) -> str:
    return (
        f'    <geom name="{name}" type="box" pos="{x:.3f} {y:.3f} 0.300"\n'
        f'          size="{sx:.3f} {sy:.3f} 0.300" euler="0 0 {yaw:.3f}" '
        f'material="obstacle_mat" class="world"/>'
    )


def _outer_walls() -> list[str]:
    return [
        "    <!-- === Generated outer walls (24x16m, x in [0,24], y in [-8,8]) === -->",
        _box("wall_north", 12.0, 8.1, 12.1, 0.1, material="wall_mat"),
        _box("wall_south", 12.0, -8.1, 12.1, 0.1, material="wall_mat"),
        _box("wall_west", -0.1, 0.0, 0.1, 8.1, material="wall_mat"),
        _box("wall_east", 24.1, 0.0, 0.1, 8.1, material="wall_mat"),
    ]


def _central_junction() -> list[str]:
    return [
        "    <!-- Central 4m x 4m junction, matching demo3_mixed connectivity. -->",
        _box("cross_h_w", 5.0, 0.0, 5.0, 0.075),
        _box("cross_h_e", 19.0, 0.0, 5.0, 0.075),
        _box("cross_v_n", 12.0, 5.0, 0.075, 3.0),
        _box("cross_v_s", 12.0, -5.0, 0.075, 3.0),
    ]


def _random_clutter(prefix: str, rng: random.Random, count: int) -> list[str]:
    geoms: list[str] = []
    blocked_spawn = [(4.0, 2.0, 1.4), (4.0, -6.0, 1.4), (12.0, 0.0, 2.2)]
    attempts = 0
    while len(geoms) < count and attempts < count * 40:
        attempts += 1
        x = rng.uniform(1.5, 22.5)
        y = rng.uniform(-6.8, 6.8)
        if any(math.hypot(x - sx, y - sy) < r for sx, sy, r in blocked_spawn):
            continue
        if rng.random() < 0.55:
            geoms.append(_cylinder(f"{prefix}_pillar_{len(geoms) + 1}", x, y, rng.uniform(0.16, 0.24)))
        else:
            geoms.append(_clutter_box(
                f"{prefix}_box_{len(geoms) + 1}",
                x,
                y,
                rng.uniform(0.22, 0.45),
                rng.uniform(0.22, 0.45),
                rng.uniform(-1.0, 1.0),
            ))
    return geoms


def _rooms_variant(seed: int) -> tuple[list[str], dict]:
    _ = seed
    geoms = _outer_walls()
    geoms += [
        "    <!-- Generated clean rooms maze: axis-aligned rooms, gates >= 2.0m. -->",
        _h_wall("generated_rooms_clean_mid_west", 0.8, 9.0, 0.0),
        _h_wall("generated_rooms_clean_mid_east", 15.0, 23.2, 0.0),
        _v_wall("generated_rooms_clean_mid_north", 12.0, 2.8, 7.2),
        _v_wall("generated_rooms_clean_mid_south", 12.0, -7.2, -2.8),
        _h_wall("generated_rooms_clean_nw_partition_left", 0.8, 4.8, 4.4),
        _h_wall("generated_rooms_clean_nw_partition_right", 6.8, 9.0, 4.4),
        _v_wall("generated_rooms_clean_nw_divider_lower", 6.0, 1.0, 3.4),
        _v_wall("generated_rooms_clean_nw_divider_upper", 6.0, 5.4, 7.2),
        _h_wall("generated_rooms_clean_ne_partition_left", 15.0, 17.0, 4.4),
        _h_wall("generated_rooms_clean_ne_partition_right", 19.0, 23.2, 4.4),
        _v_wall("generated_rooms_clean_ne_divider_lower", 18.0, 1.0, 3.4),
        _v_wall("generated_rooms_clean_ne_divider_upper", 18.0, 5.4, 7.2),
        _h_wall("generated_rooms_clean_sw_partition_left", 0.8, 4.8, -4.2),
        _h_wall("generated_rooms_clean_sw_partition_right", 6.8, 9.0, -4.2),
        _v_wall("generated_rooms_clean_sw_divider_lower", 6.0, -7.2, -5.4),
        _v_wall("generated_rooms_clean_sw_divider_upper", 6.0, -3.4, -1.0),
        _h_wall("generated_rooms_clean_se_partition_left", 15.0, 17.0, -4.2),
        _h_wall("generated_rooms_clean_se_partition_right", 19.0, 23.2, -4.2),
        _v_wall("generated_rooms_clean_se_divider_lower", 18.0, -7.2, -5.4),
        _v_wall("generated_rooms_clean_se_divider_upper", 18.0, -3.4, -1.0),
        _cylinder("generated_rooms_clean_pillar_nw", 3.0, 6.2, 0.18),
        _cylinder("generated_rooms_clean_pillar_ne", 21.0, 6.1, 0.18),
        _cylinder("generated_rooms_clean_pillar_sw", 8.3, -6.3, 0.18),
        _cylinder("generated_rooms_clean_pillar_se", 16.0, -2.4, 0.18),
    ]
    meta = {
        "layout_family": "rooms",
        "target_corridor_width_m": TARGET_CORRIDOR_WIDTH_M,
        "minimum_gate_width_m": MIN_GATE_WIDTH_M,
        "wall_thickness_m": WALL_THICKNESS_M,
    }
    return geoms, meta


def _corridors_variant(seed: int) -> tuple[list[str], dict]:
    _ = seed
    geoms = _outer_walls()
    geoms += [
        "    <!-- Generated clean corridor maze: broad switchbacks, gates >= 2.0m. -->",
        _h_wall("generated_corridors_clean_lane_top", 0.8, 18.0, 5.4),
        _h_wall("generated_corridors_clean_lane_upper", 6.0, 23.2, 2.6),
        _h_wall("generated_corridors_clean_lane_middle", 0.8, 18.0, -0.2),
        _h_wall("generated_corridors_clean_lane_lower", 6.0, 23.2, -3.0),
        _h_wall("generated_corridors_clean_lane_bottom", 7.0, 23.2, -5.6),
        _v_wall("generated_corridors_clean_turn_right_upper", 18.0, 5.4, 7.2),
        _v_wall("generated_corridors_clean_turn_left_upper", 6.0, 2.6, 4.4),
        _v_wall("generated_corridors_clean_turn_right_mid", 18.0, -0.2, 1.6),
        _v_wall("generated_corridors_clean_turn_left_lower", 6.0, -3.0, -1.2),
        _v_wall("generated_corridors_clean_turn_right_bottom", 18.0, -5.6, -4.0),
        _cylinder("generated_corridors_clean_pillar_top", 21.0, 6.5, 0.18),
        _cylinder("generated_corridors_clean_pillar_upper", 3.0, 3.7, 0.18),
        _cylinder("generated_corridors_clean_pillar_middle", 21.0, 0.8, 0.18),
        _cylinder("generated_corridors_clean_pillar_lower", 3.0, -4.3, 0.18),
        _cylinder("generated_corridors_clean_pillar_bottom", 20.8, -6.8, 0.18),
    ]
    meta = {
        "layout_family": "corridors",
        "target_corridor_width_m": TARGET_CORRIDOR_WIDTH_M,
        "minimum_gate_width_m": MIN_GATE_WIDTH_M,
        "wall_thickness_m": WALL_THICKNESS_M,
    }
    return geoms, meta


def _geometry_for_variant(variant: str, seed: int) -> tuple[list[str], dict]:
    if variant == "rooms":
        return _rooms_variant(seed)
    if variant == "corridors":
        return _corridors_variant(seed)
    raise ValueError(f"unknown maze variant '{variant}'")


def _estimate_blocked_area_m2(geoms: Iterable[str]) -> float:
    total = 0.0
    for line in geoms:
        if 'type="box"' in line:
            match = re.search(r'size="([0-9.]+) ([0-9.]+) ([0-9.]+)"', line)
            if match:
                sx, sy, _ = (float(match.group(i)) for i in range(1, 4))
                total += 4.0 * sx * sy
        elif 'type="cylinder"' in line:
            match = re.search(r'size="([0-9.]+) ([0-9.]+)"', line)
            if match:
                radius = float(match.group(1))
                total += math.pi * radius * radius
    return total


def _replace_model_name(xml: str, model_name: str) -> str:
    return re.sub(r'<mujoco model="[^"]+">', f'<mujoco model="{model_name}">', xml, count=1)


def _rewrite_meshdir(xml: str, *, template_path: Path, output_dir: Path) -> str:
    """Keep generated MJCFs loadable when they live outside the template dir."""
    asset_dir = template_path.parent / "assets"
    meshdir = os.path.relpath(asset_dir, output_dir).replace(os.sep, "/")
    return re.sub(
        r'(<compiler\b[^>]*\bmeshdir=")[^"]+(")',
        rf"\1{meshdir}\2",
        xml,
        count=1,
    )


def _replace_world_geometry(xml: str, geoms: list[str]) -> str:
    start_marker = "    <!-- === Outer walls"
    end_marker = "    <!-- === Go2W Robot"
    try:
        start = xml.index(start_marker)
        end = xml.index(end_marker)
    except ValueError as exc:
        raise RuntimeError("template does not contain expected demo3_mixed world markers") from exc
    generated_block = "\n".join(geoms) + "\n\n"
    return xml[:start] + generated_block + xml[end:]


def generate_maze_scene(
    *,
    template_path: Path,
    output_dir: Path,
    variant: str,
    seed: int,
) -> GeneratedScene:
    template_path = Path(template_path)
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    model_name = f"demo3_mixed_{variant}_seed{seed}"
    scene_path = output_dir / f"{model_name}.xml"
    metadata_path = output_dir / f"{model_name}.json"

    geoms, variant_meta = _geometry_for_variant(variant, seed)
    xml = template_path.read_text()
    xml = _replace_model_name(xml, model_name)
    xml = _rewrite_meshdir(xml, template_path=template_path, output_dir=output_dir)
    xml = _replace_world_geometry(xml, geoms)
    scene_path.write_text(xml)

    blocked_area = _estimate_blocked_area_m2(geoms)
    metadata = {
        "scene": model_name,
        "variant": variant,
        "seed": seed,
        "template": str(template_path),
        "scene_area_m2": SCENE_AREA_M2,
        "bounds": BOUNDS,
        "spawn_poses": SPAWN_POSES,
        "wall_count": sum('type="box"' in g and "wall_" not in g for g in geoms),
        "obstacle_count": sum("obstacle_mat" in g for g in geoms),
        "blocked_area_estimate_m2": round(blocked_area, 3),
        "free_space_estimate_m2": round(max(0.0, SCENE_AREA_M2 - blocked_area), 3),
        **variant_meta,
    }
    metadata_path.write_text(json.dumps(metadata, indent=2) + "\n")
    return GeneratedScene(scene_path=scene_path, metadata_path=metadata_path, metadata=metadata)


def _parse_variant_spec(spec: str) -> tuple[str, int]:
    if ":" not in spec:
        raise argparse.ArgumentTypeError("variant spec must be '<variant>:<seed>'")
    variant, seed_text = spec.split(":", 1)
    try:
        seed = int(seed_text)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"invalid seed in '{spec}'") from exc
    return variant.strip(), seed


def main() -> int:
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--template",
        type=Path,
        default=root / "src" / "go2w" / "go2_gazebo_sim" / "mujoco" / "demo3_mixed.xml",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=root / "src" / "go2w" / "go2_gazebo_sim" / "mujoco" / "generated",
    )
    parser.add_argument(
        "--variant",
        action="append",
        type=_parse_variant_spec,
        default=[],
        help="Generate one variant as '<rooms|corridors>:<seed>'. Repeatable.",
    )
    args = parser.parse_args()
    variants = args.variant or [("rooms", 101), ("corridors", 202)]

    for variant, seed in variants:
        generated = generate_maze_scene(
            template_path=args.template,
            output_dir=args.output_dir,
            variant=variant,
            seed=seed,
        )
        print(f"{generated.scene_path}  metadata={generated.metadata_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
