#!/usr/bin/env python3
"""Unit tests for exploration benchmark helper scripts.

These tests intentionally avoid ROS runtime dependencies; they validate the
deterministic file-generation and aggregation pieces that are safe to exercise
in CI or from a plain shell.
"""

from __future__ import annotations

import csv
import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
BENCH_DIR = ROOT / "scripts" / "bench"
RUNTIME_DIR = ROOT / "scripts" / "runtime"
if str(BENCH_DIR) not in sys.path:
    sys.path.insert(0, str(BENCH_DIR))
if str(RUNTIME_DIR) not in sys.path:
    sys.path.insert(0, str(RUNTIME_DIR))


class GeneratedMazeTests(unittest.TestCase):
    def test_generation_is_deterministic_and_preserves_robot_bodies(self) -> None:
        from generate_exploration_mazes import generate_maze_scene

        template = ROOT / "src" / "go2w" / "go2_gazebo_sim" / "mujoco" / "demo3_mixed.xml"
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = Path(tmp)
            first = generate_maze_scene(
                template_path=template,
                output_dir=tmp_path,
                variant="rooms",
                seed=101,
            )
            second = generate_maze_scene(
                template_path=template,
                output_dir=tmp_path,
                variant="rooms",
                seed=101,
            )

            xml_text = first.scene_path.read_text()
            self.assertEqual(first.scene_path.read_text(), second.scene_path.read_text())
            self.assertIn('<mujoco model="demo3_mixed_rooms_seed101">', xml_text)
            self.assertIn('<body name="base_link"', xml_text)
            self.assertIn('<body name="b_base_link"', xml_text)
            self.assertIn("generated_rooms_", xml_text)
            self.assertAlmostEqual(first.metadata["scene_area_m2"], 384.0)
            self.assertEqual(first.metadata["seed"], 101)
            self.assertEqual(first.metadata["variant"], "rooms")
            self.assertTrue(first.metadata_path.exists())

    def test_generated_layouts_keep_robot_passable_clearances(self) -> None:
        from generate_exploration_mazes import generate_maze_scene

        template = ROOT / "src" / "go2w" / "go2_gazebo_sim" / "mujoco" / "demo3_mixed.xml"
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = Path(tmp)
            for variant, seed in (("rooms", 101), ("corridors", 202)):
                generated = generate_maze_scene(
                    template_path=template,
                    output_dir=tmp_path,
                    variant=variant,
                    seed=seed,
                )
                xml_text = generated.scene_path.read_text()
                self.assertGreaterEqual(generated.metadata["minimum_gate_width_m"], 1.6)
                self.assertGreaterEqual(generated.metadata["target_corridor_width_m"], 1.8)
                self.assertIn("clean_", xml_text)
                self.assertNotIn("0.9-1.2m doors", xml_text)


class BenchmarkAggregationTests(unittest.TestCase):
    def test_summary_uses_global_csv_and_per_robot_json(self) -> None:
        from aggregate_exploration_benchmark import collect_trial_records, summarise_records

        with tempfile.TemporaryDirectory() as tmp:
            trial_dir = Path(tmp) / "demo3_mixed" / "cfpa2" / "trial_01"
            trial_dir.mkdir(parents=True)
            for ns, area, distance in (
                ("robot_a", 120.0, 31.0),
                ("robot_b", 95.0, 25.0),
            ):
                (trial_dir / f"{ns}.json").write_text(json.dumps({
                    "coverage": {
                        "explored_area_m2": area,
                        "coverage_ratio_of_scene": area / 384.0,
                    },
                    "progress": {"distance_travelled_m": distance},
                    "safety": {"wall_contact_count": 0, "tipped_over": False},
                    "outcome": "completed",
                }))

            with (trial_dir / "metrics.csv").open("w", newline="") as f:
                writer = csv.DictWriter(
                    f,
                    fieldnames=[
                        "t_wall",
                        "global_explored_area_m2",
                        "global_coverage_ratio",
                        "overlap_pct",
                    ],
                )
                writer.writeheader()
                writer.writerow({
                    "t_wall": "600.0",
                    "global_explored_area_m2": "190.0",
                    "global_coverage_ratio": "0.4947916667",
                    "overlap_pct": "13.0",
                })

            records = collect_trial_records(Path(tmp))
            summary = summarise_records(records)
            key = ("demo3_mixed", "cfpa2")
            self.assertIn(key, summary)
            self.assertEqual(summary[key]["trial_count"], 1)
            self.assertAlmostEqual(summary[key]["global_explored_area_m2_mean"], 190.0)
            self.assertAlmostEqual(summary[key]["global_coverage_ratio_mean"], 0.4947916667)
            self.assertAlmostEqual(summary[key]["robot_a_distance_m_mean"], 31.0)
            self.assertAlmostEqual(summary[key]["robot_b_distance_m_mean"], 25.0)


class MTARECommonExecutorTests(unittest.TestCase):
    def test_assigns_distinct_frontiers_from_map_and_robot_positions(self) -> None:
        from mtare_common_executor_core import assign_frontiers, extract_frontier_clusters

        width = 12
        height = 6
        data = [-1] * (width * height)
        for y in range(1, 5):
            for x in range(1, 11):
                data[y * width + x] = 0
        # Create two separated frontier-rich pockets by adding known free
        # islands next to unknown space at opposite ends.
        clusters = extract_frontier_clusters(
            data=data,
            width=width,
            height=height,
            resolution=1.0,
            origin_x=0.0,
            origin_y=0.0,
            min_cluster_size=2,
        )

        assignments = assign_frontiers(
            clusters=clusters,
            robot_positions={"robot_a": (2.0, 2.0), "robot_b": (9.0, 3.0)},
            previous_goals={},
            min_peer_goal_separation=2.0,
        )

        self.assertEqual(set(assignments), {"robot_a", "robot_b"})
        ax, ay = assignments["robot_a"]
        bx, by = assignments["robot_b"]
        self.assertGreater(((ax - bx) ** 2 + (ay - by) ** 2) ** 0.5, 2.0)


class BaselineWrapperTests(unittest.TestCase):
    def test_benchmark_runner_defaults_to_formal_three_planner_matrix(self) -> None:
        runner = ROOT / "scripts" / "bench" / "benchmark_exploration_planners.sh"
        text = runner.read_text()
        self.assertIn('NUM_TRIALS="${NUM_TRIALS:-3}"', text)
        self.assertIn('PLANNERS="${PLANNERS:-cfpa2 gbplanner2 mtare}"', text)
        self.assertIn("Allowed: cfpa2 gbplanner2 mtare", text)
        self.assertIn("gbplanner3 is intentionally excluded", text)

    def test_gbplanner_dual_wrapper_declares_required_topics(self) -> None:
        wrapper = ROOT / "scripts" / "sim" / "gbplanner3_mujoco" / "launch_dual_common_executor.sh"
        compose = ROOT / "scripts" / "sim" / "gbplanner3_mujoco" / "compose" / "docker-compose.collab_qrc_dual.yml"
        text = wrapper.read_text() + "\n" + compose.read_text()
        self.assertIn("/robot_a/command/trajectory", text)
        self.assertIn("/robot_b/command/trajectory", text)
        self.assertIn("ROS_MASTER_URI=http://localhost:11311", text)
        self.assertIn("ROS_MASTER_URI=http://localhost:11312", text)
        self.assertIn("GBPLANNER_VERSION", text)
        self.assertIn("gbplanner2_config.yaml", text)
        self.assertIn("gbplanner_service_path_executor.py", text)
        self.assertIn("service_path", text)


if __name__ == "__main__":
    unittest.main()
