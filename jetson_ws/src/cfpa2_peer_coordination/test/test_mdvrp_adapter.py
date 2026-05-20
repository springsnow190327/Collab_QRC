"""Tests for the MDVRP adapter.

These tests run without ROS — they exercise the algorithmic core
of solve_frontier_assignment using plain Python tuples and dicts.

Run with:
    pytest src/collaborative_exploration/cfpa2_peer_coordination/test/test_mdvrp_adapter.py -v

The most critical test in this file is test_determinism_under_shuffled_inputs.
If that fails, the peer protocol is broken: two robots running the same
solver on the same data would produce different assignments and disagree
on what to negotiate.
"""

from __future__ import annotations

import random

import pytest

from cfpa2_peer_coordination.mdvrp_adapter import (
    Point3,
    solve_frontier_assignment,
)

# Degenerate input cases

def test_empty_robot_poses_returns_empty_dict() -> None:
    """With no robots, the adapter has nothing to assign and returns {}."""
    # TODO: call solve_frontier_assignment with empty robot_poses
    # and a non-empty frontier list. Assert result == {}.
    result = solve_frontier_assignment(
        robot_poses = {},
        candidate_frontiers=[(1.0, 0.0, 0.0)],  # just some frontiers to ensure the function doesn't short-circuit on empty input before it gets to the point of returning an empty dict
    )

    assert result == {}

def test_no_frontiers_returns_empty_lists_per_robot() -> None:
    """Robots present but no frontiers: each robot gets an empty list."""
    # TODO: call with two robot poses and an empty frontier list.
    # Assert result has both robot IDs as keys, each mapped to [].
    result = solve_frontier_assignment(
        # poses can be understood as position tuples
        robot_poses = {
            "robot_a": (10.0, 0.0, 0.0),
            "robot_b": (0.0, 0.0, 0.0),
        },
        candidate_frontiers=[],
    )

    assert result == {
        "robot_a": [],
        "robot_b": [],
    }


def test_single_robot_returns_all_frontiers_to_that_robot() -> None:
    """One robot online: it gets every frontier (no negotiation needed)."""
    # TODO: call with exactly one robot and three frontiers.
    # Assert all three frontiers are assigned to that one robot.
    frontiers = [
        (3.0, 0.0, 0.0),
        (1.0, 0.0, 0.0),
        (2.0, 0.0, 0.0),
    ]

    result = solve_frontier_assignment(
        robot_poses = {"robot_a": (0.0, 0.0, 0.0)},
        candidate_frontiers=frontiers,
        min_robot_to_frontier_dist=0.25,
    )

    # The adapter sorts frontiers deterministically before returning them
    assert result == {
        "robot_a" : [
            (1.0, 0.0, 0.0),
            (2.0, 0.0, 0.0),
            (3.0, 0.0, 0.0),
        ]
    }


def test_all_frontiers_filtered_by_distance() -> None:
    """Frontiers all closer than min_dist to a robot are filtered out."""
    # TODO: call with two robots at known positions and frontiers all
    # placed within 0.1m of one of them. Use min_robot_to_frontier_dist=0.25.
    # Assert each robot gets [].
    result = solve_frontier_assignment(
        robot_poses = {
            "robot_a": (0.0, 0.0, 0.0),
            "robot_b": (10.0, 0.0, 0.0),
        },
        candidate_frontiers=[
            (0.05, 0.00, 0.0),    # near robot_a
            (0.10, 0.00, 0.0),    # near robot_a
            (10.05, 0.00, 0.0),   # near robot_b
            (10.10, 0.00, 0.0),   # near robot_b
        ],
        min_robot_to_frontier_dist=0.25,
    )

    assert result == {
        "robot_a": [],
        "robot_b": [],
    }


# Basic correctness/cluster test
def test_two_robots_two_clusters_each_robot_gets_its_cluster() -> None:
    """The fundamental correctness test.

    Two robots placed at well-separated positions, with two clusters of
    frontiers each centred on one robot. The solver should give each
    robot its own cluster — anything else suggests the distance matrix
    or index mapping is wrong.
    """
    # TODO: place robot_a near (0, 0), robot_b near (10, 0).
    # Place 3 frontiers near each robot.
    # Assert each robot's assigned frontiers are the ones near it,
    # not the ones near the other robot. (You can check this by
    # asserting all assigned frontiers for robot_a have x < 5, etc.)
    robot_poses = {
        "robot_a": (0.0, 0.0, 0.0),
        "robot_b": (10.0, 0.0, 0.0),
    }

    frontiers = (
        _make_grid_frontiers(centre=(1.0, -0.5, 0.0), spacing=0.5, count=3) +
        _make_grid_frontiers(centre=(8.0, 0.5, 0.0), spacing=0.5, count=3)
    )

    result = solve_frontier_assignment(
        robot_poses=robot_poses,
        candidate_frontiers=frontiers,
        min_robot_to_frontier_dist=0.25,
    )

    assert set(result.keys()) == {"robot_a", "robot_b"}

    robot_a_xs = [frontier[0] for frontier in result["robot_a"]]
    robot_b_xs = [frontier[0] for frontier in result["robot_b"]]

    assert robot_a_xs, "robot_a received no frontiers"
    assert robot_b_xs, "robot_b received no frontiers"

    assert all(x < 5.0 for x in robot_a_xs), (
        f"robot_a got a frontier on robot_b's side: {result['robot_a']}"
    )
    assert all(x >= 5.0 for x in robot_b_xs), (
        f"robot_b got a frontier on robot_a's side: {result['robot_b']}"
    )

    assigned = result["robot_a"] + result["robot_b"]
    assert len(assigned) == len(frontiers)
    assert set(assigned) == set(frontiers)


# Determinism (the critical test)

def test_determinism_under_shuffled_inputs() -> None:
    """Same logical inputs in different orders MUST produce identical output.

    This is the property the peer protocol depends on. If robot_a constructs
    its dict in one order and robot_b constructs the same dict in another
    order, they must still independently compute the same assignment.

    If this test fails, the rest of the project is on quicksand.
    """
    # TODO:
    # 1. Build a "canonical" input: 2 robots, ~6 frontiers, well-distributed.
    # 2. Call solve_frontier_assignment, save the result.
    # 3. Build the SAME inputs but with:
    #    - robot_poses dict keys in reversed insertion order
    #    - candidate_frontiers list shuffled (use a fixed seed for repeatability)
    # 4. Call solve_frontier_assignment again.
    # 5. Assert the two results are equal.
    #
    # Hint: dict equality and list equality are both straightforward
    # if you ensure the comparison is sane. tuple positions compared
    # directly should work since they're built from the same source data.
    robot_poses = {
        "robot_a": (0.0, 0.0, 0.0),
        "robot_b": (10.0, 0.0, 0.0),
    }

    frontiers = [
        (1.0, -0.5, 0.0),
        (1.5, 0.0, 0.0),
        (2.0, 0.5, 0.0),
        (8.0, -0.5, 0.0),
        (8.5, 0.0, 0.0),
        (9.0, 0.5, 0.0),
    ]

    result_1 = solve_frontier_assignment(
        robot_poses=robot_poses,
        candidate_frontiers=frontiers,
        min_robot_to_frontier_dist=0.25,
    )

    reversed_poses = {
        key: value for key, value in reversed(list(robot_poses.items()))
    }

    shuffled_frontiers = frontiers.copy()
    random.Random(42).shuffle(shuffled_frontiers)

    result_2 = solve_frontier_assignment(
        robot_poses=reversed_poses,
        candidate_frontiers=shuffled_frontiers,
        min_robot_to_frontier_dist=0.25,
    )

    assert result_1 == result_2


# Helpers
def _make_grid_frontiers(
    centre: Point3,
    spacing: float = 1.0,
    count: int = 3,
) -> list[Point3]:
    """Build a small line of frontiers around a centre point."""
    cx, cy, cz = centre
    return [(cx + i * spacing, cy, cz) for i in range(count)]