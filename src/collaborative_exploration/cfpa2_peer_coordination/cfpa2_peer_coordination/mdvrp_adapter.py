"""Pure-Python wrapper around upstream MDVRP solver

This module provides a deterministic ROS-free interface to ``cfpa2_collaborative_autonomy.mdvrp_solver`` for use by the peer coordinator negotiation logic

Design contract:
  - All inputs are plain Python tuples and dicts (no ROS messages)
  - Robot ordering is deterministic: sorted alphabetically by robot ID. Both peers in a negotiation MUST therefore agree on the assignment independently, given the same inputs
  - Degenerate inputs return sensible defaults rather than raising: see docstring on ``solve_frontier_assignment`` for cases

Implementation notes:
    build_mdvrp_distance_matrix expects:
    [cells..., depots...]

    solve_mdvrp returns:
    route indices where each index is a cell/frontier index, not depot index

    scale default:
    100.0, not 1000.0
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

from cfpa2_collaborative_autonomy.mdvrp_solver import (
    build_mdvrp_distance_matrix,
    solve_mdvrp,
)

# Type aliases used throughout the module
Point3 = tuple[float, float, float]
Assignment = dict[str, list[Point3]]  # robot ID -> list of frontier points

# Distance scaling: the upstream solver requires integer distances, so metres are multiplied by this factor before rounding. Matches what upstream coordinator uses

DISTANCE_SCALE = 100.0

def solve_frontier_assignment(
    *,  
    robot_poses: dict[str, Point3],
    candidate_frontiers: list[Point3],
    min_robot_to_frontier_dist: float = 0.25,   # distance in metres
    time_limit_sec: float = 0.5,
    span_cost_coefficient: int = 100,
) -> Assignment:
    """Assign candidate frontiers to robots via the upstream MDVRP solver
    Args:
        robot_poses: Mapping of robot ID to (x, y, yaw_or_z) pose.
            Robot IDs are arbitrary strings; ordering is determined
            internally by sorted(robot_poses.keys()) for determinism.
        candidate_frontiers: List of (x, y, z) candidate frontier
            positions that may be assigned.
        min_robot_to_frontier_dist: Frontiers closer than this to
            ANY robot are filtered out before solving. Prevents
            "go to your current pose" assignments during stale
            frontier moments.
        time_limit_sec: Forwarded to the upstream solver.
        span_cost_coefficient: Forwarded to the upstream solver.
            Higher = more aggressive load balancing across robots.

    Returns:
        Mapping of robot ID to ordered list of assigned frontier
        positions. The ordering within each list is the visit order
        proposed by the solver.

    Degenerate cases (handled permissively, no exceptions raised):
        - Empty robot_poses: returns {}
        - Single robot: returns {only_robot_id: filtered_frontiers}
        - Empty candidate_frontiers: returns {robot_id: [] for each robot}
        - All frontiers filtered out by distance: same as empty frontiers
    """
    if not robot_poses:
        return {}

    robot_ids_sorted = sorted(robot_poses.keys())
    robot_positions = [robot_poses[robot_id] for robot_id in robot_ids_sorted]
    assignments: Assignment = {robot_id: [] for robot_id in robot_ids_sorted}

    # Sort frontiers deterministically before filtering/solving
    frontiers_sorted = sorted(candidate_frontiers, key=lambda p: (p[0], p[1], p[2]))

    filtered_frontiers = _filter_nearby_frontiers(
        frontiers=frontiers_sorted,
        robot_positions=robot_positions,
        min_dist=min_robot_to_frontier_dist,
    )

    if not filtered_frontiers:
        return assignments

    if len(robot_ids_sorted) == 1:
        # Single robot, no need to solve optimisation problem
        only_robot_id = robot_ids_sorted[0]
        assignments[only_robot_id] = filtered_frontiers
        return assignments

    distance_matrix = _build_distance_matrix(
        robot_positions=robot_positions,
        frontier_positions=filtered_frontiers,
    )

    routes = solve_mdvrp(
        exploring_cell_positions=filtered_frontiers,
        robot_positions=robot_positions,
        distance_matrix=distance_matrix,
        time_limit_sec=time_limit_sec,
        span_cost_coefficient=span_cost_coefficient,
    )

    return _route_indices_to_positions(
        routes=routes,
        robot_ids_sorted=robot_ids_sorted,
        frontier_positions=filtered_frontiers,
    )

def _filter_nearby_frontiers(
    frontiers: list[Point3],
    robot_positions: list[Point3],
    min_dist: float,
) -> list[Point3]:
    """Return frontiers whose distance to every robot exceeds min_dist

    Uses 2D Euclidean distance (x, y only); the third tuple element
    is treated as yaw or z and ignored for distance purposes here,
    matching the upstream solver's distance assumptions.
    """
    if min_dist <= 0.0:
        return list(frontiers)

    filtered: list[Point3] = []
    for frontier in frontiers:
        too_close = any(
            _distance_xy(frontier, robot_position) <= min_dist
            for robot_position in robot_positions
        )
        if not too_close:
            filtered.append(frontier)

    return filtered

def _build_distance_matrix(
    robot_positions: list[Point3],
    frontier_positions: list[Point3],
) -> list[list[int]]:
    """Build the distance matrix expected by solver_mdvrp

    The upstream solver expects integer distances, so all distances
    are multiplied by ``DISTANCE_SCALE`` and rounded.

    Delegates to ``cfpa2_collaborative_autonomy.mdvrp_solver
    .build_mdvrp_distance_matrix`` if its signature is compatible;
    otherwise builds the matrix directly here.
    """
    return build_mdvrp_distance_matrix(
        frontier_positions,
        robot_positions,
        scale=DISTANCE_SCALE,
    )

def _route_indices_to_positions(
    routes: dict[int, list[int]],
    robot_ids_sorted: list[str],
    frontier_positions: list[Point3],
) -> Assignment:
    """Convert solver output (index-keyed routes) to ID-keyed positions

    The upstream solver returns ``{0: [3, 5, 1], 1: [2, 4]}`` where
    keys are robot indices and values are frontier indices. This
    function maps both back to their string IDs and (x, y, z) positions
    using the deterministic ordering established by the caller.

    Note: upstream solve_mdvrp already filters depot indices out of routes
    (see mdvrp_solver.py: `if node < num_cells:`), so all incoming
    indices in `route` are guaranteed to be frontier indices.
    """
    assignments: Assignment = {robot_id: [] for robot_id in robot_ids_sorted}


    # Defensive bounds-checking: the upstream solver should never produce out-of-range indices, but if it does (e.g. a future upstream bug), we silently skip the bad index rather than crash a long-running node
    for robot_idx, route in routes.items():
        if robot_idx < 0 or robot_idx >= len(robot_ids_sorted):
            continue  # skip invalid robot indices

        robot_id = robot_ids_sorted[robot_idx]

        for frontier_idx in route:
            if 0 <= frontier_idx < len(frontier_positions):
                assignments[robot_id].append(frontier_positions[frontier_idx])

    return assignments

def _distance_xy(a: Point3, b: Point3) -> float:
    """Return 2D Euclidean distance using only the x and y elements of the input tuples"""
    return math.hypot(a[0] - b[0], a[1] - b[1])

# Conversion helpers for the ROS layer (kept here so callers don't need to know the tuple convention used by the algorithmic core). These are NOT used by solve_frontier_assignment itself; they exist for the peer_coordinator_node to convert its ROS messages into the tuple form the core expects.

if TYPE_CHECKING:
    from geometry_msgs.msg import Point, Pose

def pose_msg_to_tuple(pose: Pose) -> Point3:
    """Convert a geometry_msgs/Pose to (x, y, yaw)"""
    q = pose.orientation

    yaw = math.atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )

    return(
      float(pose.position.x),
      float(pose.position.y),
      float(yaw),
    )

def point_msg_to_tuple(point: Point) -> Point3:
    """Convert a geometry_msgs/Point to (x, y, z)"""
    return (
      float(point.x),
      float(point.y),
      float(point.z),
    )