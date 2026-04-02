#!/usr/bin/env python3
"""Min-max MDVRP helper functions used by mtare_coordinator."""

from __future__ import annotations

import math
from typing import Optional

try:
    from ortools.constraint_solver import pywrapcp, routing_enums_pb2

    _HAS_ORTOOLS = True
except Exception:
    pywrapcp = None  # type: ignore[assignment]
    routing_enums_pb2 = None  # type: ignore[assignment]
    _HAS_ORTOOLS = False


def _nearest_neighbor_order(
    *,
    assigned_cells: list[int],
    distance_matrix: list[list[int]],
    depot_node: int,
) -> list[int]:
    route: list[int] = []
    remaining = set(assigned_cells)
    cur = depot_node
    while remaining:
        nxt = min(remaining, key=lambda c: distance_matrix[cur][c])
        route.append(nxt)
        remaining.remove(nxt)
        cur = nxt
    return route


def _heuristic_mdvrp(
    *,
    exploring_cell_positions: list[tuple[float, float, float]],
    robot_positions: list[tuple[float, float, float]],
    distance_matrix: list[list[int]],
) -> dict[int, list[int]]:
    """Greedy load-balanced MDVRP approximation independent of OR-Tools."""
    num_cells = len(exploring_cell_positions)
    num_vehicles = len(robot_positions)
    routes: dict[int, list[int]] = {i: [] for i in range(num_vehicles)}
    if num_cells <= 0 or num_vehicles <= 0:
        return routes

    cell_assignments: dict[int, list[int]] = {i: [] for i in range(num_vehicles)}
    mean_cost = max(1.0, sum(sum(row) for row in distance_matrix) / max(1, len(distance_matrix) ** 2))
    load_penalty = 0.15 * mean_cost

    for cell_idx in range(num_cells):
        best_robot = 0
        best_cost = float("inf")
        for robot_idx in range(num_vehicles):
            depot_idx = num_cells + robot_idx
            projected_load = len(cell_assignments[robot_idx])
            cost = float(distance_matrix[depot_idx][cell_idx]) + (load_penalty * projected_load)
            if cost < best_cost:
                best_cost = cost
                best_robot = robot_idx
        cell_assignments[best_robot].append(cell_idx)

    for robot_idx in range(num_vehicles):
        depot_idx = num_cells + robot_idx
        routes[robot_idx] = _nearest_neighbor_order(
            assigned_cells=cell_assignments[robot_idx],
            distance_matrix=distance_matrix,
            depot_node=depot_idx,
        )
    return routes


def build_mdvrp_distance_matrix(
    exploring_cell_positions: list[tuple[float, float, float]],
    robot_positions: list[tuple[float, float, float]],
    *,
    scale: float = 100.0,
) -> list[list[int]]:
    """Build a Euclidean distance matrix for [cells..., depots...]."""
    locations = list(exploring_cell_positions) + list(robot_positions)
    if not locations:
        return []

    int_scale = max(1.0, float(scale))
    matrix: list[list[int]] = []
    for fx, fy, fz in locations:
        row: list[int] = []
        for tx, ty, tz in locations:
            dist = math.dist((fx, fy, fz), (tx, ty, tz))
            row.append(int(round(dist * int_scale)))
        matrix.append(row)
    return matrix


def solve_mdvrp(
    exploring_cell_positions: list[tuple[float, float, float]],
    robot_positions: list[tuple[float, float, float]],
    distance_matrix: list[list[int]],
    *,
    time_limit_sec: float = 1.0,
    span_cost_coefficient: int = 100,
) -> dict[int, list[int]]:
    """Return ordered cell indices per robot for min-max MDVRP."""
    num_cells = len(exploring_cell_positions)
    num_vehicles = len(robot_positions)
    num_locations = num_cells + num_vehicles
    if num_vehicles <= 0:
        return {}
    if num_cells <= 0:
        return {i: [] for i in range(num_vehicles)}
    if len(distance_matrix) != num_locations:
        return {}
    if any(len(row) != num_locations for row in distance_matrix):
        return {}
    if not _HAS_ORTOOLS:
        return _heuristic_mdvrp(
            exploring_cell_positions=exploring_cell_positions,
            robot_positions=robot_positions,
            distance_matrix=distance_matrix,
        )

    starts = list(range(num_cells, num_locations))
    ends = starts

    manager = pywrapcp.RoutingIndexManager(num_locations, num_vehicles, starts, ends)
    routing = pywrapcp.RoutingModel(manager)

    def distance_callback(from_index: int, to_index: int) -> int:
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return int(distance_matrix[from_node][to_node])

    transit_cb_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_cb_index)

    max_arc = max((max(row) for row in distance_matrix), default=0)
    max_route_cost = max(1, max_arc * max(2, num_cells + 1))
    routing.AddDimension(transit_cb_index, 0, max_route_cost, True, "Distance")
    distance_dim = routing.GetDimensionOrDie("Distance")
    distance_dim.SetGlobalSpanCostCoefficient(max(0, int(span_cost_coefficient)))

    search_params = pywrapcp.DefaultRoutingSearchParameters()
    search_params.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    search_params.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    search_params.time_limit.FromSeconds(max(1, int(math.ceil(max(0.0, time_limit_sec)))))

    solution = routing.SolveWithParameters(search_params)
    if solution is None:
        return _heuristic_mdvrp(
            exploring_cell_positions=exploring_cell_positions,
            robot_positions=robot_positions,
            distance_matrix=distance_matrix,
        )

    routes: dict[int, list[int]] = {}
    for vehicle_idx in range(num_vehicles):
        route: list[int] = []
        index = routing.Start(vehicle_idx)
        while not routing.IsEnd(index):
            node = manager.IndexToNode(index)
            if node < num_cells:
                route.append(node)
            index = solution.Value(routing.NextVar(index))
        routes[vehicle_idx] = route
    return routes


def first_goal_for_route(
    route: list[int],
    exploring_cell_positions: list[tuple[float, float, float]],
    *,
    robot_xy: Optional[tuple[float, float]] = None,
    min_assign_distance: float = 0.0,
) -> Optional[tuple[float, float]]:
    """Pick the first route cell that is not already effectively reached."""
    for cell_idx in route:
        if cell_idx < 0 or cell_idx >= len(exploring_cell_positions):
            continue
        gx, gy, _ = exploring_cell_positions[cell_idx]
        if robot_xy is not None and min_assign_distance > 0.0:
            if math.hypot(gx - robot_xy[0], gy - robot_xy[1]) <= min_assign_distance:
                continue
        return (gx, gy)
    return None
