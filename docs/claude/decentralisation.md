# Decentralised Collaborative Exploration

## Goal
Replace centralised CFPA2 coordinator with peer-to-peer pairwise negotiation, deployed on Jetson Orin Nano per robot.

## Reference
RACER (Zhou et al., 2022) — arxiv 2209.08533

## Current centralisation (to verify)
- cfpa2_coordinator_node.py — single node sees both robots
- mdvrp_solver.py — multi-depot VRP across all frontiers × all robots
- (need to check map_merge_utils.py — is fusion centralised or per-robot?)
- Verified centralisation pattern: single node, namespace-parametric, dispatches via goal_pubs[ns].

## Open questions for supervisor (5-day meeting)
- Confirm scope: full Jetson deployment (a) vs nearby Jetson (b)?
- Is "graceful degradation under comms loss" a required demo feature?
- Cross-robot map fusion under SLAM drift — in scope or out?

## MDVRP solver audit

`mdvrp_solver.py` is pure Python and reusable outside the central coordinator.
It exposes `solve_mdvrp(...) -> dict[int, list[int]]`, where robot identities are represented only by integer indices. It does not depend on `rclpy` or ROS message types.

This means the decentralised peer coordination package can reuse the existing MDVRP solver by wrapping it with deterministic ordering:
- robot IDs are sorted before solving;
- frontier candidates are deduplicated and sorted by quantised coordinate;
- integer assignments are mapped back to robot IDs after solving.

This avoids rewriting the allocator while removing the centralised assumption that one node directly subscribes to all robot namespaces.

# Decentralised Collaborative Exploration

## Scope

This work only concerns collaborative map exploration. The door/VLM task is out of scope.

The goal is to replace the current centralised CFPA2 frontier coordination with a peer-to-peer decentralised coordination layer. Each Go2 robot runs its own local exploration stack and peer coordination node, intended for deployment on Jetson Orin Nano hardware.

## Current System

The current CFPA2 coordinator is namespace-parameterised, but centralised in execution. A single node can subscribe to multiple robot namespaces, maintain state for all robots, solve the joint assignment problem, and publish goals to each robot.

Relevant files:

- `cfpa2_coordinator_node.py`
- `cfpa2_single_robot_node.py`
- `mdvrp_solver.py`
- `map_merge_utils.py`

The key centralised behaviour is in `_tick_impl`, where the coordinator uses all robot states and frontier candidates to decide assignments.

## Existing Algorithm Modes

The existing coordinator contains multiple planning modes:

- `cfpa2`: two-robot joint assignment, explicitly centralised.
- `mtare`: per-namespace greedy utility assignment.
- `collaborative`: auction-style assignment with overlap penalties.
- proposed: `decentralised`: peer-to-peer hard frontier claims with request/response negotiation.

## Decentralisation Criterion

The system is considered decentralised if each robot can continue exploring using its own local map and local planner when communication with its peer is lost.

Peer information must only be received through explicit peer messages, not by directly subscribing to the peer robot’s raw map, odometry, or status topics.

## Proposed Architecture

Each robot runs:

- existing local exploration/navigation stack;
- existing `cfpa2_single_robot_node`;
- new `cfpa2_peer_coordination` node.

The peer coordination node:

- publishes this robot’s current pose, goal, and claimed frontiers;
- subscribes to peer state messages;
- sends and receives negotiation requests;
- uses the existing pure-Python MDVRP solver for pairwise assignment;
- maintains hard frontier claims with timestamps;
- expires stale claims if communication is lost.

## MDVRP Solver Audit

`mdvrp_solver.py` is reusable. It is pure Python and has no dependency on `rclpy` or ROS messages.

The main entry point is:

```python
solve_mdvrp(
    exploring_cell_positions,
    robot_positions,
    distance_matrix,
    time_limit_sec=1.0,
    span_cost_coefficient=100,
) -> dict[int, list[int]]

## Status
- [ ] Verify centralisation in cfpa2_coordinator_node.py
- [ ] Audit map_merge_utils.py for centralised assumptions
- [ ] Architecture sketch
- [ ] Jetson environment setup
- [ ] Implement peer map sync
- [ ] Implement pairwise frontier negotiation
- [ ] Integration + demo