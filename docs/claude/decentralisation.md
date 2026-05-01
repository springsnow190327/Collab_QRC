# Decentralised Collaborative Exploration

## Goal
Replace centralised CFPA2 coordinator with peer-to-peer pairwise negotiation, deployed on Jetson Orin Nano per robot.

## Scope

This work only concerns collaborative map exploration. The door/VLM task is out of scope.

The goal is to replace the current centralised CFPA2 frontier coordination with a peer-to-peer decentralised coordination layer. Each Go2 robot runs its own local exploration stack and peer coordination node, intended for deployment on Jetson Orin Nano hardware.

## Reference
RACER (Zhou et al., 2022) — arxiv 2209.08533

## Current centralisation (to verify) - DONE
- cfpa2_coordinator_node.py — single node sees both robots
- mdvrp_solver.py — multi-depot VRP across all frontiers × all robots
- (need to check map_merge_utils.py — is fusion centralised or per-robot?)
- Verified centralisation pattern: single node, namespace-parametric, dispatches via goal_pubs[ns].

## Open questions for supervisor (5-day meeting)
- Confirm scope: full Jetson deployment (a) vs nearby Jetson (b)?
- Is "graceful degradation under comms loss" a required demo feature?
- Cross-robot map fusion under SLAM drift — in scope or out?

## MDVRP solver audit

- `mdvrp_solver.py` is pure Python and reusable outside the central coordinator,
- It exposes `solve_mdvrp(...) -> dict[int, list[int]]`, where robot identities are represented only by integer indices. It does not depend on `rclpy` or ROS message types.

- This means the decentralised peer coordination package can reuse the existing MDVRP solver by wrapping it with deterministic ordering:
- robot IDs are sorted before solving;
- frontier candidates are deduplicated and sorted by quantised coordinate;
- integer assignments are mapped back to robot IDs after solving.

- This avoids rewriting the allocator while removing the centralised assumption that one node directly subscribes to all robot namespaces.

The main entry point is:

```python
solve_mdvrp(
    exploring_cell_positions,
    robot_positions,
    distance_matrix,
    time_limit_sec=1.0,
    span_cost_coefficient=100,
) -> dict[int, list[int]]
```

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

To make clear: mode is a string the existing coordinator might recognise; the package is the new code.

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

## Deliverables

### Package structure
- New sibling package: `cfpa2_peer_coordination/`
- Separate interface package: `cfpa2_peer_coordination_msgs/`
- Both live in `src/collaborative_exploration/` alongside `cfpa2_collaborative_autonomy/`
- Self-contained — teammates can copy the two folders across to the upstream repo

### Custom messages (`cfpa2_peer_coordination_msgs`)
- `ClaimedFrontier.msg` — single frontier owned by a robot (position-only identity, no string ID, no expiry stamp — local timeout policy)
- `PeerState.msg` — periodic heartbeat (~2 Hz) carrying pose, claimed frontiers, interaction timestamps for RACER-style anti-conflict
- `NegotiationRequest.msg` — proposed pairwise allocation, simplified (anti-conflict info inferred from PeerState heartbeats)
- `NegotiationResponse.msg` — accept/reject with confirmed allocation echoed back

### Nodes (`cfpa2_peer_coordination`)
- `peer_coordinator_node.py` — runs once per robot
  - Broadcasts own `PeerState` at ~2 Hz
  - Subscribes to peer's `PeerState`
  - Subscribes to peer's `NegotiationRequest`, publishes own `NegotiationRequest`
  - Subscribes to peer's `NegotiationResponse`, publishes own `NegotiationResponse`
  - Periodically attempts to initiate negotiation (RACER `RequestInteraction` analogue)
  - Implements anti-conflict double-check (RACER Algorithm 2 lines 18–21)
  - Solves pairwise MDVRP via existing `solve_mdvrp` (reused as-is from `cfpa2_collaborative_autonomy`)
  - Maintains hard frontier claims with timestamps
  - Expires stale claims based on local `claim_timeout_sec` parameter
  - Falls back to free exploration when peer broadcasts stop arriving (comms-loss survival)
- `peer_map_merger_node.py` — runs once per robot
  - Subscribes to peer's `/<peer_ns>/map`
  - Calls existing `overlay_map` from `map_merge_utils.py` to fuse into local merged-map view
  - Publishes a per-robot merged occupancy grid for the local exploration stack to consume
  - Inherits the existing system's shared-world-frame assumption (drift handling out of scope)

### Helpers (`cfpa2_peer_coordination`)
- `mdvrp_adapter.py` — pure Python wrapper around `solve_mdvrp` for pairwise (2-robot) use; deterministic ordering of robot IDs and frontier positions; maps integer indices back to namespaces
- `frontier_utils.py` — `frontier_matches(pos_a, pos_b, tol_m)` and related positional-identity helpers
- Unit tests for both helpers, runnable without ROS

### Integration with existing system
- Frontier filter mechanism: peer coordinator publishes a "blocked frontiers" list; existing `cfpa2_single_robot_node` consumes it and excludes those frontiers from its own selection
- Launch flag: `use_decentralised:=true|false` to select between existing centralised coordinator and new peer-based system (avoids dual-publisher conflict on `/<ns>/goal`)

### Hardware deployment
- Two Jetson Orin Nano units, one per Go2
- ROS 2 Humble + the relevant subset of the Collab_QRC stack built on each Jetson
- Peer-to-peer DDS communication over WiFi between Jetsons
- Documented build/setup process in `docs/claude/jetson_setup.md`

### Documentation
- `docs/claude/decentralisation.md` — design rationale, architectural decisions, supervisor questions, status
- `docs/claude/jetson_setup.md` — replicable Jetson environment setup
- Per-package `README.md` for `cfpa2_peer_coordination` and `cfpa2_peer_coordination_msgs`

### Demo
- Two robots exploring collaboratively with peer-based frontier negotiation
- Comms-cut survival demo: kill one peer coordinator mid-run, show graceful degradation to independent exploration (the demonstrable proof of decentralisation)
- Optional stretch: side-by-side comparison with centralised mode via the launch flag

## Status
- [X] Verify centralisation in cfpa2_coordinator_node.py
- [X] Audit map_merge_utils.py for centralised assumptions
- [ ] Architecture sketch
- [ ] Jetson environment setup
- [ ] Implement peer map sync
- [ ] Implement pairwise frontier negotiation
- [ ] Integration + demo