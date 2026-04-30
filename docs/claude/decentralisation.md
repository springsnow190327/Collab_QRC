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

## Status
- [ ] Verify centralisation in cfpa2_coordinator_node.py
- [ ] Audit map_merge_utils.py for centralised assumptions
- [ ] Architecture sketch
- [ ] Jetson environment setup
- [ ] Implement peer map sync
- [ ] Implement pairwise frontier negotiation
- [ ] Integration + demo