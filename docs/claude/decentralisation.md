# Decentralised Collaborative Exploration

## Goal
Replace centralised CFPA2 coordinator with peer-to-peer pairwise negotiation, deployed on Jetson Orin Nano per robot.

## Scope

This work only concerns collaborative map exploration. The door/VLM task is out of scope.

The goal is to replace the current centralised CFPA2 frontier coordination with a peer-to-peer decentralised coordination layer. Each Go2 robot runs its own local exploration stack and peer coordination node, intended for deployment on Jetson Orin Nano hardware.

## Reference
RACER (Zhou et al., 2022) — arxiv 2209.08533
M-TARE/TARE (Cao et al., 2021) - DOI:10.15607/RSS.2021.XVII.018

## Current centralisation (to verify) - DONE 30/04/2026
- cfpa2_coordinator_node.py — single node sees both robots
- mdvrp_solver.py — multi-depot VRP across all frontiers × all robots
- (need to check map_merge_utils.py — is fusion centralised or per-robot?)
- Verified centralisation pattern: single node, namespace-parametric, dispatches via goal_pubs[ns].

## Open questions for Dimitrios - DONE 11/05/2026
Responses:
1) Peer-stale vs claim-stale behaviour. 
  Q: Each ClaimedFrontier carries a claim_stamp and expires locally after claim_timeout_sec (currently 30s). Peer heartbeats are tracked separately, with peers marked stale after peer_timeout_sec (currently 5s). The question is what happens between those two — i.e. when a peer's heartbeat has expired but its claims are still individually fresh.

  A: Option B: drop all of a stale peer's claims immediately. Treats staleness as "I cannot trust any of this peer's state," reverting to single-robot behaviour faster. 

  Maps cleanly onto the decentralisation criterion, and the responsiveness argument outweighs A's conservatism. The WiFi-blip sensitivity is real but the fix is the one you suggest: bump peer_timeout_sec somewhat (I'd start at 10–15s and tune empirically). I'd rather not pay the state-machine complexity of C upfront when B plus a more forgiving timeout will likely cover it; we can revisit if testing shows genuine thrash. Worth instrumenting from the start: log every peer-stale event so you can see post-hoc whether dropouts are real peer loss or transient comms.

2. Decentralised map fusion. 
  Q: The existing map_merge_utils.py exposes a stateless overlay_map primitive that overlays one occupancy grid on another (occupied beats free beats unknown). Both robots already share a world frame via common-origin Fast-LIO initialisation, so a per-robot map fusion node would only need to subscribe to the peer's /map and call overlay_map locally, which would perhaps be a day's wiring.
  A: Include it. A day's wiring for a materially better demo is a good trade, and the drift limitation is inherited from the centralised baseline rather than introduced by your work: listing it explicitly as future work is the honest framing. Inter-robot loop closure is correctly out of scope. If the wiring stretches past ~2 days, stop and reassess, but I don't expect that given overlay_map is already stateless.

3. Success criterion for the demo. 
  Q: I've been working on the assumption that the demonstrable proof of decentralisation is graceful degradation under simulated comms loss, which is kill one peer coordinator mid-run, show the other robot continues exploring without it. This maps directly onto how the literature (RACER, etc.) characterises decentralised systems.

  A: Yes, graceful degradation under simulated comms loss is what I want to see, not a head-to-head efficiency comparison. Three reasons: it directly demonstrates the property you're claiming (an efficiency comparison would be confounded and may not even favour the decentralised mode, which isn't the point); it's what the RACER-style framing supports; and the testing infrastructure is lighter. If you can additionally show recovery when the peer rejoins, not just survival when it leaves, that strengthens the demo without much extra cost, but the kill-mid-run scenario is the headline. Build accordingly; treat Q3 as settled.

## MDVRP solver audit - DONE 07/05/2026
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
After upstream removal on main (10/05/2026), mdvrp_solver.py was vendored into cfpa2_peer_coordination/ to keep the decentralised package self-contained and avoid future breakage.

### MDVRP Adapter Determinism Note - TESTED 08/05/2026

The adapter test suite verifies that shuffled robot/frontier input ordering produces identical assignments. This confirms that the deterministic ordering wrapper works with the current upstream MDVRP solver.

The tests implicitly rely on OR-Tools producing reproducible routes for identical inputs. If this test becomes flaky on another machine or OR-Tools version, the likely cause is solver-side randomisation rather than adapter ordering. This could be hardened by setting an explicit random seed in the upstream `solve_mdvrp` search parameters, or by weakening the test to compare equivalent assignment sets rather than exact route order.

## Current System

The current CFPA2 coordinator is namespace-parameterised, but centralised in execution. A single node can subscribe to multiple robot namespaces, maintain state for all robots, solve the joint assignment problem, and publish goals to each robot.

Relevant files:

- `cfpa2_coordinator_node.py`
- `cfpa2_single_robot_node.py`
- `mdvrp_solver.py`
- `map_merge_utils.py`

The key centralised behaviour is in `_tick_impl`, where the coordinator uses all robot states and frontier candidates to decide assignments.

- UPDATED 12/05/2026
  The centralised CFPA2 mode in cfpa2_coordinator_node performs joint scoring across both robots. The single-robot variant cfpa2_single_robot_node already scores frontiers independently per robot. The decentralisation effort therefore turns the two single-robot pipelines into a coordinated pair: each robot retains its own frontier scoring, but blocked-frontier filtering and broadcast claims prevent both robots from independently selecting overlapping goals.

  The existing centralised coordinator performs joint multi-robot assignment after computing utilities for all robots, whereas the single-robot planner already performs local frontier extraction and local utility scoring. The decentralised extension reuses the single-robot planner and injects peer-claim awareness by filtering out frontiers claimed by peers before utility maximisation.

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

### Custom messages (`cfpa2_peer_coordination_msgs`) - DONE 01/05/2026
- `ClaimedFrontier.msg` — single frontier owned by a robot (position-only identity, no string ID, no expiry stamp — local timeout policy)
- `PeerState.msg` — periodic heartbeat (~2 Hz) carrying pose, claimed frontiers, interaction timestamps for RACER-style anti-conflict
- `NegotiationRequest.msg` — proposed pairwise allocation, simplified (anti-conflict info inferred from PeerState heartbeats)
- `NegotiationResponse.msg` — accept/reject with confirmed allocation echoed back

### Request ID Strategy

Negotiation requests use deterministic, human-readable request IDs of the form:

`<requester_id>-<local_counter>`

Each peer coordinator maintains a local monotonically increasing counter. The requester ID scopes the counter, so simultaneous requests from different robots cannot collide. This is preferred over timestamp-only IDs because robot clocks may differ or produce near-simultaneous values, and it is preferred over UUIDs because it is easier to inspect in ROS logs and debugging output.y

```python
self.request_counter = 0

def _next_request_id(self) -> str:
    self.request_counter += 1
    return f"{self.robot_id}-{self.request_counter}"
```

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
- Onboard Jetson on each Go2 (built-in, no separate units required)
- ROS 2 code maintained throughout; ROS 1 bridge to the Go2 SDK
- Peer-to-peer DDS over the existing Go2 WiFi

### Documentation
- `docs/claude/decentralisation.md` — design rationale, architectural decisions, supervisor questions, status
- `docs/claude/jetson_setup.md` — replicable Jetson environment setup
- Per-package `README.md` for `cfpa2_peer_coordination` and `cfpa2_peer_coordination_msgs`

### Demo
- Two robots exploring collaboratively with peer-based frontier negotiation
- Comms-cut survival demo: kill one peer coordinator mid-run, show graceful degradation to independent exploration (the demonstrable proof of decentralisation)
- Optional stretch: side-by-side comparison with centralised mode via the launch flag
- (Stretch) Peer rejoin and resume

## Notes on  `peer_coordination_node.py`
### PeerState Heartbeat Design - DONE 08/05/2026

PeerState messages are published using best-effort QoS with depth 1. This matches the semantics of heartbeat/state broadcasts: old messages become stale quickly, so the receiver only needs the most recent state.

Each node stores peer heartbeat data in a `PeerInfo` dataclass keyed by peer ID. The stored receive timestamp uses the local ROS 2 clock, not the sender's message timestamp, so freshness checks are based on when this robot last heard from the peer.

PeerState topics use publisher-scoped namespacing:

`/{robot_namespace}/cfpa2_peer_coordination/peer_state`

Each robot publishes under its own namespace and subscribes to the configured peer namespaces.

### Claim and Frontier Management - DONE 12/05/2026
For the first integration, the peer coordinator ingests local frontier candidates from the existing CFPA2 frontier MarkerArray output. This avoids invasive changes to the existing planner while giving the decentralised layer access to the same frontier positions already used for visualisation. A future cleaner integration would expose frontier candidates through a dedicated typed topic rather than parsing visualisation markers.

Implemented and manually tested the first claim-management layer for decentralised exploration.

The peer coordinator now subscribes to the existing CFPA2 frontier visualisation output, using `visualization_msgs/MarkerArray` markers in the `cfpa2_frontiers` namespace as a pragmatic v1 frontier input. These marker positions are converted into deterministic `(x, y, z)` tuples and stored as `local_frontiers`.

The node now maintains three frontier/claim stores:

- `local_frontiers`: candidate frontiers visible to this robot;
- `own_claims`: frontiers currently claimed by this robot, to be populated by future negotiation logic;
- `peer_claims`: frontiers claimed by peers and received through `PeerState` heartbeats.

Peer claims are filtered using `claim_timeout_sec`, so stale claims stop blocking local frontiers after expiry. Frontier equality is determined by spatial proximity using a module-level `FRONTIER_MATCH_TOLERANCE_M = 0.5`, ensuring both peers use the same deterministic matching rule.

Conflict resolution has also been added for future negotiation use. If this robot and a peer claim the same frontier, the deterministic winner rule is:

1. the earlier `claim_stamp` wins;
2. if timestamps tie, the lexicographically smaller `claimed_by` robot ID wins.

Manual ROS testing verified the full claim-blocking lifecycle:

1. With fake frontier markers only, `local_frontiers=2` and `available_frontiers=2`.
2. After publishing a fake `robot_b` `PeerState` containing one matching `ClaimedFrontier`, the peer coordinator stores one peer claim and `available_frontiers` drops from 2 to 1.
3. After the claim timeout expires, the stale claim is removed and `available_frontiers` recovers from 1 to 2.

This confirms that local frontier ingestion, peer claim storage, stale-claim expiry, and peer-claim blocking are working. The deterministic conflict-resolution helper is implemented, but should still be unit-tested with hand-crafted claims before being marked fully verified.

Claim storage, peer-claim ingestion, claim expiry, frontier blocking, and deterministic conflict resolution have been implemented and tested. The deterministic rule uses earliest claim timestamp as the winner, with robot ID lexicographic ordering as a tie-break. Current testing uses interim MDVRP-generated own claims before the full request/response negotiation protocol is implemented.

### Frontier Filter Output - 13/05/2026
- Note: cfpa2_collaborative_autonomy is now in 3d rather than 2d
peer coordinator claim → blocked_frontiers PoseArray → single_robot_node receives → _peer_has_claimed() returns True

planner is still saying:

```python
Waiting for map topic from: robot_a
Waiting for map topic from: robot_b
```

So this test proves the blocked-frontier communication + filter logic, not full goal publication yet.

## Negotiation Protocol Notes
### State machines
The protocol uses two independent state machines per peer per direction:
- REQUESTER (IDLE / REQUESTING) — drives outgoing proposals
- RESPONDER (mostly stateless, with one timing field for anti-conflict) —
  evaluates incoming proposals

A robot can be REQUESTING to peer X while RESPONDING to peer X
(due to crossed in-flight requests) without conflict.

### Anti-conflict
Layered RACER-style anti-conflict (Algorithm 2 lines 18-21):
1. If currently REQUESTING to the same peer, the request crossed in flight.
   Reject deterministically; both robots back off and serialise via cooldown.
2. General epsilon-att check on `last_interaction_attempt_stamp`: if I just
   tried any interaction (with anyone), reject incoming requests within
   epsilon-att window.

### Commit ordering and atomicity
Responder sends accept *before* committing locally. This avoids the case
where the responder commits but the response is lost in flight, leaving
the requester ignorant of a commitment that exists on the peer side.

Even with this ordering, the protocol is eventually consistent under
message loss rather than strictly atomic. Recovery mechanisms:
- Claim timeout (`claim_timeout_sec`) drops stale commitments
- Peer staleness (`peer_timeout_sec`) drops all claims from disappeared peers
- Heartbeat-based reconciliation lets diverging views re-align

This is acceptable for the comms-cut survival demo, which is the explicit
success criterion (supervisor decision Q3, 08/05/2026). Stricter atomicity
(two-phase commit, consensus) is out of scope.

### Validation by re-solving
Responder validates a proposal by re-solving the MDVRP locally with the
same inputs and comparing to the proposed allocation via set equality
with FRONTIER_MATCH_TOLERANCE-tolerant matching. This works because
MDVRP solver determinism is tested
(see test_determinism_under_shuffled_inputs).

If the comparison fails, the responder rejects and lets state refresh
(heartbeats, frontier updates) before the next retry. Counter-proposals
are out of scope.

### Tunables (defaults)
- request_timeout_sec: 2.0  (matches negotiation_cooldown_sec)
- epsilon_att: 2.0  (matches negotiation_cooldown_sec for simplicity)
- backoff after reject: exponential, capped (TBD)
- backoff after timeout: shorter than reject (treats as comms issue)

### Logs
- Chunk A - DONE 14/05/2026
``` bash
ems@headgoboom:~/Collab_QRC$ $ ros2 topilist | grep negotiation
/robot_a/cfpa2_peer_coordination/inbox/negotiation_request
/robot_a/cfpa2_peer_coordination/inbox/negotiation_response
/robot_b/cfpa2_peer_coordination/inbox/negotiation_request
/robot_b/cfpa2_peer_coordination/inbox/negotiation_response
```

## Status
- [X] Verify centralisation in cfpa2_coordinator_node.py
- [X] Audit map_merge_utils.py for centralised assumptions
- [X] Architecture sketch
- [ ] Jetson environment setup
- [X] PeerState heartbeat publish + subscribe
- [X] Peer state freshness tracking (timeout detection)
- [X] Log peer-stale events
- [X] Bonus: real-pose ingestion from `/odom/nav`
- [ ] Pairwise frontier negotiation (request/response protocol)
  - [X] Chunk A: request/response inbox wiring
  - [ ] Chunk B: requester state machine
  - [ ] Chunk C: responder validation + accept/reject
  - [ ] Chunk D: replace interim MDVRP auto-claim path
- [X] Claim management: storage, expiry, and peer-claim blocking implemented/tested
- [X] Frontier management: local frontier ingestion from CFPA2 MarkerArray and peer-claim filtering implemented
- [X] MDVRP-generated own-claim proposal
- [X] Claim conflict resolution rule implemented/tested
- [X] Frontier filter output (so single_robot_node respects claims)
- [ ] Peer map subscriber + overlay_map fusion - being implemented by Haichen 13/05/2026.  My work will integrate with his merged-map interface once topic name, message type and freshness semantics are confirmed.
- [ ] Integration with existing single_robot_node
- [ ] Comms-cut survival demo

