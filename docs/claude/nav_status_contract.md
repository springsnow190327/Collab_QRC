# nav_status/v1 — Planner → CFPA2 feedback contract

A canonical JSON schema on `/<ns>/nav_status` (std_msgs/String) that any local planner can publish and CFPA2 will understand. Lets CFPA2 blacklist unreachable frontier goals within ~200 ms instead of the 8 s motion-based stuck timer.

## Message

```
std_msgs/String
  data: JSON, UTF-8, one message per publish
```

## Schema

Required fields:

| Field | Type | Meaning |
|---|---|---|
| `schema` | str `"nav_status/v1"` | version tag |
| `source` | str | producing node; e.g. `"reactive_nav"`, `"default_nav"`, `"far_adapter"` |
| `state` | enum | one of `"idle"`, `"navigating"`, `"goal_reached"`, `"unreachable"`, `"stalled"`, `"failed"` (see semantics below) |
| `goal` | [x, y] float, or `null` | current goal in map frame; null when idle |
| `goal_seq` | int | planner-local monotonic counter, incremented on every new goal received; used by CFPA2 for dedup |
| `reason` | str | short machine-readable tag, e.g. `"reached"`, `"no_path"`, `"graph_disconnected"`, `"far_silent"`, `"blocked_front"` |
| `stamp_sec` | float | ROS time when this status was published |

Optional / legacy-compat fields (preserved by `default_nav` for the pre-existing coordinator stall path):

| Field | Type | Notes |
|---|---|---|
| `mode` | str | legacy state name (e.g. `"navigate"`, `"goal_reached"`, `"stuck"`) |
| `stall_event_count` | int | legacy stall counter; coordinator's 45 s blacklist path reads this |
| `stall_sec` | float | how long stalled |
| `pos` | [x, y] | robot position |
| `speed` | float | |
| `dist_goal_live` | float | |

Planners other than `default_nav` do not need to emit legacy fields.

## State vocabulary (semantics)

| State | Meaning | CFPA2 response |
|---|---|---|
| `idle` | No goal to act on | no-op |
| `navigating` | Actively pursuing goal; path exists | no-op (normal operation) |
| `goal_reached` | Within goal tolerance | CFPA2's reached-goal path fires |
| `unreachable` | Planner has **definitively** determined the current goal can't be reached (A* returned no-path N times, FAR V-graph disconnected, RRT* exhausted) | **Fast-blacklist (~200 ms)** — goal added to blacklist for `fast_unreachable_blacklist_sec` (60 s default), CFPA2 re-picks next best frontier |
| `stalled` | Soft signal: making no progress but not confirmed unreachable (obstacle, bumper, slow going) | legacy coordinator path on `stall_event_count`; 45 s timer |
| `failed` | Planner crash / critical error; effectively unreachable-permanent | same as `unreachable` |

Only `unreachable` and `failed` trigger the fast blacklist. `stalled` remains on the slow legacy timer so that genuinely hard-to-reach (but reachable-with-patience) goals aren't prematurely given up on.

## Producers

| Planner | Source | How state is inferred |
|---|---|---|
| `reactive_nav_node` (C++) | `source="reactive_nav"` | Tick-level state tracking: RRT* failure count ≥ 3 + global A* fail → `unreachable`. Within goal tolerance → `goal_reached`. Otherwise `navigating`. Throttled at 5 Hz. |
| `default_nav.py` | `source="default_nav"` | Maps existing `mode` field. `mode=="goal_reached"` → `goal_reached`; `mode=="stuck"` or `stall_event_count` incrementing → `stalled`; definitive planner failure → `unreachable`. |
| FAR (CMU stack) | `source="far_adapter"` | Non-invasive: adapter node subscribes to `/far_reach_goal_status` (Bool) + `/way_point` (FAR's route output) + `/goal_point` (goal input). No `/way_point` within `unreachable_timeout_sec` → `unreachable`. `/far_reach_goal_status` true → `goal_reached`. Heartbeat from `/far_planning_time` alive → `navigating`. |

Future planners (TARE, custom) can plug in by publishing to the same topic with the same schema. No CFPA2 change required.

## Consumer

`cfpa2_coordinator_node.py` subscribes to `/<ns>/nav_status`. Both single-robot and dual inherit this (single-robot class inherits from coordinator).

In `_nav_status_cb`:
1. Parse JSON, store payload (legacy path for `stall_event_count` stays intact).
2. If `payload["state"] ∈ {"unreachable", "failed"}`:
   a. Dedup: ignore if `(ns, payload["goal_seq"])` already handled.
   b. Match: compute `_goal_key(payload["goal"])` and compare to `_goal_key(last_goal[ns])`. If mismatch → planner is reporting on a stale goal; ignore.
   c. Blacklist: add `goal_key → now + fast_unreachable_blacklist_sec * 1e9` to `goal_blacklist_until_ns[ns]`.
   d. Clear progress samples + goal_fail_counts so next assignment starts clean.
   e. Log: `WARN "{ns}: FAST-BL goal=({x:.2f},{y:.2f}) state={state} reason={reason} src={source}"`.

## Parameters (`cfpa2_single_robot.yaml` / `cfpa2_coordinator.yaml`)

| Parameter | Default | Meaning |
|---|---|---|
| `fast_unreachable_enabled` | `true` | master enable |
| `fast_unreachable_blacklist_sec` | `60.0` | TTL applied to fast-blacklisted goals |

Legacy params (`local_nav_stall_blacklist_sec`, `local_nav_status_stale_sec`) remain and drive the slow `stalled` path.

## Swappability

To add a new planner:
1. Make it publish `std_msgs/String` JSON on `/<ns>/nav_status` following the required schema.
2. That's it — CFPA2 recognises the contract and feeds back within ~200 ms of `unreachable`/`failed`.

To replace a planner: same contract, same behaviour. No CFPA2 changes.

## Testing

```bash
# 1. Verify emission
ros2 topic echo /robot/nav_status --once
# Expect all required fields populated, schema="nav_status/v1".

# 2. Force an unreachable goal
ros2 topic pub --once /robot/way_point_coord geometry_msgs/PointStamped \
  "{header: {frame_id: 'map'}, point: {x: 999.0, y: 999.0, z: 0.0}}"
# Within 1-3 s planner publishes state="unreachable".
# CFPA2 WARN log shows "FAST-BL goal=(999.00,999.00) state=unreachable src=<planner>".

# 3. Swap backend, same test — schema is identical regardless of planner.
# nav_backend=far spawns far_status_adapter which emits same fields.
```
