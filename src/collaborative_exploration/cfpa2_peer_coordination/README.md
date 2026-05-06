# cfpa2_peer_coordination

Decentralised peer-to-peer coordination layer for collaborative Go2 exploration.

Each robot publishes its own heartbeat:

/{robot_namespace}/cfpa2_peer_coordination/peer_state

Each robot receives negotiation messages through its own inbox:
/{robot_namespace}/cfpa2_peer_coordination/inbox/negotiation_request
/{robot_namespace}/cfpa2_peer_coordination/inbox/negotiation_response

Each robot subscribes to peer heartbeats using the configured `peer_namespaces `

## Status 
Skeleton only. Protocol logic not yet implemented

## Scope 
In scope:
- peer state broadcast;
- hard frontier claims;
- request/response negotiation;
- fallback to independent exploration when peer communication is stale;
- optional local overlay of peer map data in a shared frame

Out of scope for version 1:
- distributed SLAM-frame alignment;
- drift correction;
- door/VLM coordination

## Run 
ros2 run cfpa2_peer_coordination peer_coordinator_node

## Parameters
- `robot_id`: protocol identity for this robot.
- `robot_namespace`: ROS namespace for this robot.
- `peer_namespaces`: list of peer robot namespaces. In v1, peer ID is assumed to match peer namespace.
- `peer_timeout_sec`: time before peer state is considered stale.
- `claim_timeout_sec`: time before claims are considered stale.
- `peer_state_rate_hz`: heartbeat publication rate.
- `negotiation_rate_hz`: negotiation attempt rate.
- `negotiation_cooldown_sec`: anti-conflict cooldown between negotiation attempts.
- `odom_topic_suffix`: suffix for this robot's odometry topic.

## Related
- Messages defined in `cfpa2_peer_coordination_msgs`
- Reuses `mdvrp_solver` from `cfpa2_collaborative_autonomy`
- See `docs/claude/decentralisation.md` for design rationale

