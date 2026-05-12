"""Decentralised peer coordinator node.

Each robot runs one instance of this node. It broadcasts this robot's PeerState heartbeat and (later) negotiates frontier ownership with peers.

This is the skeleton; protocol logic is not yet implemented.

"""

from __future__ import annotations

from dataclasses import dataclass
from visualization_msgs.msg import MarkerArray 

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy, QoSProfile

from std_msgs.msg import Header
from geometry_msgs.msg import Point, Pose
from nav_msgs.msg import Odometry

from cfpa2_peer_coordination.mdvrp_adapter import (
    Point3,
    distance_xy,
    point_msg_to_tuple,
    pose_msg_to_tuple,
    solve_frontier_assignment,
)

from cfpa2_peer_coordination_msgs.msg import (
    ClaimedFrontier,         
    NegotiationRequest,      # noqa: F401
    NegotiationResponse,     # noqa: F401
    PeerState,
)

PROTOCOL_VERSION = 1   # bump when message formats change incompatibly

# Both peers must use the same value for claim equity
"""if two frontiers are within this distance of each other, they are considered the same frontier for the purpose of claim equity calculations"""
FRONTIER_MATCH_TOLERANCE = 0.5  # metres

# Topic naming convention
# PeerState is published under the sender's namespace:
#   /robot_a/cfpa2_peer_coordination/peer_state
#
# Negotiation messages will later use destination-scoped inboxes:
#   /robot_a/cfpa2_peer_coordination/inbox/negotiation_request
#   /robot_a/cfpa2_peer_coordination/inbox/negotiation_response
PEER_STATE_TOPIC = "cfpa2_peer_coordination/peer_state"
NEGOTIATION_REQUEST_TOPIC = "cfpa2_peer_coordination/inbox/negotiation_request"
NEGOTIATION_RESPONSE_TOPIC = "cfpa2_peer_coordination/inbox/negotiation_response"

# Existing CFPA2 frontier visualisation markers use this namespace
CFPA2_FRONTIER_MARKER_NS = "cfpa2_frontiers"  # ns = namespace

# QoS profile for peer-state heartbeats: best-effort, latest-only.
# Heartbeats are intentionally lossy; a missed message means we use the previous state (and eventually trigger a freshness timeout).
# Reliable delivery would queue stale heartbeats behind retried ones, which is the wrong behaviour for state broadcasts.
PEER_STATE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)

@dataclass
class PeerInfo:
    """Tracks per-peer state for the local coordinator.

    Extended over the project: currently holds the last received
    PeerState and the local timestamp it arrived at. Future fields
    will include claim tracking and negotiation-state machines."""

    last_state: PeerState | None = None
    last_received_ns: int = 0
    was_fresh: bool = False


class PeerCoordinatorNode(Node):
    """Per-robot peer coordinator. One instance per robot."""

    def __init__(self) -> None:
        super().__init__("cfpa2_peer_coordinator")

        # Parameters
        self.declare_parameter("robot_id", "robot_a")
        self.declare_parameter("robot_namespace", "robot_a")

        # assumption: peer_id == peer_namespace. Used for both protocol identity and topic addressing. Split into separate parameters if this assumption ever needs to break
        self.declare_parameter("peer_namespaces", ["robot_b"])

        self.declare_parameter("peer_timeout_sec", 10.0) 
        self.declare_parameter("claim_timeout_sec", 30.0)
        self.declare_parameter("peer_state_rate_hz", 2.0)
        self.declare_parameter("negotiation_rate_hz", 1.0)
        self.declare_parameter("negotiation_cooldown_sec", 2.0)

        self.declare_parameter("odom_topic_suffix", "/odom/nav")
        self.declare_parameter("frontier_markers_topic", "/mtare/frontier_markers")

        # Interim milestone parameters
        # This is NOT the final request/response protocol yet. It lets the node generate local own_claims from the shared MDVRP solver so claim broadcast and conflict resolution can be tested before full negotiation exists
        self.declare_parameter("enable_mdvrp_auto_claims", True)
        self.declare_parameter("mdvrp_time_limit_sec", 0.5)
        self.declare_parameter("mdvrp_span_cost_coefficient", 100)

        # Read parameters into fields for easy access.
        self.robot_id: str = self.get_parameter("robot_id").value
        self.robot_namespace: str = (
            self.get_parameter("robot_namespace").value.strip().strip("/")
        )
        self.peer_namespaces: list[str] = list(
            self.get_parameter("peer_namespaces").value
        )
        self.peer_ids: list[str] = list(self.peer_namespaces)  # derived view

        self.peer_timeout_sec: float = float(
            self.get_parameter("peer_timeout_sec").value
        )
        self.claim_timeout_sec: float = float(
            self.get_parameter("claim_timeout_sec").value
        )
        self.peer_state_rate_hz: float = float(
            self.get_parameter("peer_state_rate_hz").value
        )
        self.negotiation_rate_hz: float = float(
            self.get_parameter("negotiation_rate_hz").value
        )
        self.negotiation_cooldown_sec: float = float(
            self.get_parameter("negotiation_cooldown_sec").value
        )

        self.odom_topic_suffix: str = self.get_parameter("odom_topic_suffix").value
        self.frontier_markers_topic: str = str(
            self.get_parameter("frontier_markers_topic").value
        )

        # Interim milestone parameters
        self.enable_mdvrp_auto_claims: bool = bool(
            self.get_parameter("enable_mdvrp_auto_claims").value
        )
        self.mdvrp_time_limit_sec: float = float(
            self.get_parameter("mdvrp_time_limit_sec").value
        )
        self.mdvrp_span_cost_coefficient: int = int(
            self.get_parameter("mdvrp_span_cost_coefficient").value
        )

        # Per-peer storage. Initialised with a placeholder for every known peer. Updated by _peer_state_received(), read by negotiation logic (later).
        self.peer_info: dict[str, PeerInfo] = {
            peer_id: PeerInfo() for peer_id in self.peer_ids
        }

        # Local pose storage. Updated by _odom_received(), published in PeerState
        self._latest_pose: Pose | None = None

        # Frontier + claim storage
        """local_frontiers is updated from the existing CFPA2 frontier MarkerArray
        own_claims will be filled by the future negotiation logic
        peer_claims is updated from received PeerState messages"""
        self.local_frontiers: list[Point3] = []
        self.own_claims: list[ClaimedFrontier] = []
        self.peer_claims: dict[str, list[ClaimedFrontier]] = {
            peer_id: [] for peer_id in self.peer_ids
        }

        # Cooldown for interim MDVRP auto-claim generation
        # Final request/response negotiation will use the same cooldown concept
        self._last_negotiation_attempt_ns: int = 0

        # Topic names
        self.own_peer_state_topic = f"/{self.robot_namespace}/{PEER_STATE_TOPIC}"

        self.own_request_inbox_topic = (
            f"/{self.robot_namespace}/{NEGOTIATION_REQUEST_TOPIC}"
        )
        self.own_response_inbox_topic = (
            f"/{self.robot_namespace}/{NEGOTIATION_RESPONSE_TOPIC}"
        )

        self.peer_state_topics = {
            peer_id: f"/{peer_ns}/{PEER_STATE_TOPIC}"
            for peer_id, peer_ns in zip(self.peer_ids, self.peer_namespaces)
        }
        self.peer_request_inbox_topics = {
            peer_id: f"/{peer_ns}/{NEGOTIATION_REQUEST_TOPIC}"
            for peer_id, peer_ns in zip(self.peer_ids, self.peer_namespaces)
        }
        self.peer_response_inbox_topics = {
            peer_id: f"/{peer_ns}/{NEGOTIATION_RESPONSE_TOPIC}"
            for peer_id, peer_ns in zip(self.peer_ids, self.peer_namespaces)
        }

        self.own_odom_topic = f"/{self.robot_namespace}{self.odom_topic_suffix}"

        # Local odometry subscriber
        self.odom_sub = self.create_subscription(
            Odometry,
            self.own_odom_topic,
            self._odom_received,
            10,
        )
        self.get_logger().info(f"Subscribed to own odometry on {self.own_odom_topic}")

        # Local frontier marker subscriber
        # v1 uses existing CFPA2 visualisation markers as a pragmatic frontier input
        self.frontier_markers_sub = self.create_subscription(
            MarkerArray,
            self.frontier_markers_topic,
            self._frontier_markers_received,
            10,
        )
        self.get_logger().info(
            f"Subscribed to frontier markers on {self.frontier_markers_topic}"
        )

        # PeerState publisher/subscribers
        self.peer_state_pub = self.create_publisher(
            PeerState,
            self.own_peer_state_topic,
            PEER_STATE_QOS,
        )

        # Store subscription objects so they are not garbage-collected. We only subscribe to peer state topics for now; negotiation inboxes will be added later.
        self.peer_state_subs = []

        for peer_id, peer_topic in self.peer_state_topics.items():
            sub = self.create_subscription(
                PeerState,
                peer_topic,
                lambda msg, pid=peer_id: self._peer_state_received(msg, pid),  # pid=peer_id
                PEER_STATE_QOS,
            )
            self.peer_state_subs.append(sub)

        # Timers (stubs only for now)
        peer_state_period = 1.0 / max(self.peer_state_rate_hz, 1e-6)
        negotiation_period = 1.0 / max(self.negotiation_rate_hz, 1e-6)

        self.peer_state_timer = self.create_timer(
            peer_state_period, 
            self._publish_peer_state,
        )
        self.negotiation_timer = self.create_timer(
            negotiation_period, 
            self._decide_negotiation,
        )

        self._logged_negotiation_stub = False  # to avoid spamming logs with the stub message

        # Startup logs
        self.get_logger().info(
            f"PeerCoordinatorNode starting | robot_id={self.robot_id} "
            f"namespace={self.robot_namespace} "
            f"peer_namespaces={self.peer_namespaces} "
            f"protocol_version={PROTOCOL_VERSION}"
        )
        self.get_logger().info(
            f"Publishing own PeerState on {self.own_peer_state_topic}"
        )
        self.get_logger().info(
            f"Subscribed peer PeerState topics: {self.peer_state_topics}"
        )
        self.get_logger().info(
            "Own negotiation inbox topics planned | "
            f"request={self.own_request_inbox_topic} "
            f"response={self.own_response_inbox_topic}"
        )
        self.get_logger().info(
            f"Peer request inbox publishers planned: {self.peer_request_inbox_topics}"
        )
        self.get_logger().info(
            f"Peer response inbox publishers planned: {self.peer_response_inbox_topics}"
        )

        # Interim MDVRP auto-claim generation log
        self.get_logger().info(
            "Interim MDVRP auto-claims | "
            f"enabled={self.enable_mdvrp_auto_claims} "
            f"time_limit_sec={self.mdvrp_time_limit_sec:.2f}s "
            f"span_cost={self.mdvrp_span_cost_coefficient}"
        )
    
    # Function for making message headers
    def _make_header(self) -> Header:
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"
        return header

    # Function for receiving odom 
    def _odom_received(self, msg: Odometry) -> None:
        """Store this robot's latest pose for inclusion in PeerState heartbeats."""
        self._latest_pose = msg.pose.pose

    # Function for receiving frontier markers
    def _frontier_markers_received(self, msg: MarkerArray) -> None:
        """Store local frontier candidates from existing CFPA2 frontier markers."""
        frontiers: list[Point3] = []

        for marker in msg.markers:
            # Ignore DELETE/DELETEALL markers
            if marker.action != marker.ADD:
                continue

            # Existing CFPA2 frontier visualisation markers use this namespace
            if marker.ns != CFPA2_FRONTIER_MARKER_NS:
                continue

            frontiers.append(
                (
                    float(marker.pose.position.x),
                    float(marker.pose.position.y),
                    float(marker.pose.position.z),
                )
            )
        
        self.local_frontiers = sorted(frontiers, key=lambda p: (p[0], p[1], p[2]))  # sort for consistency

        self.get_logger().debug(
            f"Stored {len(self.local_frontiers)} local frontier candidates"
        )

    # Function for publishing PeerState   
    def _publish_peer_state(self) -> None:
        """Broadcast this robot's current PeerState heartbeat."""
        msg = PeerState()
        msg.header = self._make_header()
        msg.robot_id = self.robot_id

        if self._latest_pose is not None:
            msg.pose = self._latest_pose
        else:
            # Not yet received any odom; default zero pose is left in place.
            # Log periodically to surface the issue without spamming.
            self.get_logger().warn(
                f"Publishing heartbeat with no odom yet on {self.own_peer_state_topic}",
                throttle_duration_sec=2.0,
            )

        self._expire_stale_claims()
        msg.claimed_frontiers = list(self.own_claims) 

        # These become meaningful once negotiation is implemented. For now, use zero/default timesteps
        msg.last_interaction_attempt_stamp.sec = 0
        msg.last_interaction_attempt_stamp.nanosec = 0
        msg.last_successful_interaction_stamp.sec = 0
        msg.last_successful_interaction_stamp.nanosec = 0

        msg.protocol_version = PROTOCOL_VERSION

        self.peer_state_pub.publish(msg)

    # Function for receiving PeerState messages from peers
    def _peer_state_received(self, msg: PeerState, peer_id: str) -> None:
        """Store an incoming PeerState heartbeat from a configured peer."""
        if peer_id not in self.peer_info:
            self.get_logger().warn(
                f"Received PeerState from unknown peer_id={peer_id}; ignoring"
            )
            return

        if msg.robot_id != peer_id:
            self.get_logger().warn(
                f"Ignoring PeerState for expected peer_id={peer_id}: "
                f"message robot_id={msg.robot_id}"
            )
            return

        if msg.protocol_version != PROTOCOL_VERSION:
            self.get_logger().warn(
                f"Protocol version mismatch from peer_id={peer_id}: "
                f"theirs={msg.protocol_version}, ours={PROTOCOL_VERSION}. Ignoring"
            )
            return

        info = self.peer_info[peer_id]
        info.last_state = msg
        info.last_received_ns = self.get_clock().now().nanoseconds
        info.was_fresh = True  # reset staleness on new message

        self.peer_claims[peer_id] = list(msg.claimed_frontiers)

        # Eager expiry is intentional for v1 simplicity; this can become lazy if claim counts grow 
        self._expire_stale_claims()
        self._resolve_own_claim_conflicts()

        self.get_logger().debug(
            f"Received PeerState from {peer_id} at ns={info.last_received_ns}; "
            f"stored {len(self.peer_claims[peer_id])} peer claims"
        )

    # Function for checking peer freshness and deciding whether to negotiate
    def _peer_is_fresh(self, peer_id: str) -> bool:
        """Check if the last received PeerState from a peer is still fresh (i.e. within timeout)

        Return True if peer has published a recent heartbeat
        """
        info = self.peer_info.get(peer_id)

        if info is None or info.last_state is None:
            return False
        
        age_sec = (
            self.get_clock().now().nanoseconds - info.last_received_ns
        ) / 1e9
        return age_sec <= self.peer_timeout_sec

    # Helper functions for freshness
    def _fresh_peer_ids(self) -> list[str]:
        """Return configured peers whose latest heartbeat is still fresh."""
        return [
            peer_id for peer_id in self.peer_ids if self._peer_is_fresh(peer_id)
        ]

    def _stale_peer_ids(self) -> list[str]:
        """Return configured peers with missing or expired heartbeats."""
        return [
            peer_id for peer_id in self.peer_ids if not self._peer_is_fresh(peer_id)
        ]

    def _handle_peer_staleness(self) -> None:
        """Drop claims from peers whose heartbeat has become stale.

        Supervisor decision: Option B.
        If a peer's heartbeat times out, all claims from that peer are treated
        as stale immediately. This supports graceful degradation under comms loss:
        the local robot falls back to its own frontier observations instead of
        preserving possibly-dead peer claims.
        """
        now_ns = self.get_clock().now().nanoseconds

        for peer_id in self.peer_ids:
            info = self.peer_info.get(peer_id)
            is_fresh = self._peer_is_fresh(peer_id)

            if is_fresh:
                if info is not None:
                    info.was_fresh = True  # update was_fresh for next time
                continue

            # Only log/drop on transition from fresh -> stale
            if info is not None and info.was_fresh:
                dropped_claims = len(self.peer_claims.get(peer_id, []))
                self.peer_claims[peer_id] = []  # drop all claims from this peer
                info.was_fresh = False  # update was_fresh for next time

                age_sec = (
                    (now_ns - info.last_received_ns) / 1e9 if info.last_received_ns > 0 else float('inf')
                )

                self.get_logger().warn(
                    "Peer became stale; dropping peer claims | "
                    f"peer_id={peer_id}; "
                    f"age_sec={age_sec:.2f}; "
                    f"peer_timeout_sec={self.peer_timeout_sec:.2f}; "
                    f"dropped_claims={dropped_claims}"
                )


    # Functions for claim management and conflict resolution
    def _claim_stamp_ns(self, claim: ClaimedFrontier) -> int:
        """Convert a claim timestamp to nanoseconds."""
        return (
            int(claim.claim_stamp.sec) * 1_000_000_000
            + int(claim.claim_stamp.nanosec)
        )

    def _claim_is_fresh(self, claim: ClaimedFrontier) -> bool:
        """Return True if a claim is still within claim_timeout_sec."""
        claim_ns = self._claim_stamp_ns(claim)
        now_ns = self.get_clock().now().nanoseconds
        age_sec = (now_ns - claim_ns) / 1e9
        return age_sec <= self.claim_timeout_sec 

    def _expire_stale_claims(self) -> None:
        """Remove expired own and peer claims. Eager expiry is intentional for v1 simplicity; this can become lazy if claim counts grow."""
        self.own_claims = [claim for claim in self.own_claims if self._claim_is_fresh(claim)]

        for peer_id, claims in self.peer_claims.items():
            self.peer_claims[peer_id] = [claim for claim in claims if self._claim_is_fresh(claim)]

    def _same_frontier_position(self, a: Point3, b: Point3) -> bool:
        """Return True if two frontier positions refer to the same frontier."""
        return distance_xy(a, b) <= FRONTIER_MATCH_TOLERANCE

    def _claim_wins_against(self, a: ClaimedFrontier, b: ClaimedFrontier) -> bool:
        """Return True if claim a wins over claim b for the same frontier.

        Conflict rule:
            1. Earlier claim_stamp wins.
            2. If timestamps tie, lexicographically smaller claimed_by wins.
        """
        a_ns = self._claim_stamp_ns(a)
        b_ns = self._claim_stamp_ns(b)

        if a_ns != b_ns:
            return a_ns < b_ns  # earlier timestamp wins

        return a.claimed_by < b.claimed_by  # tie-breaker: lex (string comparison of robot IDs) smaller wins

    def _resolve_own_claim_conflicts(self) -> None:
        """Drop own claims when a peer has a winning claim for the same frontier."""
        surviving_claims: list[ClaimedFrontier] = []
        dropped_details: list[str] = []

        for own_claim in self.own_claims:
            own_point = point_msg_to_tuple(own_claim.position)
            winning_peer_claim: ClaimedFrontier | None = None

            for peer_claim_list in self.peer_claims.values():
                for peer_claim in peer_claim_list:
                    peer_point = point_msg_to_tuple(peer_claim.position)

                    if not self._same_frontier_position(own_point, peer_point):
                        continue  # not the same frontier, skip
                    
                    if self._claim_wins_against(peer_claim, own_claim):
                        winning_peer_claim = peer_claim
                        break  # no need to check other peer claims for this frontier

                if winning_peer_claim is not None:
                    break  # no need to check other peers

            if winning_peer_claim is None:
                surviving_claims.append(own_claim)
            else:
                dropped_details.append(
                    "own_claim="
                    f"({own_point[0]:.2f}, {own_point[1]:.2f}) "
                    f"lost_to={winning_peer_claim.claimed_by} "
                    f"peer_claim_stamp={self._claim_stamp_ns(winning_peer_claim)} "
                    f"own_claim_stamp={self._claim_stamp_ns(own_claim)}"
                )

        dropped = len(self.own_claims) - len(surviving_claims)
        self.own_claims = surviving_claims

        if dropped > 0:
            self.get_logger().warn(
                "Dropped own claim(s) due to deterministic peer conflict resolution | "
                f"dropped={dropped}; details={dropped_details}"
            )

    # Helper functions for frontier matching 
    def _frontier_blocked_by_peer_claim(self, frontier: Point3) -> bool:
        """Return True if a frontier is already claimed by a peer."""
        for claims in self.peer_claims.values():
            for claim in claims:
                claim_point = point_msg_to_tuple(claim.position)
                if self._same_frontier_position(frontier, claim_point):
                    return True
        return False

    def _available_local_frontiers(self) -> list[Point3]:
        """Return local frontiers not blocked by fresh peer claims."""
        self._handle_peer_staleness()  # ensure we are not blocking on stale peers
        self._expire_stale_claims()  # ensure we are checking against only fresh claims

        return [
            frontier for frontier in self.local_frontiers
            if not self._frontier_blocked_by_peer_claim(frontier)
        ]

    # Interim MDVRP claim generation. This is deliberately not the final request/response protocol. It lets us create own_claims from the same MDVRP solver that the centralised CFPA2 mode uses, so claim broadcasting and conflict resolution can be tested before negotiation messages exist
    def _point3_to_claim(self, point: Point3, *, information_gain: float = 0.0) -> ClaimedFrontier:
        """Convert a frontier point into an owned ClaimedFrontier message."""
        claim = ClaimedFrontier()
        
        claim.position = Point(
            x=float(point[0]),
            y=float(point[1]),
            z=float(point[2]),
        )
        claim.claimed_by = self.robot_id
        claim.claim_stamp = self.get_clock().now().to_msg()
        claim.information_gain = float(information_gain)

        return claim

    def _generate_mdvrp_own_claims(
        self,
        *,
        fresh_peers: list[str],
        available_frontiers: list[Point3],
    ) -> None:
        """Generate local own_claims using the reusable MDVRP adapter.

        This is an interim step only: the final decentralised implementation
        will place the resulting proposal inside NegotiationRequest and only
        commit it after the peer accepts. For now, we commit local own_claims
        directly so the heartbeat/claim/conflict pipeline can be tested.
        """
        if not self.enable_mdvrp_auto_claims:
            return

        if self._latest_pose is None:
            self.get_logger().debug("Skipping MDVRP auto-claim: no local odom yet")
            return
        
        if not fresh_peers:
            self.get_logger().debug("Skipping MDVRP auto-claim: no fresh peers")
            return

        if not available_frontiers:
            # No candidates means any old own claims should naturally expire, but we should not fabricate new ones
            self.get_logger().debug("Skipping MDVRP auto-claim: no available frontiers")
            return

        now_ns = self.get_clock().now().nanoseconds
        if self._last_negotiation_attempt_ns > 0:
            cooldown_age_sec = (now_ns - self._last_negotiation_attempt_ns) / 1e9
            if cooldown_age_sec < self.negotiation_cooldown_sec:
                return  # still in cooldown

        robot_poses: dict[str, Point3] = {
            self.robot_id: pose_msg_to_tuple(self._latest_pose)
        }

        for peer_id in fresh_peers:
            info = self.peer_info.get(peer_id)
            if info is None or info.last_state is None:
                continue  # should not happen since we check for fresh peers, but be defensive
            robot_poses[peer_id] = pose_msg_to_tuple(info.last_state.pose)

        if len(robot_poses) <= 1:
            self.get_logger().debug(
                "Skipping MDVRP auto-claim: no usable fresh peer poses"
            )
            return

        assignment = solve_frontier_assignment(
            robot_poses=robot_poses,
            candidate_frontiers=available_frontiers,
            time_limit_sec=self.mdvrp_time_limit_sec,
            span_cost_coefficient=self.mdvrp_span_cost_coefficient,
        )

        assigned_to_self = assignment.get(self.robot_id, [])
        
        self.own_claims = [
            self._point3_to_claim(frontier) for frontier in assigned_to_self
        ]

        self._resolve_own_claim_conflicts()  # resolve against peer claims in case of overlap
        self._last_negotiation_attempt_ns = now_ns  # start cooldown

        self.get_logger().info(
            "MDVRP auto-claim proposal generated | "
            f"robots={sorted(robot_poses.keys())} "
            f"candidate_frontiers={len(available_frontiers)} "
            f"own_claims={len(self.own_claims)}"
        )

    # Function for negotiation logic 
    def _decide_negotiation(self) -> None:
        """Decide whether to initiate negotiation with a peer. Stub"""
        fresh_peers = self._fresh_peer_ids()
        stale_peers = self._stale_peer_ids()
        available_frontiers = self._available_local_frontiers()

        self._generate_mdvrp_own_claims(
            fresh_peers=fresh_peers,
            available_frontiers=available_frontiers,
        )

        if not self._logged_negotiation_stub:
            self.get_logger().info(
                "_decide_negotiation stub reached; "
                f"fresh_peers={fresh_peers}; "
                f"stale_peers={stale_peers}; "
                f"local_frontiers={len(self.local_frontiers)}; "
                f"available_frontiers={len(available_frontiers)}; "
                "negotiation logic not yet implemented"
            )
            self._logged_negotiation_stub = True
        
        self.get_logger().debug(
            f"Fresh peers: {fresh_peers}; "
            f"Stale peers: {stale_peers}; "
            f"local_frontiers={len(self.local_frontiers)}; "
            f"available_frontiers={len(available_frontiers)}"
        )

def main(args=None) -> None:
    rclpy.init(args=args)
    node = PeerCoordinatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()

