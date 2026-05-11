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
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry

from cfpa2_peer_coordination.mdvrp_adapter import (
    Point3,
    distance_xy,
    point_msg_to_tuple,
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


class PeerCoordinatorNode(Node):
    """Per-robot peer coordinator. One instance per robot."""

    def __init__(self) -> None:
        super().__init__("cfpa2_peer_coordinator")

        # Parameters
        self.declare_parameter("robot_id", "robot_a")
        self.declare_parameter("robot_namespace", "robot_a")

        # assumption: peer_id == peer_namespace. Used for both protocol identity and topic addressing. Split into separate parameters if this assumption ever needs to break
        self.declare_parameter("peer_namespaces", ["robot_b"])

        self.declare_parameter("peer_timeout_sec", 5.0)
        self.declare_parameter("claim_timeout_sec", 30.0)
        self.declare_parameter("peer_state_rate_hz", 2.0)
        self.declare_parameter("negotiation_rate_hz", 1.0)
        self.declare_parameter("negotiation_cooldown_sec", 2.0)

        self.declare_parameter("odom_topic_suffix", "/odom/nav")
        self.declare_parameter("frontier_markers_topic", "/mtare/frontier_markers")

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
                lambda msg, pid=peer_id: self._peer_state_received(msg, pid),
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

        # self.get_logger().debug(
        #     f"_publish_peer_state: would publish {msg.robot_id} "
        #     f"v{msg.protocol_version}"
        # )

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

        for own_claim in self.own_claims:
            own_point = point_msg_to_tuple(own_claim.position)
            peer_wins = False

            for peer_claim_list in self.peer_claims.values():
                for peer_claim in peer_claim_list:
                    peer_point = point_msg_to_tuple(peer_claim.position)

                    if not self._same_frontier_position(own_point, peer_point):
                        continue  # not the same frontier, skip
                    
                    if self._claim_wins_against(peer_claim, own_claim):
                        peer_wins = True
                        break  # no need to check other peer claims for this frontier

                if peer_wins:
                    break  # no need to check other peers

            if not peer_wins:
                surviving_claims.append(own_claim)

        dropped = len(self.own_claims) - len(surviving_claims)
        self.own_claims = surviving_claims

        if dropped > 0:
            self.get_logger().warn(
                f"Dropped {dropped} own claim(s) due to deterministic peer conflict resolution"
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
        self._expire_stale_claims()  # ensure we are checking against only fresh claims

        return [
            frontier for frontier in self.local_frontiers
            if not self._frontier_blocked_by_peer_claim(frontier)
        ]

    # Function for negotiation logic 
    def _decide_negotiation(self) -> None:
        """Decide whether to initiate negotiation with a peer. Stub"""
        fresh_peers = self._fresh_peer_ids()
        stale_peers = self._stale_peer_ids()
        available_frontiers = self._available_local_frontiers()

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

