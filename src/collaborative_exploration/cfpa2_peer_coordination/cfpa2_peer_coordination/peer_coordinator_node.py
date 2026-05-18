"""Decentralised peer coordinator node.

Each robot runs one instance of this node. It broadcasts this robot's PeerState heartbeat and negotiates frontier ownership with peers via NegotiationRequest/NegotiationResponse. Claims are committed only on negotiated accept; there is no unilateral claim path.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from visualization_msgs.msg import MarkerArray 

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy, QoSProfile
from rclpy.publisher import Publisher

from std_msgs.msg import Header
from geometry_msgs.msg import Point, Pose, PoseArray
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
    NegotiationRequest,      
    NegotiationResponse,     
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
BLOCKED_FRONTIERS_TOPIC = "cfpa2_peer_coordination/blocked_frontiers"

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

# QoS profile for negotiation request/response messages: reliable.
# These are point-to-point protocol messages where dropped delivery causes correctness issues (the requester would hang waiting for a response that never arrives). Reliable + KEEP_LAST gives us at-least-once delivery with bounded queueing.
NEGOTIATION_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    # Volatile (not transient_local) because late-joining subscribers don't need to receive old requests (stale)
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    # depth=10 because negotiation messages are infrequent but should queue if the receiver is briefly slow. 
    depth=10,   
)

# Negotiation state machine states for the requester role
REQUESTER_IDLE = "IDLE"
REQUESTER_REQUESTING = "REQUESTING"

# Negotiation responder outcomes / reject reasons
ACCEPT_REASON = "accepted"
REJECT_CROSSING_LOST = "crossing_lost"  # simultaneous request collision; this robot lost the deterministic tie-break
REJECT_LOCAL_SOLVE_MISMATCH = "local_solve_mismatch"  # proposal differs from responder's local MDVRP solve
REJECT_CLAIM_OVERLAP = "claim_overlap"  # proposed responder claims partially overlap existing committed own claims
REJECT_FRONTIER_UNKNOWN_LOCALLY = "frontier_unknown_locally"  # proposal references a frontier not observed by responder
REJECT_PROTOCOL_ERROR = "protocol_error"  # malformed well-addressed request
REJECT_NO_LOCAL_EXPECTED_PROPOSAL = "no_local_expected_proposal"  # responder lacks enough local state to compute an expected proposal

@dataclass
class PeerInfo:
    """Tracks per-peer state for the local coordinator.

    Extended over the project: currently holds the last received PeerState and the local timestamp it arrived at. Future fields will include claim tracking and negotiation-state machines."""

    last_state: PeerState | None = None
    last_received_ns: int = 0
    was_fresh: bool = False

@dataclass
class RequesterState:
    """Per-peer requester-side negotiation state.

    Tracks one in-flight proposal from this robot to a specific peer. Claims are not committed until the peer accepts the request.
    """
    state: str = REQUESTER_IDLE
    in_flight_request_id: str | None = None
    request_sent_ns: int = 0
    proposed_own_claims: list[ClaimedFrontier] = field(default_factory=list)
    proposed_peer_claims: list[ClaimedFrontier] = field(default_factory=list)
    consecutive_rejects: int = 0
    backoff_until_ns: int = 0

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

        self.declare_parameter("request_timeout_sec", 2.0)
        self.declare_parameter("request_reject_backoff_sec", 2.0)
        self.declare_parameter("request_timeout_backoff_sec", 1.0)
        self.declare_parameter("enable_negotiation_requests", True)

        self.declare_parameter("odom_topic_suffix", "/odom/nav")
        self.declare_parameter("frontier_markers_topic", "/mtare/frontier_markers")
        self.declare_parameter("blocked_frontiers_rate_hz", 1.0)

        # Interim milestone parameters
        # This is NOT the final request/response protocol yet. It lets the node generate local own_claims from the shared MDVRP solver so claim broadcast and conflict resolution can be tested before full negotiation exists
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

        self.request_timeout_sec: float = float(
            self.get_parameter("request_timeout_sec").value
        )
        self.request_reject_backoff_sec: float = float(
            self.get_parameter("request_reject_backoff_sec").value
        )
        self.request_timeout_backoff_sec: float = float(
            self.get_parameter("request_timeout_backoff_sec").value
        )
        self.enable_negotiation_requests: bool = bool(
            self.get_parameter("enable_negotiation_requests").value
        )

        self.odom_topic_suffix: str = self.get_parameter("odom_topic_suffix").value
        self.frontier_markers_topic: str = str(
            self.get_parameter("frontier_markers_topic").value
        )
        self.blocked_frontiers_rate_hz: float = float(
            self.get_parameter("blocked_frontiers_rate_hz").value
        )

        # MDVRP solver parameter readback
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
        """local_frontiers: updated from CFPA2 frontier MarkerArray
            own_claims: committed only on negotiated accept (responder or requester path)
            peer_claims: updated from received PeerState messages AND from negotiated accept of the corresponding peer (immediate cache)
        """
        self.local_frontiers: list[Point3] = []
        self.own_claims: list[ClaimedFrontier] = []
        self.peer_claims: dict[str, list[ClaimedFrontier]] = {
            peer_id: [] for peer_id in self.peer_ids
        }

        # Cooldown for interim MDVRP auto-claim generation
        # Updated when this robot sends a NegotiationRequest
        self._last_negotiation_attempt_ns: int = 0

        # Request ID counter for negotiation protocol
        # Format: f"{robot_id}-{counter}" — see _next_request_id()
        self._request_counter: int = 0

        # Per-peer requester-side negotiation state
        self.requester_states: dict[str, RequesterState] = {
            peer_id: RequesterState() for peer_id in self.peer_ids
        }

        # Interaction timestamps exposed in PeerState
        self._last_interaction_attempt_ns: int = 0
        self._last_successful_interaction_ns: int = 0

        # Topic names
        self.own_peer_state_topic = f"/{self.robot_namespace}/{PEER_STATE_TOPIC}"

        self.own_blocked_frontiers_topic = (
            f"/{self.robot_namespace}/{BLOCKED_FRONTIERS_TOPIC}"
        )

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

        self.blocked_frontiers_pub = self.create_publisher(
            PoseArray,
            self.own_blocked_frontiers_topic,
            1,  # can make a named QoS profile later if needed
        )

        # Negotiation publishers: send request/response to peer inboxes
        # Each peer gets its own publisher because requests are point-to-point
        self.peer_request_pubs: dict[str, Publisher] = {}
        self.peer_response_pubs: dict[str, Publisher] = {}

        for peer_id, peer_ns in zip(self.peer_ids, self.peer_namespaces):
            request_topic = self.peer_request_inbox_topics[peer_id]
            response_topic = self.peer_response_inbox_topics[peer_id]

            self.peer_request_pubs[peer_id] = self.create_publisher(
                NegotiationRequest,
                request_topic,
                NEGOTIATION_QOS,
            )
            self.peer_response_pubs[peer_id] = self.create_publisher(
                NegotiationResponse,
                response_topic,
                NEGOTIATION_QOS,
            )

            self.get_logger().info(
                f"Created negotiation publishers for peer {peer_id}: "
                f"request -> {request_topic}, response -> {response_topic}"
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

        # Negotiation subscribers: receive requests/responses addressed to this robot
        self.own_request_sub = self.create_subscription(
            NegotiationRequest,
            self.own_request_inbox_topic,
            self._negotiation_request_received, 
            NEGOTIATION_QOS,
        )
        self.own_response_sub = self.create_subscription(
            NegotiationResponse,
            self.own_response_inbox_topic,
            self._negotiation_response_received,
            NEGOTIATION_QOS,
        )
        self.get_logger().info(
            f"Subscribed to own negotiation inbox: "
            f"request <- {self.own_request_inbox_topic}, "
            f"response <- {self.own_response_inbox_topic}"
        )

        # Timers (stubs only for now)
        peer_state_period = 1.0 / max(self.peer_state_rate_hz, 1e-6)
        negotiation_period = 1.0 / max(self.negotiation_rate_hz, 1e-6)
        blocked_frontiers_period = 1.0 / max(self.blocked_frontiers_rate_hz, 1e-6)

        self.peer_state_timer = self.create_timer(
            peer_state_period, 
            self._publish_peer_state,
        )
        self.negotiation_timer = self.create_timer(
            negotiation_period, 
            self._decide_negotiation,
        )
        self.blocked_frontiers_timer = self.create_timer(
            blocked_frontiers_period,
            self._publish_blocked_frontiers,
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
            f"Publishing blocked frontiers on {self.own_blocked_frontiers_topic} "
            f"at {self.blocked_frontiers_rate_hz:.2f} Hz"
        )
        self.get_logger().info(
            f"Subscribed peer PeerState topics: {self.peer_state_topics}"
        )
        self.get_logger().info(
            "Negotiation inbox topics wired | "
            f"request_in={self.own_request_inbox_topic} "
            f"response_in={self.own_response_inbox_topic}"
        )
        self.get_logger().info(
            f"Peer negotiation publishers wired | "
            f"requests_to={list(self.peer_request_pubs.keys())} "
            f"responses_to={list(self.peer_response_pubs.keys())}"
        )
        self.get_logger().info(
            "MDVRP solver | "
            f"time_limit_sec={self.mdvrp_time_limit_sec:.2f}s "
            f"span_cost={self.mdvrp_span_cost_coefficient}"
        )
        self.get_logger().info(
            "Negotiation requester | "
            f"enabled={self.enable_negotiation_requests} "
            f"request_timeout_sec={self.request_timeout_sec:.2f} "
            f"cooldown_sec={self.negotiation_cooldown_sec:.2f}"
        )
    
    # Function for making message headers
    def _make_header(self) -> Header:
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"
        return header

    # Function for converting nanoseconds since epoch/ROS clock to builtin_interfaces/Time
    def _ns_to_time_msg(self, ns: int):
        msg = self.get_clock().now().to_msg()
        if ns <= 0:
            msg.sec = 0
            msg.nanosec = 0
            return msg

        msg.sec = int(ns // 1_000_000_000)
        msg.nanosec = int(ns % 1_000_000_000)
        return msg

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

        msg.last_interaction_attempt_stamp = self._ns_to_time_msg(
            self._last_interaction_attempt_ns
        )
        msg.last_successful_interaction_stamp = self._ns_to_time_msg(
            self._last_successful_interaction_ns
        )

        msg.protocol_version = PROTOCOL_VERSION

        self.peer_state_pub.publish(msg)

    # Function for publishing blocked frontiers
    def _publish_blocked_frontiers(self) -> None:
        """Publish peer-claimed frontier positions for the local planner to avoid."""
        self._expire_stale_claims()

        msg = PoseArray()
        msg.header = self._make_header()

        for claims in self.peer_claims.values():
            for claim in claims:
                pose = Pose()
                pose.position.x = float(claim.position.x)
                pose.position.y = float(claim.position.y)
                pose.position.z = float(claim.position.z)
                pose.orientation.w = 1.0  # identity orientation; we only care about position
                msg.poses.append(pose)

        self.blocked_frontiers_pub.publish(msg)

        self.get_logger().debug(
            f"Published {len(msg.poses)} blocked frontier(s)"
        )

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
        If a peer's heartbeat times out, all claims from that peer are treated as stale immediately. This supports graceful degradation under comms loss:
        the local robot falls back to its own frontier observations instead of preserving possibly-dead peer claims.
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

                requester_state = self.requester_states.get(peer_id)
                if requester_state is not None:
                    was_in_flight = requester_state.state == REQUESTER_REQUESTING
                    in_flight_id = requester_state.in_flight_request_id
                    self.requester_states[peer_id] = RequesterState()
                    if was_in_flight:
                        self.get_logger().warn(
                            "Peer became stale while negotiation request was in-flight; "
                            "cancelling requester state | "
                            f"peer_id={peer_id}; "
                            f"request_id={in_flight_id}"
                        )
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
            
    # Functions for negotiation chunk C: responder validation + accept/reject
    def _claims_match_within_tolerance(self, a: list[ClaimedFrontier], b: list[ClaimedFrontier]) -> bool:
        """Return True if two claim lists refer to the same 2D frontier set.

        Uses FRONTIER_MATCH_TOLERANCE rather than mm-rounded equality, so independently generated requester/responder proposals can still match despite small floating point or projection differences.
        """
        if len(a) != len(b):
            return False

        # copy b to a mutable list so we can mark off matches without modifying the original
        unmatched_b = list(b)

        for claim_a in a:
            point_a = point_msg_to_tuple(claim_a.position)
            matched_idx: int | None = None

            for idx, claim_b in enumerate(unmatched_b):
                point_b = point_msg_to_tuple(claim_b.position)
                if self._same_frontier_position(point_a, point_b):
                    matched_idx = idx
                    break

            if matched_idx is None:
                return False  # claim_a has no match in b, so sets don't match

            unmatched_b.pop(matched_idx)  # remove matched claim from consideration

        return True  # all claims in a have a match in b, and lengths are equal, so sets match

    def _claim_position_known_locally(self, claim: ClaimedFrontier) -> bool:
        """Return True if a claim position matches one of this robot's local frontiers."""
        claim_point = point_msg_to_tuple(claim.position)

        for frontier in self.local_frontiers:
            if self._same_frontier_position(claim_point, frontier):
                return True # claim matches a currently observed local frontier

        return False  # claim does not match any currently observed local frontier

    def _responder_local_resolve(self, msg: NegotiationRequest) -> tuple[list[ClaimedFrontier], list[ClaimedFrontier]] | None:
        """Re-run MDVRP locally from the responder's view.

        Returns expected (requester_claims, responder_claims), or None if this
        robot cannot form a local expected proposal.
        """
        if self._latest_pose is None:
            self.get_logger().debug(
                "Responder validation cannot re-solve: no local odom yet"
            )
            return None

        if not self._peer_is_fresh(msg.requester_id):
            self.get_logger().debug(
                "Responder validation cannot re-solve: requester heartbeat stale | "
                f"requester_id={msg.requester_id}"
            )
            return None

        if not self.local_frontiers:
            self.get_logger().debug(
                "Responder validation cannot re-solve: no local frontiers"
            )
            return None

        info = self.peer_info.get(msg.requester_id)
        if info is None or info.last_state is None:
            self.get_logger().debug(
                "Responder validation cannot re-solve: no requester PeerState | "
                f"requester_id={msg.requester_id}"
            )
            return None

        robot_poses: dict[str, Point3] = {
            self.robot_id: pose_msg_to_tuple(self._latest_pose),
            msg.requester_id: pose_msg_to_tuple(info.last_state.pose)
        }

        assignment = solve_frontier_assignment(
            robot_poses=robot_poses,
            candidate_frontiers=list(self.local_frontiers),
            time_limit_sec=self.mdvrp_time_limit_sec,
            span_cost_coefficient=self.mdvrp_span_cost_coefficient,
        )

        requester_points = assignment.get(msg.requester_id, [])
        responder_points = assignment.get(self.robot_id, [])

        expected_requester_claims = [
            self._point3_to_claim_for(point, claimed_by=msg.requester_id)
            for point in requester_points
        ]
        expected_responder_claims = [
            self._point3_to_claim_for(point, claimed_by=self.robot_id)
            for point in responder_points
        ]

        if not expected_requester_claims and not expected_responder_claims:
            return None
        
        return expected_requester_claims, expected_responder_claims

    # Validator function (Chunk C)
    def _validate_negotiation_request(self, msg: NegotiationRequest) -> tuple[bool, str]:
        """Validate an incoming negotiation request.

        Protocol-level addressing/version checks are handled before this method.
        This method decides whether a well-addressed request should be accepted.
        """
        # 1. Crossing resolution
        own_requester_state = self.requester_states.get(msg.requester_id)
        if (own_requester_state is not None and own_requester_state.state == REQUESTER_REQUESTING):
            if self.robot_id < msg.requester_id:
                # Lexicographically smaller robot_id wins the crossing
                self.get_logger().info(
                    "Crossing negotiation detected; keeping own request and rejecting incoming | "
                    f"incoming_request_id={msg.request_id}; "
                    f"own_request_id={own_requester_state.in_flight_request_id}; "
                    f"this_robot={self.robot_id}; "
                    f"requester={msg.requester_id}"
                )
                return False, REJECT_CROSSING_LOST  
            
            # Lose crossing. Cancel own in-flight request, then evaluate incoming
            cancelled_request_id = own_requester_state.in_flight_request_id
            self.requester_states[msg.requester_id] = RequesterState()  # reset requester
            self.get_logger().info(
                "Crossing negotiation detected; cancelling own request and evaluating incoming | "
                f"cancelled_request_id={cancelled_request_id}; "
                f"incoming_request_id={msg.request_id}; "
                f"this_robot={self.robot_id}; "
                f"requester={msg.requester_id}"
            )

        # 2. claimed_by sanity
        for claim in msg.requester_claims:
            if claim.claimed_by != msg.requester_id:
                self.get_logger().warn(
                    "NegotiationRequest has requester claim assigned to wrong robot | "
                    f"request_id={msg.request_id}; "
                    f"claim.claimed_by={claim.claimed_by}; "
                    f"expected={msg.requester_id}"
                )
                return False, REJECT_PROTOCOL_ERROR

        for claim in msg.responder_claims:
            if claim.claimed_by != self.robot_id:
                self.get_logger().warn(
                    "NegotiationRequest has responder claim assigned to wrong robot | "
                    f"request_id={msg.request_id}; "
                    f"claim.claimed_by={claim.claimed_by}; "
                    f"expected={self.robot_id}"
                )
                return False, REJECT_PROTOCOL_ERROR

        # 3. frontier_unknown_locally
        for claim in list(msg.requester_claims) + list(msg.responder_claims):
            if not self._claim_position_known_locally(claim):
                claim_point = point_msg_to_tuple(claim.position)
                self.get_logger().warn(
                    "NegotiationRequest references frontier unknown locally | "
                    f"request_id={msg.request_id}; "
                    f"claim=({claim_point[0]:.2f}, {claim_point[1]:.2f}); "
                    f"claimed_by={claim.claimed_by}"
                )
                return False, REJECT_FRONTIER_UNKNOWN_LOCALLY

        # 4. claim_overlap
        # Re-committing exactly the same responder claim set is allowed
        if not self._claims_match_within_tolerance(list(msg.responder_claims), self.own_claims):
            for proposed in msg.responder_claims:
                proposed_point = point_msg_to_tuple(proposed.position)

                for existing in self.own_claims:
                    existing_point = point_msg_to_tuple(existing.position)

                    if self._same_frontier_position(proposed_point, existing_point):
                        self.get_logger().warn(
                            "NegotiationRequest overlaps existing responder claim | "
                            f"request_id={msg.request_id}; "
                            f"overlap=({proposed_point[0]:.2f}, {proposed_point[1]:.2f})"
                        )
                        return False, REJECT_CLAIM_OVERLAP

        # 5.. local_solve_mismatch
        local_resolve = self._responder_local_resolve(msg)
        if local_resolve is None:
            return False, REJECT_NO_LOCAL_EXPECTED_PROPOSAL

        expected_requester_claims, expected_responder_claims = local_resolve

        requester_matches = self._claims_match_within_tolerance(list(msg.requester_claims), expected_requester_claims)
        
        responder_matches = self._claims_match_within_tolerance(list(msg.responder_claims), expected_responder_claims)

        if not requester_matches or not responder_matches:
            self.get_logger().warn(
                "NegotiationRequest rejected: local MDVRP solve mismatch | "
                f"request_id={msg.request_id}; "
                f"requester_claims={len(msg.requester_claims)} "
                f"expected_requester_claims={len(expected_requester_claims)}; "
                f"responder_claims={len(msg.responder_claims)} "
                f"expected_responder_claims={len(expected_responder_claims)}"
            )
            return False, REJECT_LOCAL_SOLVE_MISMATCH

        # Passed all checks, accept request
        return True, ACCEPT_REASON  

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

    def _point3_to_claim_for(self, point: Point3, *, claimed_by: str, information_gain: float = 0.0) -> ClaimedFrontier:
        """Convert a frontier point into an owned ClaimedFrontier message."""
        claim = ClaimedFrontier()
        
        claim.position = Point(
            x=float(point[0]),
            y=float(point[1]),
            z=float(point[2]),
        )
        claim.claimed_by = claimed_by
        claim.claim_stamp = self.get_clock().now().to_msg()
        claim.information_gain = float(information_gain)

        return claim

    # Negotiation Chunk A: request/response inbox wiring
    def _next_request_id(self) -> str:
        """Generate a unique request ID for negotiation.

        Format: <robot_id>-<counter>. Per-robot scoping prevents collisions
        even if multiple robots increment their counters in lockstep.
        """
        self._request_counter += 1
        return f"{self.robot_id}-{self._request_counter}"

    def _negotiation_request_received(self, msg: NegotiationRequest) -> None:
        """Receive a NegotiationRequest from a peer"""
        if msg.protocol_version != PROTOCOL_VERSION:
            self.get_logger().warn(
                f"Ignoring NegotiationRequest with protocol mismatch: "
                f"request_id={msg.request_id}; "
                f"theirs={msg.protocol_version}; ours={PROTOCOL_VERSION}"
            )
            return

        if msg.responder_id != self.robot_id:
            self.get_logger().warn(
                f"Ignoring NegotiationRequest not addressed to this robot: "
                f"request_id={msg.request_id}; "
                f"requester_id={msg.requester_id}; "
                f"responder_id={msg.responder_id}; "
                f"this_robot={self.robot_id}"
            )
            return

        if msg.requester_id not in self.peer_ids:
            self.get_logger().warn(
                f"Ignoring NegotiationRequest from unknown requester: "
                f"request_id={msg.request_id}; requester_id={msg.requester_id}"
            )
            return

        # Chunk C responder behaviour
        accepted, reason = self._validate_negotiation_request(msg)

        response = NegotiationResponse()
        response.header = self._make_header()
        response.request_id = msg.request_id
        response.requester_id = msg.requester_id
        response.responder_id = self.robot_id
        response.response_stamp = self.get_clock().now().to_msg()
        response.accepted = accepted
        response.reason = reason
        response.responder_last_interaction_attempt_stamp = self._ns_to_time_msg(self._last_interaction_attempt_ns)
        response.protocol_version = PROTOCOL_VERSION

        if accepted:
            response.accepted_requester_claims = list(msg.requester_claims)
            response.accepted_responder_claims = list(msg.responder_claims)
        else:
            response.accepted_requester_claims = []
            response.accepted_responder_claims = []

        # Publish BEFORE local commit
        self.peer_response_pubs[msg.requester_id].publish(response)

        if accepted:
            # Chunk D: cache peer claims immediately so the requester-side suppression check in _tick_requester_state sees consistent state before the next PeerState heartbeat arrives
            self.own_claims = list(msg.responder_claims)
            self.peer_claims[msg.requester_id] = list(msg.requester_claims)
            self._last_successful_interaction_ns = self.get_clock().now().nanoseconds

        log_fn = self.get_logger().info if accepted else self.get_logger().warn
        log_fn(
            f"NegotiationRequest {'accepted' if accepted else 'rejected'} | "
            f"request_id={msg.request_id}; "
            f"from={msg.requester_id}; "
            f"requester_claims={len(msg.requester_claims)}; "
            f"responder_claims={len(msg.responder_claims)}; "
            f"reason='{reason}'"
        )

        self.get_logger().info(
            f"NegotiationResponse sent | "
            f"request_id={msg.request_id}; "
            f"to={msg.requester_id}; "
            f"accepted={accepted}; "
            f"reason='{reason}'"
        )

    def _negotiation_response_received(self, msg: NegotiationResponse) -> None:
        """Receive a NegotiationResponse from a peer"""
        if msg.protocol_version != PROTOCOL_VERSION:
            self.get_logger().warn(
                f"Ignoring NegotiationResponse with protocol mismatch: "
                f"request_id={msg.request_id}; "
                f"theirs={msg.protocol_version}; ours={PROTOCOL_VERSION}"
            )
            return

        if msg.requester_id != self.robot_id:
            self.get_logger().warn(
                f"Ignoring NegotiationResponse not addressed to this requester: "
                f"requester_id={msg.requester_id}; "
                f"responder_id={msg.responder_id}; "
                f"this_robot={self.robot_id}"
            )
            return

        if msg.responder_id not in self.peer_ids:
            self.get_logger().warn(
                f"Ignoring NegotiationResponse from unknown responder: "
                f"request_id={msg.request_id}; responder_id={msg.responder_id}"
            )
            return

        state = self.requester_states.get(msg.responder_id)
        if state is None:
            self.get_logger().warn(
                f"Ignoring NegotiationResponse with no requester state: "
                f"request_id={msg.request_id}; responder_id={msg.responder_id}"
            )
            return

        if state.state != REQUESTER_REQUESTING:
            self.get_logger().warn(
                f"Ignoring NegotiationResponse while not REQUESTING: "
                f"request_id={msg.request_id}; "
                f"responder_id={msg.responder_id}; "
                f"state={state.state}"
            )
            return

        if msg.request_id != state.in_flight_request_id:
            self.get_logger().warn(
                f"Ignoring NegotiationResponse for stale/unknown request_id: "
                f"received={msg.request_id}; "
                f"expected={state.in_flight_request_id}; "
                f"responder_id={msg.responder_id}"
            )
            return

        now_ns = self.get_clock().now().nanoseconds

        if msg.accepted:
            if not self._claims_match_within_tolerance(list(msg.accepted_requester_claims), state.proposed_own_claims) or not self._claims_match_within_tolerance(list(msg.accepted_responder_claims), state.proposed_peer_claims):
                state.consecutive_rejects += 1

                self.get_logger().warn(
                    "Ignoring accepted NegotiationResponse because accepted claims "
                    "do not match in-flight proposal | "
                    f"request_id={msg.request_id}; "
                    f"peer_id={msg.responder_id}"
                    f"consecutive_rejects={state.consecutive_rejects}"
                )

                state.state = REQUESTER_IDLE
                state.in_flight_request_id = None
                state.request_sent_ns = 0
                state.proposed_own_claims = []
                state.proposed_peer_claims = []
                state.backoff_until_ns = now_ns + int(self.request_reject_backoff_sec * 1e9)
                return

            # Chunk D: cache peer claims immediately so future suppression checks have consistent state before next PeerState heartbeat
            self.own_claims = list(msg.accepted_requester_claims)
            self.peer_claims[msg.responder_id] = list(msg.accepted_responder_claims)

            state.state = REQUESTER_IDLE
            state.in_flight_request_id = None
            state.request_sent_ns = 0
            state.proposed_own_claims = []
            state.proposed_peer_claims = []
            state.consecutive_rejects = 0
            state.backoff_until_ns = now_ns + int(self.negotiation_cooldown_sec * 1e9)

            self._last_successful_interaction_ns = now_ns

            self.get_logger().info(
                "Negotiation accepted; committed own claims | "
                f"request_id={msg.request_id}; "
                f"peer_id={msg.responder_id}; "
                f"own_claims={len(self.own_claims)}"
            )
        else:
            state.consecutive_rejects += 1
            backoff_sec = self.request_reject_backoff_sec * min(4, 2 ** max(0, state.consecutive_rejects - 1))  # exponential backoff with a cap

            state.state = REQUESTER_IDLE
            state.in_flight_request_id = None
            state.request_sent_ns = 0
            state.proposed_own_claims = []
            state.proposed_peer_claims = []
            state.backoff_until_ns = now_ns + int(backoff_sec * 1e9)

            self.get_logger().warn(
                "Negotiation rejected | "
                f"request_id={msg.request_id}; "
                f"peer_id={msg.responder_id}; "
                f"reason='{msg.reason}'; "
                f"consecutive_rejects={state.consecutive_rejects}; "
                f"backoff_sec={backoff_sec:.2f}"
            )

    # Negotiation Chunk B: Requester State Machine
    def _tick_requester_state(self, *, fresh_peers: list[str], candidate_frontiers: list[Point3]) -> None:
        """Drive requester-side negotiation state machines.

        Implements requester-side; responder side in Chunk C
        """
        if not self.enable_negotiation_requests:
            return

        now_ns = self.get_clock().now().nanoseconds
        request_timeout_ns = int(self.request_timeout_sec * 1e9)

        for peer_id in fresh_peers:
            state = self.requester_states[peer_id]

            if state.state == REQUESTER_REQUESTING:
                age_ns = now_ns - state.request_sent_ns
                if age_ns > request_timeout_ns:
                    self.get_logger().warn(
                        "Negotiation request timed out | "
                        f"peer_id={peer_id}; "
                        f"request_id={state.in_flight_request_id}; "
                        f"age_sec={age_ns / 1e9:.2f}; "
                        f"timeout_sec={self.request_timeout_sec:.2f}"
                    )

                    state.state = REQUESTER_IDLE
                    state.in_flight_request_id = None
                    state.request_sent_ns = 0
                    state.proposed_own_claims = []
                    state.proposed_peer_claims = []
                    state.backoff_until_ns = now_ns + int(self.request_timeout_backoff_sec * 1e9)
                continue  # only check timeouts while in REQUESTING state

            if state.state != REQUESTER_IDLE:
                self.get_logger().warn(
                    f"Unknown requester state for peer_id={peer_id}: {state.state}; resetting"
                )
                self.requester_states[peer_id] = RequesterState()
                continue

            if now_ns < state.backoff_until_ns:
                continue

            if self._last_negotiation_attempt_ns > 0:
                cooldown_age_sec = (now_ns - self._last_negotiation_attempt_ns) / 1e9
                if cooldown_age_sec < self.negotiation_cooldown_sec:
                    continue  

            proposal = self._build_negotiation_proposal(peer_id, candidate_frontiers)
            if proposal is None:
                continue

            proposed_own_claims, proposed_peer_claims = proposal

            # Avoid repeatedly negotiating the same already-committed allocation
            if self._claims_match_within_tolerance(proposed_own_claims, self.own_claims) and self._claims_match_within_tolerance(proposed_peer_claims, self.peer_claims.get(peer_id, [])):
                continue

            request_id = self._next_request_id()

            msg = NegotiationRequest()
            msg.header = self._make_header()
            msg.request_id = request_id
            msg.requester_id = self.robot_id
            msg.responder_id = peer_id
            msg.request_stamp = self.get_clock().now().to_msg()
            msg.requester_claims = list(proposed_own_claims)
            msg.responder_claims = list(proposed_peer_claims)
            msg.protocol_version = PROTOCOL_VERSION

            self.peer_request_pubs[peer_id].publish(msg)

            state.state = REQUESTER_REQUESTING
            state.in_flight_request_id = request_id
            state.request_sent_ns = now_ns
            state.proposed_own_claims = list(proposed_own_claims)
            state.proposed_peer_claims = list(proposed_peer_claims)

            self._last_negotiation_attempt_ns = now_ns 
            self._last_interaction_attempt_ns = now_ns

            self.get_logger().info(
                "NegotiationRequest sent | "
                f"peer_id={peer_id}; "
                f"request_id={request_id}; "
                f"own_claims={len(proposed_own_claims)}; "
                f"peer_claims={len(proposed_peer_claims)}"
            )

    # Function for building an MDVRP proposal for robot + peer
    def _build_negotiation_proposal(
        self,
        peer_id: str,
        candidate_frontiers: list[Point3],
    ) -> tuple[list[ClaimedFrontier], list[ClaimedFrontier]] | None:
        """Returns:
        (own_claims, peer_claims) if a proposal can be built,
        otherwise None."""
        if self._latest_pose is None:
            self.get_logger().debug(
                f"Skipping negotiation proposal for {peer_id}: no local odom yet"
            )
            return None
            
        if not self._peer_is_fresh(peer_id):
            return None

        if not candidate_frontiers:
            return None

        info = self.peer_info.get(peer_id)
        if info is None or info.last_state is None:
            return None

        robot_poses: dict[str, Point3] = {
            self.robot_id: pose_msg_to_tuple(self._latest_pose),
            peer_id: pose_msg_to_tuple(info.last_state.pose),
        }
        
        assignment = solve_frontier_assignment(
            robot_poses=robot_poses,
            candidate_frontiers=candidate_frontiers,
            time_limit_sec=self.mdvrp_time_limit_sec,
            span_cost_coefficient=self.mdvrp_span_cost_coefficient,
        )

        own_points = assignment.get(self.robot_id, [])
        peer_points = assignment.get(peer_id, [])

        proposed_own_claims = [
            self._point3_to_claim_for(point, claimed_by=self.robot_id)
            for point in own_points
        ]
        proposed_peer_claims = [
            self._point3_to_claim_for(point, claimed_by=peer_id)
            for point in peer_points
        ]

        if not proposed_own_claims and not proposed_peer_claims:
            return None  # no proposal if neither side gets any frontiers

        return proposed_own_claims, proposed_peer_claims

    # Function for negotiation logic 
    def _decide_negotiation(self) -> None:
        """Decide whether to initiate negotiation with a peer"""
        fresh_peers = self._fresh_peer_ids()
        stale_peers = self._stale_peer_ids()
        available_frontiers = self._available_local_frontiers()

        if self.enable_negotiation_requests:
            # Negotiation must solve over the full local frontier set
            # Peer claims are used by the local planner as avoidance constraints, not as inputs that remove candidates from the negotiation problem
            self._tick_requester_state(
                fresh_peers=fresh_peers,
                candidate_frontiers=list(self.local_frontiers),
            )

        if not self._logged_negotiation_stub:
            self.get_logger().info(
                "_decide_negotiation stub reached; "
                f"fresh_peers={fresh_peers}; "
                f"stale_peers={stale_peers}; "
                f"local_frontiers={len(self.local_frontiers)}; "
                f"available_frontiers={len(available_frontiers)}; "
                f"negotiation_requests_enabled={self.enable_negotiation_requests}"
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