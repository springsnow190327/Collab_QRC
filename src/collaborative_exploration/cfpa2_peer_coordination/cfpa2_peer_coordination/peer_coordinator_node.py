"""Decentralised peer coordinator node.

Each robot runs one instance of this node. It broadcasts this robot's PeerState heartbeat and (later) negotiates frontier ownership with peers.

This is the skeleton; protocol logic is not yet implemented.

"""

from __future__ import annotations

from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy, QoSProfile

from std_msgs.msg import Header
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry

from cfpa2_peer_coordination_msgs.msg import (
    ClaimedFrontier,         # noqa: F401 
    NegotiationRequest,      # noqa: F401
    NegotiationResponse,     # noqa: F401
    PeerState,
)

PROTOCOL_VERSION = 1   # bump when message formats change incompatibly

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

        # Per-peer storage. Initialised with a placeholder for every known peer. Updated by _peer_state_received(), read by negotiation logic (later).
        self.peer_info: dict[str, PeerInfo] = {
            peer_id: PeerInfo() for peer_id in self.peer_ids
        }

        # Local pose storage. Updated by _odom_received(), published in PeerState
        self._latest_pose: Pose | None = None

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

        msg.claimed_frontiers = []

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

        self.get_logger().debug(
            f"Received PeerState from {peer_id} at ns={info.last_received_ns}"
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

    # Helper functions
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

    # Function for negotiation logic 
    def _decide_negotiation(self) -> None:
        """Decide whether to initiate negotiation with a peer. Stub"""
        fresh_peers = self._fresh_peer_ids()
        stale_peers = self._stale_peer_ids()

        if not self._logged_negotiation_stub:
            self.get_logger().info(
                "_decide_negotiation stub reached; "
                f"fresh_peers={fresh_peers}; "
                f"stale_peers={stale_peers}; "
                "negotiation logic not yet implemented"
            )
            self._logged_negotiation_stub = True
        
        self.get_logger().debug(f"Fresh peers: {fresh_peers}; Stale peers: {stale_peers}")

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

