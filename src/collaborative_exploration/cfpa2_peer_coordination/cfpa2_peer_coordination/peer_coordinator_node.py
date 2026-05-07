"""Decentralised peer coordinator node.

Each robot runs one instance of this node. It broadcasts this robot's PeerState heartbeat and (later) negotiates frontier ownership with peers.

This is the skeleton; protocol logic is not yet implemented.

"""

from __future__ import annotations

import rclpy
from rclpy.node import Node

from cfpa2_peer_coordination_msgs.msg import(
    ClaimedFrontier,
    NegotiationRequest,
    NegotiationResponse,
    PeerState,
)

PROTOCOL_VERSION = 1   # bump when message formats change incompatibly

# Topic naming convention
PEER_STATE_TOPIC = "cfpa2_peer_coordination/peer_state"
NEGOTIATION_REQUEST_TOPIC = "cfpa2_peer_coordination/inbox/negotiation_request"
NEGOTIATION_RESPONSE_TOPIC = "cfpa2_peer_coordination/inbox/negotiation_response"


class PeerCoordinatorNode(Node):
    """Per-robot peer coordinator. One instance per robot."""

    def __init__(self) -> None:
        super().__init__("cfpa2_peer_coordinator")

        # Parameters
        self.declare_parameter("robot_id", "robot_a")
        self.declare_parameter("robot_namespace", "robot_a")

        # v1 assumption: peer_id == peer_namespace. Used for both protocol identity and topic addressing. Split into separate parameters if this assumption ever needs to break
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

        self.get_logger().info(
            f"PeerCoordinatorNode starting | robot_id={self.robot_id} "
            f"namespace={self.robot_namespace} "
            f"peer_namespaces={self.peer_namespaces} "
            f"protocol_version={PROTOCOL_VERSION}"
        )

        # Timers (stubs only for now)
        peer_state_period = 1.0 / max(self.peer_state_rate_hz, 1e-6)
        negotiation_period = 1.0 / max(self.negotiation_rate_hz, 1e-6)

        self.peer_state_timer = self.create_timer(
            peer_state_period, self._publish_peer_state
        )
        self.negotiation_timer = self.create_timer(
            negotiation_period, self._decide_negotiation
        )


    # Timer callbacks (stubs)
    def _publish_peer_state(self) -> None:
        """Broadcast this robot's current state. Stub"""
        # Construct an empty PeerState just to verify the message import works.
        msg = PeerState()
        msg.robot_id = self.robot_id
        msg.protocol_version = PROTOCOL_VERSION
        # No publisher wired yet; just log on first tick to confirm liveness
        self.get_logger().debug(
            f"_publish_peer_state: would publish {msg.robot_id} "
            f"v{msg.protocol_version}"
        )

    def _decide_negotiation(self) -> None:
        """Decide whether to initiate negotiation with a peer. Stub"""
        self.get_logger().debug("_decide_negotiation stub")

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





