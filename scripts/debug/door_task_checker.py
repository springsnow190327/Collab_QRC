#!/usr/bin/env python3
"""MuJoCo physics-based door task success checker.

Loads a copy of the MJCF model, syncs the full simulation state via ROS
topics (odom for free-body poses, joint_states for actuated joints, door
angle for hinge), then calls mj_forward() and reads mjData.contact to
detect inter-robot body collisions at the physics level.

Success criteria:
  1. Both robots in the same room (both x < 3.5 or both x > 4.5)
  2. Door hinge angle exceeded 70 degrees (1.2217 rad) at some point
  3. Zero inter-robot body collisions (including wheels) throughout the run

Reads from:
  /door_task/door_state     (Float64)    — door hinge angle
  /robot_a/odom/nav         (Odometry)   — Robot A pose
  /robot_b/odom/nav         (Odometry)   — Robot B pose
  /robot_a/joint_states     (JointState) — Robot A actuated joints
  /robot_b/joint_states     (JointState) — Robot B actuated joints

Publishes nothing. Pure observer.
"""
from __future__ import annotations

import math
import sys
import threading
import time
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
MJCF_PATH = Path(__file__).resolve().parent.parent / \
    "src/go2w/go2_gazebo_sim/mujoco/two_rooms_door_scene.xml"

ROBOT_A_ROOT = "base_link"
ROBOT_B_ROOT = "b_base_link"
SAME_ROOM_A = 3.5   # both x < this => Room A
SAME_ROOM_B = 4.5   # both x > this => Room B
DOOR_ANGLE_DEG = 70.0
DOOR_ANGLE_RAD = math.radians(DOOR_ANGLE_DEG)
MAX_WATCH_SEC = 180.0
CHECK_RATE_HZ = 20.0  # physics sync + contact check rate


class DoorTaskChecker(Node):
    def __init__(self):
        super().__init__("door_task_checker")

        # ── Load MuJoCo model ─────────────────────────────────────────
        try:
            import mujoco
            self._mj = mujoco
        except ImportError:
            self.get_logger().fatal("mujoco package not installed")
            raise SystemExit(1)

        self.get_logger().info(f"Loading MJCF: {MJCF_PATH}")
        self._model = mujoco.MjModel.from_xml_path(str(MJCF_PATH))
        self._data = mujoco.MjData(self._model)
        self._mj_lock = threading.Lock()

        # ── Build geom -> robot ownership map ─────────────────────────
        self._geom_owner, self._geom_names = self._build_ownership()

        # ── Build joint name -> qpos address map ──────────────────────
        self._joint_map: dict[str, int] = {}
        for i in range(self._model.njnt):
            name = mujoco.mj_id2name(self._model, mujoco.mjtObj.mjOBJ_JOINT, i)
            if name and self._model.jnt_type[i] != mujoco.mjtJoint.mjJNT_FREE:
                self._joint_map[name] = self._model.jnt_qposadr[i]

        # Free joint addresses (7 values each: x,y,z, qw,qx,qy,qz)
        root_jid = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_JOINT, "root")
        b_root_jid = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_JOINT, "b_root")
        self._root_qposadr = self._model.jnt_qposadr[root_jid] if root_jid >= 0 else -1
        self._b_root_qposadr = self._model.jnt_qposadr[b_root_jid] if b_root_jid >= 0 else -1

        # Door hinge address
        door_jid = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_JOINT, "door_hinge")
        self._door_qposadr = self._model.jnt_qposadr[door_jid] if door_jid >= 0 else -1

        self.get_logger().info(
            f"Model: {self._model.ngeom} geoms, "
            f"{sum(1 for v in self._geom_owner.values() if v == 'robot_a')} robot_a, "
            f"{sum(1 for v in self._geom_owner.values() if v == 'robot_b')} robot_b, "
            f"root_qpos={self._root_qposadr}, b_root_qpos={self._b_root_qposadr}, "
            f"door_qpos={self._door_qposadr}"
        )

        # ── State ─────────────────────────────────────────────────────
        self._door_angle = 0.0
        self._peak_door_angle = 0.0
        self._robot_a_pos: tuple[float, float] | None = None
        self._robot_b_pos: tuple[float, float] | None = None
        self._robot_a_odom: Odometry | None = None
        self._robot_b_odom: Odometry | None = None
        self._robot_a_js: JointState | None = None
        self._robot_b_js: JointState | None = None
        self._start_time = time.monotonic()

        # Collision tracking
        self._inter_robot_collisions: list[dict] = []
        self._total_inter_robot_contacts = 0

        # Criterion flags
        self._door_passed_threshold = False
        self._same_room_achieved = False
        self._same_room_time: float | None = None
        self._button_ever_pressed = False

        # ── Subscriptions ─────────────────────────────────────────────
        self.create_subscription(Float64, "/door_task/door_state", self._on_door, 10)
        self.create_subscription(
            Bool, "/door_task/button_pressed", self._on_button, 10,
        )
        self.create_subscription(Odometry, "/robot_a/odom/nav", self._on_a_odom, 10)
        self.create_subscription(Odometry, "/robot_b/odom/nav", self._on_b_odom, 10)
        self.create_subscription(JointState, "/robot_a/joint_states", self._on_a_js, 10)
        self.create_subscription(JointState, "/robot_b/joint_states", self._on_b_js, 10)

        # Physics sync + contact check at CHECK_RATE_HZ
        self._physics_timer = self.create_timer(
            1.0 / CHECK_RATE_HZ, self._physics_tick)
        # Status display at 0.2 Hz (every 5s)
        self._status_timer = self.create_timer(5.0, self._status_tick)

        self.get_logger().info("=" * 62)
        self.get_logger().info("  DOOR TASK CHECKER — MuJoCo physics contact detection")
        self.get_logger().info(f"  1. Both robots in same room (x<{SAME_ROOM_A} or x>{SAME_ROOM_B})")
        self.get_logger().info(f"  2. Door peak angle > {DOOR_ANGLE_DEG} deg ({DOOR_ANGLE_RAD:.4f} rad)")
        self.get_logger().info(f"  3. Zero inter-robot body collisions (including wheels)")
        self.get_logger().info(f"  Physics sync rate: {CHECK_RATE_HZ} Hz")
        self.get_logger().info("=" * 62)

    # ── Ownership map ─────────────────────────────────────────────────

    def _build_ownership(self) -> tuple[dict[int, str], dict[int, str]]:
        """Build geom_id -> owner ('robot_a'|'robot_b'|'world') map."""
        mujoco = self._mj
        model = self._model

        a_root_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, ROBOT_A_ROOT)
        b_root_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, ROBOT_B_ROOT)

        body_owner: dict[int, str] = {}

        def tag_subtree(root_id: int, owner: str):
            stack = [root_id]
            while stack:
                bid = stack.pop()
                body_owner[bid] = owner
                for child_id in range(model.nbody):
                    if model.body_parentid[child_id] == bid and child_id != bid:
                        stack.append(child_id)

        if a_root_id >= 0:
            tag_subtree(a_root_id, "robot_a")
        if b_root_id >= 0:
            tag_subtree(b_root_id, "robot_b")

        geom_owner: dict[int, str] = {}
        geom_names: dict[int, str] = {}
        for gid in range(model.ngeom):
            bid = model.geom_bodyid[gid]
            geom_owner[gid] = body_owner.get(bid, "world")
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, gid)
            geom_names[gid] = name if name else f"geom_{gid}"

        return geom_owner, geom_names

    # ── Callbacks ─────────────────────────────────────────────────────

    def _on_door(self, msg: Float64):
        self._door_angle = msg.data
        if msg.data > self._peak_door_angle:
            self._peak_door_angle = msg.data
        if msg.data >= DOOR_ANGLE_RAD and not self._door_passed_threshold:
            self._door_passed_threshold = True
            self.get_logger().info(
                f"  [CRIT 2] PASS: door reached {math.degrees(msg.data):.1f} deg "
                f"(threshold: {DOOR_ANGLE_DEG} deg)"
            )

    def _on_button(self, msg: Bool):
        if bool(msg.data) and not self._button_ever_pressed:
            self._button_ever_pressed = True
            self.get_logger().info("  [CRIT 4] PASS: button has been pressed")

    def _on_a_odom(self, msg: Odometry):
        self._robot_a_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self._robot_a_odom = msg

    def _on_b_odom(self, msg: Odometry):
        self._robot_b_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self._robot_b_odom = msg

    def _on_a_js(self, msg: JointState):
        self._robot_a_js = msg

    def _on_b_js(self, msg: JointState):
        self._robot_b_js = msg

    # ── Physics sync + contact detection ──────────────────────────────

    def _sync_free_joint(self, odom: Odometry, qposadr: int):
        """Set free joint qpos from Odometry message."""
        if qposadr < 0:
            return
        p = odom.pose.pose.position
        q = odom.pose.pose.orientation
        # MuJoCo free joint: [x, y, z, qw, qx, qy, qz]
        self._data.qpos[qposadr:qposadr + 7] = [
            p.x, p.y, p.z,
            q.w, q.x, q.y, q.z,
        ]

    def _sync_actuated_joints(self, js: JointState):
        """Set actuated joint qpos from JointState message."""
        for i, name in enumerate(js.name):
            if name in self._joint_map:
                self._data.qpos[self._joint_map[name]] = js.position[i]

    def _physics_tick(self):
        """Sync state into MuJoCo model and check contacts."""
        # Need at least odom for both robots to do useful contact check
        if self._robot_a_odom is None or self._robot_b_odom is None:
            return

        with self._mj_lock:
            # Sync free-body poses from odom
            self._sync_free_joint(self._robot_a_odom, self._root_qposadr)
            self._sync_free_joint(self._robot_b_odom, self._b_root_qposadr)

            # Sync door hinge angle
            if self._door_qposadr >= 0:
                self._data.qpos[self._door_qposadr] = self._door_angle

            # Sync actuated joints
            if self._robot_a_js:
                self._sync_actuated_joints(self._robot_a_js)
            if self._robot_b_js:
                self._sync_actuated_joints(self._robot_b_js)

            # Forward kinematics + collision detection
            self._mj.mj_forward(self._model, self._data)

            # Read contacts from mjData
            for i in range(self._data.ncon):
                contact = self._data.contact[i]
                g1, g2 = contact.geom1, contact.geom2
                self._check_contact_pair(g1, g2)

    def _check_contact_pair(self, g1: int, g2: int):
        """Check if a contact pair is inter-robot."""
        o1 = self._geom_owner.get(g1, "unknown")
        o2 = self._geom_owner.get(g2, "unknown")

        is_inter_robot = (
            (o1 == "robot_a" and o2 == "robot_b") or
            (o1 == "robot_b" and o2 == "robot_a")
        )
        if is_inter_robot:
            self._total_inter_robot_contacts += 1
            elapsed = time.monotonic() - self._start_time
            n1 = self._geom_names.get(g1, f"g{g1}")
            n2 = self._geom_names.get(g2, f"g{g2}")
            record = {
                "t": round(elapsed, 1),
                "geom1": n1, "owner1": o1,
                "geom2": n2, "owner2": o2,
            }
            # Log first occurrence of each unique pair
            pair_key = (min(n1, n2), max(n1, n2))
            already_logged = any(
                (min(r["geom1"], r["geom2"]), max(r["geom1"], r["geom2"])) == pair_key
                for r in self._inter_robot_collisions
            )
            if not already_logged:
                self._inter_robot_collisions.append(record)
                self.get_logger().warn(
                    f"  [CRIT 3] VIOLATION: inter-robot contact! "
                    f"{n1}({o1}) <-> {n2}({o2}) at t={elapsed:.1f}s"
                )

    # ── Same-room logic ───────────────────────────────────────────────

    def _room_label(self, x: float) -> str:
        if x < SAME_ROOM_A:
            return "Room_A"
        elif x > SAME_ROOM_B:
            return "Room_B"
        return "DOOR"

    def _both_same_room(self) -> bool:
        if self._robot_a_pos is None or self._robot_b_pos is None:
            return False
        ax, bx = self._robot_a_pos[0], self._robot_b_pos[0]
        return (ax < SAME_ROOM_A and bx < SAME_ROOM_A) or \
               (ax > SAME_ROOM_B and bx > SAME_ROOM_B)

    # ── Status + terminal checks ──────────────────────────────────────

    def _status_tick(self):
        elapsed = time.monotonic() - self._start_time
        same = self._both_same_room()

        # Track same-room persistence
        if same:
            if self._same_room_time is None:
                self._same_room_time = elapsed
                self.get_logger().info(
                    f"  [CRIT 1] Both robots in same room at t={elapsed:.1f}s"
                )
        else:
            self._same_room_time = None

        a_str = f"({self._robot_a_pos[0]:5.2f},{self._robot_a_pos[1]:5.2f})" \
            if self._robot_a_pos else "WAIT"
        b_str = f"({self._robot_b_pos[0]:5.2f},{self._robot_b_pos[1]:5.2f})" \
            if self._robot_b_pos else "WAIT"
        self.get_logger().info(
            f"[{elapsed:6.1f}s] door={self._door_angle:.3f}rad "
            f"peak={self._peak_door_angle:.3f} "
            f"A={a_str} B={b_str} "
            f"same={same} collisions={self._total_inter_robot_contacts}"
        )

        # Check terminal: same room for 3+ seconds
        if same and self._same_room_time and (elapsed - self._same_room_time) >= 3.0:
            self._same_room_achieved = True
            self._final_verdict()

        # Timeout
        if elapsed > MAX_WATCH_SEC:
            self._final_verdict()

    # ── Verdict ───────────────────────────────────────────────────────

    def _final_verdict(self):
        c1 = self._same_room_achieved
        c2 = self._door_passed_threshold
        c3 = self._total_inter_robot_contacts == 0
        c4 = self._button_ever_pressed
        all_pass = c1 and c2 and c3 and c4

        self.get_logger().info("")
        self.get_logger().info("=" * 62)
        self.get_logger().info("  DOOR TASK CHECKER — FINAL VERDICT")
        self.get_logger().info("=" * 62)
        self.get_logger().info(
            f"  [CRIT 1] Same room:         {'PASS' if c1 else 'FAIL'}"
        )
        if self._robot_a_pos:
            self.get_logger().info(
                f"           Robot A: ({self._robot_a_pos[0]:.2f}, {self._robot_a_pos[1]:.2f}) "
                f"[{self._room_label(self._robot_a_pos[0])}]"
            )
        if self._robot_b_pos:
            self.get_logger().info(
                f"           Robot B: ({self._robot_b_pos[0]:.2f}, {self._robot_b_pos[1]:.2f}) "
                f"[{self._room_label(self._robot_b_pos[0])}]"
            )
        self.get_logger().info(
            f"  [CRIT 2] Door > {DOOR_ANGLE_DEG} deg:     {'PASS' if c2 else 'FAIL'}  "
            f"(peak: {math.degrees(self._peak_door_angle):.1f} deg)"
        )
        self.get_logger().info(
            f"  [CRIT 3] No inter-robot col: {'PASS' if c3 else 'FAIL'}  "
            f"({self._total_inter_robot_contacts} contacts detected)"
        )
        self.get_logger().info(
            f"  [CRIT 4] Button pressed:    {'PASS' if c4 else 'FAIL'}"
        )
        if self._inter_robot_collisions:
            self.get_logger().info("           Collision log (unique pairs):")
            for r in self._inter_robot_collisions[:20]:
                self.get_logger().info(
                    f"             t={r['t']:6.1f}s  "
                    f"{r['geom1']}({r['owner1']}) <-> {r['geom2']}({r['owner2']})"
                )
        self.get_logger().info("=" * 62)
        self.get_logger().info(
            f"  RESULT: {'PASS' if all_pass else 'FAIL'}"
        )
        self.get_logger().info("=" * 62)

        raise SystemExit(0 if all_pass else 1)


def main():
    rclpy.init()
    node = DoorTaskChecker()
    try:
        rclpy.spin(node)
    except SystemExit as e:
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(e.code)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
