"""MuJoCo contact sensor bridge node.

Loads a copy of the MuJoCo model, subscribes to /joint_states to sync the
kinematic state, then reads touch sensor values from the MuJoCo data to
determine foot contact. Publishes champ_msgs/ContactsStamped at 50 Hz on
/{ns}/foot_contacts.

Touch sensors in the MJCF: FL_foot_contact, FR_foot_contact,
RL_foot_contact, RR_foot_contact.  A non-zero touch sensor value indicates
contact.
"""

import threading

import mujoco
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from champ_msgs.msg import ContactsStamped
from ament_index_python.packages import get_package_share_directory


# Names of the 4 touch sensors in the MJCF, order must match CHAMP convention:
# [FL, FR, RL, RR]
_DEFAULT_TOUCH_SENSORS = [
    'FL_foot_contact',
    'FR_foot_contact',
    'RL_foot_contact',
    'RR_foot_contact',
]


class MujocoContactNode(Node):

    def __init__(self):
        super().__init__('mujoco_contact_node',
                         parameter_overrides=[rclpy.Parameter('use_sim_time', value=True)])

        # --- Parameters ---
        self.declare_parameter('mjcf_path', '')
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('contact_threshold', 0.01)
        self.declare_parameter('touch_sensor_names', _DEFAULT_TOUCH_SENSORS)

        mjcf_path = self.get_parameter('mjcf_path').get_parameter_value().string_value
        if not mjcf_path:
            try:
                pkg_dir = get_package_share_directory('go2_gazebo_sim')
                mjcf_path = pkg_dir + '/mujoco/go2w.xml'
            except Exception:
                self.get_logger().fatal('mjcf_path not set and go2_gazebo_sim not found')
                raise RuntimeError('mjcf_path is required')

        rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.contact_threshold = self.get_parameter(
            'contact_threshold').get_parameter_value().double_value
        sensor_names = self.get_parameter(
            'touch_sensor_names').get_parameter_value().string_array_value

        # --- Load MuJoCo model ---
        self.get_logger().info(f'Loading MuJoCo model from: {mjcf_path}')
        self.mj_model = mujoco.MjModel.from_xml_path(mjcf_path)
        self.mj_data = mujoco.MjData(self.mj_model)
        self._mj_lock = threading.Lock()

        # Resolve touch sensor addresses
        self._touch_addrs = []
        for name in sensor_names:
            sid = mujoco.mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_SENSOR, name)
            if sid < 0:
                self.get_logger().error(f'Touch sensor "{name}" not found in MuJoCo model')
                self._touch_addrs.append(-1)
            else:
                adr = self.mj_model.sensor_adr[sid]
                self._touch_addrs.append(adr)
                self.get_logger().info(f'Touch sensor "{name}": sensor_id={sid}, adr={adr}')

        # Build joint name -> qpos index mapping
        self._joint_map = {}
        for i in range(self.mj_model.njnt):
            name = mujoco.mj_id2name(self.mj_model, mujoco.mjtObj.mjOBJ_JOINT, i)
            if name and self.mj_model.jnt_type[i] != mujoco.mjtJoint.mjJNT_FREE:
                qpos_adr = self.mj_model.jnt_qposadr[i]
                self._joint_map[name] = qpos_adr

        # --- ROS pub/sub ---
        self.contacts_pub = self.create_publisher(ContactsStamped, 'foot_contacts', 10)

        self.js_sub = self.create_subscription(
            JointState, 'joint_states', self._joint_state_cb, 10)

        self._latest_js = None
        self._js_lock = threading.Lock()

        self.timer = self.create_timer(1.0 / rate, self._timer_cb)
        self.get_logger().info(f'MuJoCo contact node started at {rate} Hz')

    # ------------------------------------------------------------------
    def _joint_state_cb(self, msg: JointState):
        with self._js_lock:
            self._latest_js = msg

    # ------------------------------------------------------------------
    def _timer_cb(self):
        with self._js_lock:
            js = self._latest_js
        if js is None:
            return

        # Sync joint state into MuJoCo model
        with self._mj_lock:
            for i, name in enumerate(js.name):
                if name in self._joint_map:
                    self.mj_data.qpos[self._joint_map[name]] = js.position[i]

            # Forward to compute sensor values (contact detection needs full step,
            # but mj_forward gives us the sensor readouts for touch sensors)
            mujoco.mj_forward(self.mj_model, self.mj_data)

            # Read touch sensor values
            contacts = []
            for adr in self._touch_addrs:
                if adr < 0:
                    contacts.append(False)
                else:
                    # Touch sensor returns a scalar: normal force magnitude
                    force = self.mj_data.sensordata[adr]
                    contacts.append(float(force) > self.contact_threshold)

        # Publish
        msg = ContactsStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.contacts = contacts
        self.contacts_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MujocoContactNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
