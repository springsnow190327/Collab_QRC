"""Assets/spawn-domain launch builders."""

from xml.dom import minidom

from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _strip_comments(node):
    for child in list(node.childNodes):
        if child.nodeType == child.COMMENT_NODE:
            node.removeChild(child)
        else:
            _strip_comments(child)


def _child_elements(parent, name=None):
    for child in parent.childNodes:
        if child.nodeType != child.ELEMENT_NODE:
            continue
        if name is None or child.tagName == name:
            yield child


def _get_or_create_child(parent, name):
    for child in _child_elements(parent, name):
        return child
    child = parent.ownerDocument.createElement(name)
    parent.appendChild(child)
    return child


def _set_text(parent, value):
    for child in list(parent.childNodes):
        parent.removeChild(child)
    parent.appendChild(parent.ownerDocument.createTextNode(value))


def _get_text(node):
    return "".join(child.data for child in node.childNodes if child.nodeType == child.TEXT_NODE).strip()


def _ensure_ros_argument(ros, key: str, value: str):
    target = f"{key}:={value}"
    for argument in _child_elements(ros, "argument"):
        current = _get_text(argument)
        if current.startswith(f"{key}:="):
            _set_text(argument, target)
            return
    argument = ros.ownerDocument.createElement("argument")
    _set_text(argument, target)
    ros.appendChild(argument)


def _rewrite_plugin_remap(remap_text, ns):
    text = remap_text.strip()
    if text in ("odom:=odom/ground_truth", "odom:=/odom/ground_truth"):
        return f"odom:=/{ns}/odom/ground_truth"
    if text in ("~/out:=scan", "~/out:=/scan"):
        return f"~/out:=/{ns}/scan"
    if text in ("~/out:=registered_scan", "~/out:=/registered_scan"):
        return f"~/out:=/{ns}/registered_scan"
    if text in ("~/out:=data", "~/out:=/data", "~/out:=imu/data", "~/out:=/imu/data"):
        return f"~/out:=/{ns}/imu/data"
    return text


def build_namespaced_robot_description(
    robot_description,
    ns,
    ros_control_param_file,
    ros2_control_plugin_filename: str | None = None,
    ray_sensor_plugin_filename: str | None = None,
    imu_sensor_plugin_filename: str | None = None,
    p3d_plugin_filename: str | None = None,
):
    doc = minidom.parseString(robot_description)
    _strip_comments(doc)

    for plugin in doc.getElementsByTagName("plugin"):
        if not plugin.hasAttribute("filename"):
            continue

        filename = plugin.getAttribute("filename")
        if "libgazebo_ros_p3d.so" in filename and p3d_plugin_filename:
            plugin.setAttribute("filename", p3d_plugin_filename)
            filename = p3d_plugin_filename
        elif "libgazebo_ros_imu_sensor.so" in filename and imu_sensor_plugin_filename:
            plugin.setAttribute("filename", imu_sensor_plugin_filename)
            filename = imu_sensor_plugin_filename
        elif "libgazebo_ros_ray_sensor.so" in filename and ray_sensor_plugin_filename:
            plugin.setAttribute("filename", ray_sensor_plugin_filename)
            filename = ray_sensor_plugin_filename
        if plugin.hasAttribute("name"):
            original_name = plugin.getAttribute("name")
            if original_name and not original_name.endswith(f"_{ns}"):
                plugin.setAttribute("name", f"{original_name}_{ns}")

        plugin_name = plugin.getAttribute("name")
        ros = _get_or_create_child(plugin, "ros")
        _set_text(_get_or_create_child(ros, "namespace"), f"/{ns}")
        if plugin_name:
            _ensure_ros_argument(ros, "__name", plugin_name)
        _ensure_ros_argument(ros, "__ns", f"/{ns}")

        if "libgazebo_ros2_control.so" in filename:
            plugin.setAttribute("name", f"gazebo_ros2_control_{ns}")
            if ros2_control_plugin_filename:
                plugin.setAttribute("filename", ros2_control_plugin_filename)
            _set_text(_get_or_create_child(ros, "namespace"), f"/{ns}")
            _ensure_ros_argument(ros, "__name", f"gazebo_ros2_control_{ns}")
            remap = _get_or_create_child(ros, "remapping")
            _set_text(remap, f"~/out:=/{ns}/gazebo_ros2_control/out")
            _set_text(_get_or_create_child(plugin, "robot_param"), "robot_description")
            _set_text(_get_or_create_child(plugin, "robot_param_node"), f"/{ns}/robot_state_publisher")
            _set_text(_get_or_create_child(plugin, "robotNamespace"), f"/{ns}")
            _set_text(_get_or_create_child(plugin, "parameters"), ros_control_param_file)
            continue

        remaps = list(_child_elements(ros, "remapping"))
        for remap in remaps:
            current_text = _get_text(remap)
            new_text = _rewrite_plugin_remap(current_text, ns)
            if new_text != current_text:
                _set_text(remap, new_text)

        existing_texts = {_get_text(remap) for remap in remaps}

        if "libgazebo_ros_p3d.so" in filename and not any(text.startswith("odom:=") for text in existing_texts):
            remap = doc.createElement("remapping")
            _set_text(remap, f"odom:=/{ns}/odom/ground_truth")
            ros.appendChild(remap)

        if "libgazebo_ros_imu_sensor.so" in filename and not any(text.startswith("~/out:=") for text in existing_texts):
            remap = doc.createElement("remapping")
            _set_text(remap, f"~/out:=/{ns}/imu/data")
            ros.appendChild(remap)

        if "libgazebo_ros_ray_sensor.so" in filename:
            target = f"~/out:=/{ns}/scan"
            if "3d_lidar" in plugin.getAttribute("name"):
                target = f"~/out:=/{ns}/registered_scan"
            if not any(text.startswith("~/out:=") for text in existing_texts):
                remap = doc.createElement("remapping")
                _set_text(remap, target)
                ros.appendChild(remap)

    return doc.documentElement.toxml()


def build_dual_robot_stack(
    *,
    ns,
    spawn_x,
    spawn_y,
    spawn_yaw,
    use_sim_time,
    robot_description,
    joints_config,
    links_config,
    gait_config,
    ekf_base_to_footprint,
    ekf_footprint_to_odom,
    joint_state_spawner_delay_sec=5.0,
    effort_spawner_delay_sec=5.2,
    standup_delay_sec=9.0,
    pose_guard_hold_sec=8.5,
    activate_controllers_on_spawn=True,
    stand_up_joint_preset="go2",
    cmd_vel_input_topic="cmd_vel",
    wheel_controller_name=None,
    wheel_spawner_delay_sec=None,
    rsp_publish_frequency=200.0,
    return_handles=False,
):
    tf_remaps = [("/tf", f"/{ns}/tf"), ("/tf_static", f"/{ns}/tf_static")]

    def _safe_float(value, default: float) -> float:
        try:
            return float(value)
        except (TypeError, ValueError):
            return float(default)

    spawn_x_f = _safe_float(spawn_x, 0.0)
    spawn_y_f = _safe_float(spawn_y, 0.0)
    spawn_yaw_f = _safe_float(spawn_yaw, 0.0)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=ns,
        parameters=[
            {"robot_description": ParameterValue(robot_description, value_type=str)},
            {"use_tf_static": False},
            {"publish_frequency": rsp_publish_frequency},
            {"ignore_timestamp": True},
            {"use_sim_time": use_sim_time},
        ],
        remappings=tf_remaps,
        output="screen",
    )

    joint_state_controller_name = f"{ns}_joint_states_controller"
    effort_controller_name = f"{ns}_joint_group_effort_controller"
    effort_topic = f"/{ns}/{effort_controller_name}/joint_trajectory"
    wheel_controller_name = (wheel_controller_name or "").strip()
    if wheel_spawner_delay_sec is None:
        wheel_spawner_delay_sec = effort_spawner_delay_sec + 0.2

    quadruped_controller_node = Node(
        package="champ_base",
        executable="quadruped_controller_node",
        namespace=ns,
        parameters=[
            {"use_sim_time": use_sim_time},
            {"gazebo": True},
            {"publish_joint_states": True},
            {"publish_joint_control": True},
            {"publish_foot_contacts": False},
            {"joint_controller_topic": effort_topic},
            {"urdf": ParameterValue(robot_description, value_type=str)},
            joints_config,
            links_config,
            gait_config,
        ],
        remappings=tf_remaps
        + [
            ("cmd_vel/smooth", cmd_vel_input_topic),
            ("/cmd_vel/smooth", cmd_vel_input_topic),
            ("joy", "joy"),
            ("/joy", "joy"),
        ],
        output="screen",
    )

    state_estimator_node = Node(
        package="champ_base",
        executable="state_estimation_node",
        namespace=ns,
        parameters=[
            {"use_sim_time": use_sim_time},
            {"orientation_from_imu": False},
            {"urdf": ParameterValue(robot_description, value_type=str)},
            joints_config,
            links_config,
            gait_config,
        ],
        remappings=tf_remaps,
        output="screen",
    )

    base_to_footprint_ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        namespace=ns,
        name="base_to_footprint_ekf",
        parameters=[
            {"base_link_frame": "base_link"},
            {"use_sim_time": use_sim_time},
            ekf_base_to_footprint,
        ],
        remappings=tf_remaps + [("odometry/filtered", "odom/local")],
        output="screen",
    )

    footprint_to_odom_ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        namespace=ns,
        name="footprint_to_odom_ekf",
        parameters=[
            {"base_link_frame": "base_link"},
            {"use_sim_time": use_sim_time},
            ekf_footprint_to_odom,
        ],
        remappings=tf_remaps + [("odometry/filtered", "odom")],
        output="screen",
    )

    spawn_entity_node = Node(
        package="go2w_spawn",
        executable="spawn_entity_direct.py",
        output="screen",
        arguments=[
            "--entity",
            ns,
            "--topic",
            f"/{ns}/robot_description",
            "--x",
            f"{spawn_x_f:.6f}",
            "--y",
            f"{spawn_y_f:.6f}",
            "--z",
            "0.45",
            "--roll",
            "0",
            "--pitch",
            "0",
            "--yaw",
            f"{spawn_yaw_f:.6f}",
        ],
    )

    initial_pose_guard_node = Node(
        package="go2w_spawn",
        executable="initial_pose_guard.py",
        name=f"{ns}_initial_pose_guard",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"entity_name": ns},
            {"spawn_x": spawn_x_f},
            {"spawn_y": spawn_y_f},
            {"spawn_z": 0.45},
            {"spawn_yaw": spawn_yaw_f},
            {"hold_sec": pose_guard_hold_sec},
            {"rate": 15.0},
            {"request_timeout_sec": 0.8},
            {"max_failures": 200},
            {"retry_backoff_initial_sec": 0.1},
            {"retry_backoff_max_sec": 1.5},
        ],
        output="screen",
    )

    joint_state_spawner_args = [
        joint_state_controller_name,
        "--controller-manager",
        f"/{ns}/controller_manager",
        "--controller-manager-timeout",
        "60",
    ]
    effort_spawner_args = [
        effort_controller_name,
        "--controller-manager",
        f"/{ns}/controller_manager",
        "--controller-manager-timeout",
        "60",
    ]
    if not activate_controllers_on_spawn:
        joint_state_spawner_args.append("--inactive")
        effort_spawner_args.append("--inactive")

    load_joint_state_controller = Node(
        package="controller_manager",
        executable="spawner",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=joint_state_spawner_args,
        output="screen",
    )

    load_joint_effort_controller = Node(
        package="controller_manager",
        executable="spawner",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=effort_spawner_args,
        output="screen",
    )

    load_wheel_velocity_controller = None
    if wheel_controller_name:
        wheel_spawner_args = [
            wheel_controller_name,
            "--controller-manager",
            f"/{ns}/controller_manager",
            "--controller-manager-timeout",
            "60",
        ]
        if not activate_controllers_on_spawn:
            wheel_spawner_args.append("--inactive")
        load_wheel_velocity_controller = Node(
            package="controller_manager",
            executable="spawner",
            parameters=[{"use_sim_time": use_sim_time}],
            arguments=wheel_spawner_args,
            output="screen",
        )

    contact_sensor = Node(
        package="champ_gazebo",
        executable="contact_sensor",
        namespace=ns,
        parameters=[{"use_sim_time": use_sim_time}, links_config],
        output="screen",
    )

    stand_up_node = Node(
        package="go2w_spawn",
        executable="stand_up_slowly.py",
        namespace=ns,
        parameters=[
            {"use_sim_time": use_sim_time},
            {"controller_wait_sec": 4.0},
            {"phase1_sec": 6.0},
            {"phase2_sec": 12.0},
            {"phase3_sec": 18.0},
            {"knee_bend_ratio": 0.80},
            {"joint_controller_topic": effort_topic},
            {"joint_name_preset": stand_up_joint_preset},
        ],
        output="screen",
    )

    wait_joint_states_ready = ExecuteProcess(
        cmd=[
            "bash",
            "-lc",
            (
                f"until ros2 topic echo /{ns}/joint_states --once >/dev/null 2>&1; do "
                "sleep 0.25; "
                "done"
            ),
        ],
        output="screen",
    )

    stack_actions = [
        robot_state_publisher_node,
        quadruped_controller_node,
        state_estimator_node,
        base_to_footprint_ekf,
        footprint_to_odom_ekf,
        spawn_entity_node,
        TimerAction(period=0.6, actions=[initial_pose_guard_node]),
        TimerAction(period=joint_state_spawner_delay_sec, actions=[load_joint_state_controller]),
        TimerAction(period=effort_spawner_delay_sec, actions=[load_joint_effort_controller]),
        contact_sensor,
        TimerAction(period=standup_delay_sec, actions=[wait_joint_states_ready]),
        RegisterEventHandler(
            OnProcessExit(
                target_action=wait_joint_states_ready,
                on_exit=[stand_up_node],
            )
        ),
    ]
    if load_wheel_velocity_controller is not None:
        stack_actions.insert(
            stack_actions.index(contact_sensor),
            TimerAction(period=wheel_spawner_delay_sec, actions=[load_wheel_velocity_controller]),
        )
    if return_handles:
        return (
            stack_actions,
            {
                "initial_pose_guard_node": initial_pose_guard_node,
            },
        )
    return stack_actions
