#!/usr/bin/env python3
"""Bring up the complete wheeled OpenArmX robot with two LinkerHand O6 hands.

This launch owns every hardware interface once. Do not run it together with
the standard integrated bringup or the standalone O6 bimanual bringup.
"""

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml

from openarmx_hand_description.model import inject_o6_hands_into_urdf


def _value(context, name):
    return context.perform_substitution(LaunchConfiguration(name))


def _validate_o6_configuration(mode, robot_can_interfaces, o6_can_interfaces):
    """Validate the selected O6 transport before starting hardware nodes."""
    selected = str(mode).strip().lower()
    if selected not in ("shared_bus", "independent"):
        raise RuntimeError(
            "o6_hardware_mode must be 'shared_bus' or 'independent'"
        )
    if selected == "independent":
        robot_can = {str(interface).strip() for interface in robot_can_interfaces}
        hand_can = {str(interface).strip() for interface in o6_can_interfaces}
        overlap = robot_can & hand_can
        if len(hand_can) != 2:
            raise RuntimeError("independent mode requires distinct CAN interfaces for each O6")
        if overlap:
            raise RuntimeError(
                "independent mode requires separate CAN interfaces; "
                f"overlap: {', '.join(sorted(overlap))}"
            )
    return selected


def _lift_defaults():
    return {
        "lift_node_id": "16",
        "lift_min_height": "-0.750",
        "lift_max_height": "0.400",
        "lift_max_velocity_mps": "0.10",
        "lift_lower_switch_position_m": "0.000",
        "lift_home_switch_position_m": "0.650",
        "lift_upper_switch_position_m": "0.950",
        "lift_switch_position_tolerance_m": "0.008",
        "lift_counts_per_meter": "2000000.0",
        "lift_counts_per_revolution": "10000.0",
        "lift_profile_acceleration": "50000",
        "lift_profile_deceleration": "50000",
        "lift_homing_method": "27",
        "lift_homing_speed_mps": "0.010",
        "lift_homing_low_speed_ratio": "0.2",
        "lift_homing_acceleration": "50000",
        "lift_homing_timeout_sec": "60.0",
        "lift_homing_configure_di": "true",
        "lift_di_active_low": "true",
        "lift_di6_not_func": "2",
        "lift_di4_homing_func": "22",
        "lift_di5_pot_func": "1",
        "lift_home_di_channel": "4",
        "lift_pot_di_channel": "5",
        "lift_not_di_channel": "6",
        "lift_invert_command": "true",
        "lift_invert_feedback": "true",
        "lift_command_timeout_sec": "0.5",
        "lift_sdo_timeout_sec": "0.20",
        "lift_feedback_poll_rate_hz": "20.0",
        "lift_calibration_file": os.path.join(
            get_package_share_directory("lift_slide_driver"),
            "config",
            "lift_slide_calibration.yaml",
        ),
    }


def _render_o6_description(context):
    description_share = get_package_share_directory("openarmx_integrated_description")
    xacro_path = os.path.join(description_share, "urdf", "openarmx_integrated_robot_o6.urdf.xacro")
    mappings = {
        "use_fake_hardware": _value(context, "use_fake_hardware"),
        "use_mock": _value(context, "use_fake_hardware"),
        "steering_can_interface": _value(context, "chassis_steering_can"),
        "driving_can_interface": _value(context, "chassis_driving_can"),
        "left_arm_can_interface": _value(context, "left_arm_can"),
        "right_arm_can_interface": _value(context, "right_arm_can"),
        "lift_can_interface": _value(context, "lift_can"),
        "head_can_interface": _value(context, "head_can"),
        "head_control_mode": _value(context, "head_control_mode"),
        "enable_head": _value(context, "enable_head"),
        "control_mode": _value(context, "control_mode"),
        "ros2_control": "true",
    }
    for name in _lift_defaults():
        if name.startswith("lift_"):
            mappings[name.removeprefix("lift_")] = _value(context, name)
    mappings["lift_node_id"] = _value(context, "lift_node_id")

    base_urdf = xacro.process_file(xacro_path, mappings=mappings).toxml()
    mode = _validate_o6_configuration(
        _value(context, "o6_hardware_mode"),
        {
            _value(context, "chassis_steering_can"),
            _value(context, "chassis_driving_can"),
            _value(context, "left_arm_can"),
            _value(context, "right_arm_can"),
            _value(context, "lift_can"),
            _value(context, "head_can"),
        },
        {
            _value(context, "left_o6_can_interface"),
            _value(context, "right_o6_can_interface"),
        },
    )
    return inject_o6_hands_into_urdf(
        base_urdf,
        o6_hardware_mode=mode,
        left_o6_can_id=_value(context, "left_o6_can_id"),
        right_o6_can_id=_value(context, "right_o6_can_id"),
        o6_update_rate=_value(context, "o6_update_rate"),
    )


def _independent_o6_nodes(context):
    """Create standalone O6 drivers and state plumbing for independent CAN."""
    mode = _value(context, "o6_hardware_mode").strip().lower()
    if mode != "independent":
        return []

    fake_o6 = _value(context, "use_fake_hardware").strip().lower() in (
        "true", "1", "yes", "on"
    )
    input_suffix = "command" if fake_o6 else "state"

    driver_defaults = {
        "transport": "can",
        "publish_rate": 100.0,
        "info_rate": 1.0,
        "poll_touch": False,
        "poll_matrix_touch": False,
        "poll_diagnostics": True,
        "poll_device_info": True,
    }
    nodes = []
    if not fake_o6:
        for side, can_name, can_id in (
            ("right", "right_o6_can_interface", "right_o6_can_id"),
            ("left", "left_o6_can_interface", "left_o6_can_id"),
        ):
            parameters = dict(driver_defaults)
            parameters.update({
                "side": side,
                "can": _value(context, can_name),
                "can_id": int(_value(context, can_id), 0),
            })
            nodes.append(Node(
                package="hands_hardware",
                executable="o6_driver_node",
                name=f"openarmx_o6_{side}_driver",
                output="screen",
                parameters=[parameters],
            ))

    nodes.extend([
        Node(
            package="hands_description",
            executable="o6_joint_state_mapper_node",
            name="openarmx_o6_joint_state_mapper",
            output="screen",
            parameters=[{
                "hand": "both",
                "left_input_topic": f"/openarmx/o6/left/{input_suffix}",
                "right_input_topic": f"/openarmx/o6/right/{input_suffix}",
                "joint_states_topic": "/openarmx/o6/joint_states",
                "publish_rate": 50.0,
            }],
        ),
        Node(
            package="openarmx_hand_bringup",
            executable="joint_state_merger",
            name="openarmx_hand_joint_state_merger",
            output="screen",
            parameters=[{
                "arm_topic": "/joint_states",
                "hand_topic": "/openarmx/o6/joint_states",
                "output_topic": "/openarmx/display/joint_states",
                "publish_rate": 50.0,
            }],
        ),
    ])
    return nodes


def _spawn(controller, delay, condition=None, inactive=False):
    arguments = [
        controller,
        "-c", "/controller_manager",
        "--controller-manager-timeout", "120.0",
        "--service-call-timeout", "30.0",
        "--switch-timeout", "30.0",
    ]
    if inactive:
        arguments.append("--inactive")
    return TimerAction(
        period=delay,
        actions=[Node(
            package="controller_manager",
            executable="spawner",
            arguments=arguments,
            output="screen",
            condition=condition,
        )],
    )


def _launch(context):
    description = _render_o6_description(context)
    mode = _value(context, "o6_hardware_mode").strip().lower()
    robot_description = {"robot_description": description}
    chassis_config = os.path.join(
        get_package_share_directory("swerve_description"), "config", "chassis_version_6.0.yaml"
    )
    import yaml
    with open(chassis_config, encoding="utf-8") as stream:
        chassis = yaml.safe_load(stream)["chassis"]

    controller_params = RewrittenYaml(
        source_file=PathJoinSubstitution([
            FindPackageShare("openarmx_integrated_bringup"),
            "config",
            "integrated_o6_controllers.yaml",
        ]),
        param_rewrites={
            "wheel_radius": str(chassis["wheel_radius"]),
            "fl_pos_x": str(chassis["wheelbase"] / 2.0),
            "fl_pos_y": str(chassis["track_width"] / 2.0),
            "fr_pos_x": str(chassis["wheelbase"] / 2.0),
            "fr_pos_y": str(-chassis["track_width"] / 2.0),
            "bl_pos_x": str(-chassis["wheelbase"] / 2.0),
            "bl_pos_y": str(chassis["track_width"] / 2.0),
            "br_pos_x": str(-chassis["wheelbase"] / 2.0),
            "br_pos_y": str(-chassis["track_width"] / 2.0),
            "min_position_m": LaunchConfiguration("lift_min_height"),
            "max_position_m": LaunchConfiguration("lift_max_height"),
            "max_velocity_mps": LaunchConfiguration("lift_max_velocity_mps"),
            "lower_switch_position_m": LaunchConfiguration("lift_lower_switch_position_m"),
            "home_switch_position_m": LaunchConfiguration("lift_home_switch_position_m"),
            "upper_switch_position_m": LaunchConfiguration("lift_upper_switch_position_m"),
        },
        convert_types=True,
    )
    urdf_path = "/tmp/openarmx_integrated_o6_gravity.urdf"
    with open(urdf_path, "w", encoding="utf-8") as stream:
        stream.write(description)

    rviz_env = {
        key: value
        for key in ("DISPLAY", "WAYLAND_DISPLAY", "XAUTHORITY", "XDG_RUNTIME_DIR", "QT_QPA_PLATFORM")
        if (value := os.environ.get(key))
    }
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
        remappings=(
            [("joint_states", "/openarmx/display/joint_states")]
            if mode == "independent" else []
        ),
    )
    nodes = [
        robot_state_publisher,
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            output="both",
            parameters=[robot_description, controller_params],
        ),
        _spawn("joint_state_broadcaster", 2.0),
        _spawn("swerve_drive_controller", 3.0),
        _spawn("velocity_controller", 4.0, inactive=True),
        _spawn("lift_position_controller", 4.5, inactive=True),
        _spawn("lift_state_controller", 4.5),
        _spawn("lift_manual_position_controller", 4.8),
        _spawn("left_forward_position_controller", 6.0),
        _spawn("right_forward_position_controller", 7.0),
        _spawn("left_forward_effort_controller", 7.6),
        _spawn("right_forward_effort_controller", 7.6),
        _spawn(
            "head_forward_position_controller",
            8.0,
            condition=IfCondition(LaunchConfiguration("enable_head")),
        ),
        TimerAction(
            period=8.5,
            actions=[Node(
                package="openarmx_gravity_comp",
                executable="gravity_comp_node",
                name="gravity_comp_node",
                output="screen",
                parameters=[{
                    "urdf_path": urdf_path,
                    "g_scale": 1.05,
                    "enable_left": True,
                    "enable_right": True,
                    "verbose": False,
                }],
            )],
        ),
        TimerAction(
            period=7.0,
            actions=[Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", LaunchConfiguration("rviz_config")],
                additional_env=rviz_env,
                output="screen",
                condition=IfCondition(LaunchConfiguration("use_rviz")),
            )],
        ),
    ]
    if mode == "shared_bus":
        nodes.extend([
            Node(
                package="openarmx_hand_hardware",
                executable="o6_command_adapter",
                output="screen",
            ),
            _spawn("left_o6_position_controller", 7.2),
            _spawn("right_o6_position_controller", 7.4),
        ])
    else:
        nodes.extend(_independent_o6_nodes(context))
    return nodes


def generate_launch_description():
    lift = _lift_defaults()
    arguments = [
        DeclareLaunchArgument("use_fake_hardware", default_value="false"),
        DeclareLaunchArgument(
            "o6_hardware_mode",
            default_value="shared_bus",
            choices=["shared_bus", "independent"],
        ),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=PathJoinSubstitution([
                FindPackageShare("openarmx_integrated_description"),
                "rviz", "integrated_robot_o6.rviz",
            ]),
        ),
        DeclareLaunchArgument("chassis_steering_can", default_value="can5"),
        DeclareLaunchArgument("chassis_driving_can", default_value="can4"),
        DeclareLaunchArgument("left_arm_can", default_value="can1"),
        DeclareLaunchArgument("right_arm_can", default_value="can0"),
        DeclareLaunchArgument("lift_can", default_value="can3"),
        DeclareLaunchArgument("head_can", default_value="can2"),
        DeclareLaunchArgument("right_o6_can_interface", default_value="can6"),
        DeclareLaunchArgument("left_o6_can_interface", default_value="can7"),
        DeclareLaunchArgument("enable_head", default_value="true"),
        DeclareLaunchArgument("head_control_mode", default_value="csp"),
        DeclareLaunchArgument("control_mode", default_value="mit"),
        DeclareLaunchArgument("left_o6_can_id", default_value="0x28"),
        DeclareLaunchArgument("right_o6_can_id", default_value="0x27"),
        DeclareLaunchArgument("o6_update_rate", default_value="50.0"),
    ]
    for name, default in lift.items():
        arguments.append(DeclareLaunchArgument(name, default_value=default))
    return LaunchDescription([*arguments, OpaqueFunction(function=_launch)])
