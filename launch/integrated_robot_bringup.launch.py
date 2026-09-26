#!/usr/bin/env python3

import os
import yaml
import xacro

"""
OpenArmX Integrated Robot Bringup Launch File

This launch file starts the complete integrated system:
- 4-wheel 4-steering swerve chassis
- Upgraded lift module via lift_slide_driver (can3)
- Dual OpenArm manipulators (can0 = right, can1 = left)
"""

from launch import LaunchDescription
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess, OpaqueFunction, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, PythonExpression
from nav2_common.launch import RewrittenYaml
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def load_lift_defaults():
    defaults = {
        'can_interface': 'can3',
        'node_id': 16,
        'calibration_file': '',
        'min_position_m': -0.750,
        'max_position_m': 0.400,
        'max_velocity_mps': 0.10,
        'lower_switch_position_m': 0.000,
        'home_switch_position_m': 0.650,
        'upper_switch_position_m': 0.950,
        'switch_position_tolerance_m': 0.008,
        'counts_per_meter': 2000000.0,
        'counts_per_revolution': 10000.0,
        'profile_acceleration': 50000,
        'profile_deceleration': 50000,
        'homing_method': 27,
        'homing_speed_mps': 0.010,
        'homing_low_speed_ratio': 0.2,
        'homing_acceleration': 50000,
        'homing_timeout_sec': 60.0,
        'homing_configure_di': True,
        'di_active_low': True,
        'di6_not_func': 2,
        'di4_homing_func': 22,
        'di5_pot_func': 1,
        'home_di_channel': 4,
        'pot_di_channel': 5,
        'not_di_channel': 6,
        'invert_command': True,
        'invert_feedback': True,
        'command_timeout_sec': 0.5,
        'sdo_timeout_sec': 0.20,
        'feedback_poll_rate_hz': 20.0,
        'max_acceleration_mps2': 0.08,
        'max_deceleration_mps2': 0.12,
        'position_command_min_delta_m': 0.001,
        'target_tolerance_m': 0.0005,
        'velocity_tolerance_mps': 0.0005,
        'jog_target_lookahead_time_s': 0.25,
        'jog_min_target_lookahead_m': 0.010,
    }
    config_path = os.path.join(
        get_package_share_directory('lift_slide_bringup'),
        'config',
        'ros2_controllers.yaml',
    )
    try:
        with open(config_path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f) or {}
        joint = data.get('lift_slide_defaults', {}).get('ros__parameters', {})
        for key in defaults:
            if key in joint:
                defaults[key] = joint[key]
    except Exception as exc:
        print(f'[openarmx_integrated_bringup] Failed to load {config_path}, using built-in lift defaults: {exc}')
    def launch_value(value):
        if isinstance(value, bool):
            return 'true' if value else 'false'
        return str(value)

    return {key: launch_value(value) for key, value in defaults.items()}


def generate_integrated_robot_description(context: LaunchContext) -> str:
    xacro_path = os.path.join(
        get_package_share_directory('openarmx_integrated_description'),
        'urdf',
        'openarmx_integrated_robot.urdf.xacro',
    )
    return xacro.process_file(
        xacro_path,
        mappings={
            'use_fake_hardware': context.perform_substitution(LaunchConfiguration('use_fake_hardware')),
            'use_mock': context.perform_substitution(LaunchConfiguration('use_fake_hardware')),
            'steering_can_interface': context.perform_substitution(LaunchConfiguration('chassis_steering_can')),
            'driving_can_interface': context.perform_substitution(LaunchConfiguration('chassis_driving_can')),
            'left_arm_can_interface': context.perform_substitution(LaunchConfiguration('left_arm_can')),
            'right_arm_can_interface': context.perform_substitution(LaunchConfiguration('right_arm_can')),
            'lift_can_interface': context.perform_substitution(LaunchConfiguration('lift_can')),
            'lift_node_id': context.perform_substitution(LaunchConfiguration('lift_node_id')),
            'calibration_file': context.perform_substitution(LaunchConfiguration('lift_calibration_file')),
            'min_height': context.perform_substitution(LaunchConfiguration('lift_min_height')),
            'max_height': context.perform_substitution(LaunchConfiguration('lift_max_height')),
            'max_velocity_mps': context.perform_substitution(LaunchConfiguration('lift_max_velocity_mps')),
            'counts_per_meter': context.perform_substitution(LaunchConfiguration('lift_counts_per_meter')),
            'counts_per_revolution': context.perform_substitution(LaunchConfiguration('lift_counts_per_revolution')),
            'profile_acceleration': context.perform_substitution(LaunchConfiguration('lift_profile_acceleration')),
            'profile_deceleration': context.perform_substitution(LaunchConfiguration('lift_profile_deceleration')),
            'homing_method': context.perform_substitution(LaunchConfiguration('lift_homing_method')),
            'homing_speed_mps': context.perform_substitution(LaunchConfiguration('lift_homing_speed_mps')),
            'homing_low_speed_ratio': context.perform_substitution(LaunchConfiguration('lift_homing_low_speed_ratio')),
            'homing_acceleration': context.perform_substitution(LaunchConfiguration('lift_homing_acceleration')),
            'homing_timeout_sec': context.perform_substitution(LaunchConfiguration('lift_homing_timeout_sec')),
            'homing_configure_di': context.perform_substitution(LaunchConfiguration('lift_homing_configure_di')),
            'lower_switch_position_m': context.perform_substitution(LaunchConfiguration('lift_lower_switch_position_m')),
            'home_switch_position_m': context.perform_substitution(LaunchConfiguration('lift_home_switch_position_m')),
            'upper_switch_position_m': context.perform_substitution(LaunchConfiguration('lift_upper_switch_position_m')),
            'switch_position_tolerance_m': context.perform_substitution(LaunchConfiguration('lift_switch_position_tolerance_m')),
            'di_active_low': context.perform_substitution(LaunchConfiguration('lift_di_active_low')),
            'di6_not_func': context.perform_substitution(LaunchConfiguration('lift_di6_not_func')),
            'di4_homing_func': context.perform_substitution(LaunchConfiguration('lift_di4_homing_func')),
            'di5_pot_func': context.perform_substitution(LaunchConfiguration('lift_di5_pot_func')),
            'home_di_channel': context.perform_substitution(LaunchConfiguration('lift_home_di_channel')),
            'pot_di_channel': context.perform_substitution(LaunchConfiguration('lift_pot_di_channel')),
            'not_di_channel': context.perform_substitution(LaunchConfiguration('lift_not_di_channel')),
            'invert_command': context.perform_substitution(LaunchConfiguration('lift_invert_command')),
            'invert_feedback': context.perform_substitution(LaunchConfiguration('lift_invert_feedback')),
            'command_timeout_sec': context.perform_substitution(LaunchConfiguration('lift_command_timeout_sec')),
            'sdo_timeout_sec': context.perform_substitution(LaunchConfiguration('lift_sdo_timeout_sec')),
            'feedback_poll_rate_hz': context.perform_substitution(LaunchConfiguration('lift_feedback_poll_rate_hz')),
            'ros2_control': 'true',
            'control_mode': context.perform_substitution(LaunchConfiguration('control_mode')),
            'head_can_interface': context.perform_substitution(LaunchConfiguration('head_can')),
            'head_control_mode': context.perform_substitution(LaunchConfiguration('head_control_mode')),
            'enable_head': context.perform_substitution(LaunchConfiguration('enable_head')),
        },
    ).toprettyxml(indent='  ')


def effort_controller_spawner(context: LaunchContext):
    return [Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'left_forward_effort_controller',
            'right_forward_effort_controller',
            '-c', '/controller_manager',
            '--controller-manager-timeout', '120.0',
            '--service-call-timeout', '30.0',
            '--switch-timeout', '30.0',
        ],
        output='screen',
    )]


def gravity_comp_node_launcher(context: LaunchContext):
    robot_description = generate_integrated_robot_description(context)
    urdf_path = '/tmp/openarmx_integrated_gravity.urdf'
    with open(urdf_path, 'w') as f:
        f.write(robot_description)

    return [Node(
        package='openarmx_gravity_comp',
        executable='gravity_comp_node',
        name='gravity_comp_node',
        output='screen',
        parameters=[{
            'urdf_path': urdf_path,
            'g_scale': 1.05,
            'enable_left': True,
            'enable_right': True,
            'verbose': False,
        }],
    )]


def generate_launch_description():
    lift_defaults = load_lift_defaults()
    rviz_env = {
        key: value
        for key in ('DISPLAY', 'WAYLAND_DISPLAY', 'XAUTHORITY', 'XDG_RUNTIME_DIR', 'QT_QPA_PLATFORM')
        if (value := os.environ.get(key))
    }

    use_fake_hardware_arg = DeclareLaunchArgument(
        'use_fake_hardware',
        default_value='false',
        description='Use fake hardware for testing (no real CAN communication)'
    )

    # ---------- 底盘运动学参数 ----------
    _desc_dir = get_package_share_directory('swerve_description')
    with open(os.path.join(_desc_dir, 'config', 'chassis_version_6.0.yaml')) as f:
        _cp = yaml.safe_load(f)['chassis']
    _wheel_radius   = str(_cp['wheel_radius'])
    _half_wheelbase = str(_cp['wheelbase'] / 2.0)
    _half_track     = str(_cp['track_width'] / 2.0)

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz for visualization'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('openarmx_integrated_description'),
            'rviz',
            'integrated_robot.rviz',
        ]),
        description='RViz config file path'
    )

    chassis_steering_can_arg = DeclareLaunchArgument(
        'chassis_steering_can',
        default_value='can5',
        description='CAN interface for chassis steering motors (RS06)'
    )

    chassis_driving_can_arg = DeclareLaunchArgument(
        'chassis_driving_can',
        default_value='can4',
        description='CAN interface for chassis driving motors (UM)'
    )

    left_arm_can_arg = DeclareLaunchArgument(
        'left_arm_can',
        default_value='can1',
        description='CAN interface for left arm motors (8 DOF)'
    )

    right_arm_can_arg = DeclareLaunchArgument(
        'right_arm_can',
        default_value='can0',
        description='CAN interface for right arm motors (8 DOF)'
    )

    lift_can_arg = DeclareLaunchArgument(
        'lift_can',
        default_value=lift_defaults['can_interface'],
        description='CAN interface for lift module'
    )

    head_can_arg = DeclareLaunchArgument(
        'head_can',
        default_value='can2',
        description='CAN interface for head motors (2 DOF, pitch + yaw)'
    )

    enable_head_arg = DeclareLaunchArgument(
        'enable_head',
        default_value='true',
        description='Enable integrated head model, hardware, and controller'
    )

    head_control_mode_arg = DeclareLaunchArgument(
        'head_control_mode',
        default_value='csp',
        description='Head motor control mode: mit or csp'
    )

    lift_node_id_arg = DeclareLaunchArgument(
        'lift_node_id',
        default_value=lift_defaults['node_id'],
        description='CANopen node id for lift module'
    )

    lift_calibration_file_arg = DeclareLaunchArgument(
        'lift_calibration_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('lift_slide_driver'), 'config', 'lift_slide_calibration.yaml'
        ]),
        description='Package-owned lift zero calibration record'
    )

    lift_min_height_arg = DeclareLaunchArgument(
        'lift_min_height',
        default_value=lift_defaults['min_position_m'],
        description='Lift minimum position in meters'
    )

    lift_max_height_arg = DeclareLaunchArgument(
        'lift_max_height',
        default_value=lift_defaults['max_position_m'],
        description='Lift maximum position in meters'
    )

    lift_max_velocity_mps_arg = DeclareLaunchArgument(
        'lift_max_velocity_mps',
        default_value=lift_defaults['max_velocity_mps'],
        description='Lift maximum velocity in meters per second'
    )

    lift_counts_per_meter_arg = DeclareLaunchArgument(
        'lift_counts_per_meter',
        default_value=lift_defaults['counts_per_meter'],
        description='Lift encoder counts per meter'
    )

    lift_counts_per_revolution_arg = DeclareLaunchArgument(
        'lift_counts_per_revolution',
        default_value=lift_defaults['counts_per_revolution'],
        description='Lift encoder counts per revolution'
    )

    lift_profile_acceleration_arg = DeclareLaunchArgument(
        'lift_profile_acceleration',
        default_value=lift_defaults['profile_acceleration'],
        description='Lift CANopen 6083 profile acceleration'
    )

    lift_profile_deceleration_arg = DeclareLaunchArgument(
        'lift_profile_deceleration',
        default_value=lift_defaults['profile_deceleration'],
        description='Lift CANopen 6084 profile deceleration'
    )

    lift_homing_method_arg = DeclareLaunchArgument(
        'lift_homing_method',
        default_value=lift_defaults['homing_method'],
        description='Lift CANopen 6098 homing method'
    )

    lift_homing_speed_mps_arg = DeclareLaunchArgument(
        'lift_homing_speed_mps',
        default_value=lift_defaults['homing_speed_mps'],
        description='Lift homing speed in meters per second'
    )

    lift_homing_low_speed_ratio_arg = DeclareLaunchArgument(
        'lift_homing_low_speed_ratio',
        default_value=lift_defaults['homing_low_speed_ratio'],
        description='Lift low-speed homing ratio'
    )

    lift_homing_acceleration_arg = DeclareLaunchArgument(
        'lift_homing_acceleration',
        default_value=lift_defaults['homing_acceleration'],
        description='Lift CANopen 609A homing acceleration'
    )

    lift_homing_timeout_sec_arg = DeclareLaunchArgument(
        'lift_homing_timeout_sec',
        default_value=lift_defaults['homing_timeout_sec'],
        description='Lift homing timeout in seconds'
    )

    lift_homing_configure_di_arg = DeclareLaunchArgument(
        'lift_homing_configure_di',
        default_value=lift_defaults['homing_configure_di'],
        description='Configure lift DI functions before homing'
    )

    lift_lower_switch_position_arg = DeclareLaunchArgument(
        'lift_lower_switch_position_m',
        default_value=lift_defaults['lower_switch_position_m'],
        description='Lift lower limit switch position in meters'
    )

    lift_home_switch_position_arg = DeclareLaunchArgument(
        'lift_home_switch_position_m',
        default_value=lift_defaults['home_switch_position_m'],
        description='Lift home switch position in meters'
    )

    lift_upper_switch_position_arg = DeclareLaunchArgument(
        'lift_upper_switch_position_m',
        default_value=lift_defaults['upper_switch_position_m'],
        description='Lift upper limit switch position in meters'
    )

    lift_switch_position_tolerance_arg = DeclareLaunchArgument(
        'lift_switch_position_tolerance_m',
        default_value=lift_defaults['switch_position_tolerance_m'],
        description='Lift switch position tolerance in meters'
    )

    lift_di_active_low_arg = DeclareLaunchArgument(
        'lift_di_active_low',
        default_value=lift_defaults['di_active_low'],
        description='Whether lift DI inputs are active low'
    )

    lift_di6_not_func_arg = DeclareLaunchArgument(
        'lift_di6_not_func',
        default_value=lift_defaults['di6_not_func'],
        description='Lift DI6 NOT function code'
    )

    lift_di4_homing_func_arg = DeclareLaunchArgument(
        'lift_di4_homing_func',
        default_value=lift_defaults['di4_homing_func'],
        description='Lift DI4 HOME function code'
    )

    lift_di5_pot_func_arg = DeclareLaunchArgument(
        'lift_di5_pot_func',
        default_value=lift_defaults['di5_pot_func'],
        description='Lift DI5 POT function code'
    )

    lift_home_di_channel_arg = DeclareLaunchArgument(
        'lift_home_di_channel',
        default_value=lift_defaults['home_di_channel'],
        description='Lift HOME DI channel'
    )

    lift_pot_di_channel_arg = DeclareLaunchArgument(
        'lift_pot_di_channel',
        default_value=lift_defaults['pot_di_channel'],
        description='Lift lower limit DI channel'
    )

    lift_not_di_channel_arg = DeclareLaunchArgument(
        'lift_not_di_channel',
        default_value=lift_defaults['not_di_channel'],
        description='Lift upper limit DI channel'
    )

    lift_invert_command_arg = DeclareLaunchArgument(
        'lift_invert_command',
        default_value=lift_defaults['invert_command'],
        description='Invert lift command direction'
    )

    lift_invert_feedback_arg = DeclareLaunchArgument(
        'lift_invert_feedback',
        default_value=lift_defaults['invert_feedback'],
        description='Invert lift feedback direction'
    )

    lift_command_timeout_sec_arg = DeclareLaunchArgument(
        'lift_command_timeout_sec',
        default_value=lift_defaults['command_timeout_sec'],
        description='Lift command timeout in seconds'
    )

    lift_sdo_timeout_sec_arg = DeclareLaunchArgument(
        'lift_sdo_timeout_sec',
        default_value=lift_defaults['sdo_timeout_sec'],
        description='Lift SDO timeout in seconds'
    )

    lift_feedback_poll_rate_hz_arg = DeclareLaunchArgument(
        'lift_feedback_poll_rate_hz',
        default_value=lift_defaults['feedback_poll_rate_hz'],
        description='Lift feedback polling rate in Hz'
    )

    control_mode_arg = DeclareLaunchArgument(
        'control_mode',
        default_value='mit',
        description='Motor control mode: mit or csp'
    )

    enable_forward_effort_arg = DeclareLaunchArgument(
        'enable_forward_effort',
        default_value='true',
        description='Enable gravity compensation feedforward for both arms'
    )

    use_scene_arg = DeclareLaunchArgument(
        'use_scene',
        default_value='false',
        description='Launch RViz scene publisher for test environment visualization'
    )

    auto_homing_arg = DeclareLaunchArgument(
        'auto_homing',
        default_value='false',
        description='Auto call /lift_slide_driver/enable and /lift_slide_driver/start_homing on real hardware'
    )

    auto_homing_enable_delay_arg = DeclareLaunchArgument(
        'auto_homing_enable_delay',
        default_value='9.0',
        description='Delay (s) before calling /lift_slide_driver/enable when auto_homing=true'
    )

    auto_homing_start_delay_arg = DeclareLaunchArgument(
        'auto_homing_start_delay',
        default_value='12.0',
        description='Delay (s) before calling /lift_slide_driver/start_homing when auto_homing=true'
    )

    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare('openarmx_integrated_description'),
            'urdf',
            'openarmx_integrated_robot.urdf.xacro'
        ]),
        ' use_fake_hardware:=', LaunchConfiguration('use_fake_hardware'),
        ' use_mock:=', LaunchConfiguration('use_fake_hardware'),
        ' steering_can_interface:=', LaunchConfiguration('chassis_steering_can'),
        ' driving_can_interface:=', LaunchConfiguration('chassis_driving_can'),
        ' left_arm_can_interface:=', LaunchConfiguration('left_arm_can'),
        ' right_arm_can_interface:=', LaunchConfiguration('right_arm_can'),
        ' lift_can_interface:=', LaunchConfiguration('lift_can'),
        ' lift_node_id:=', LaunchConfiguration('lift_node_id'),
        ' calibration_file:=', LaunchConfiguration('lift_calibration_file'),
        ' min_height:=', LaunchConfiguration('lift_min_height'),
        ' max_height:=', LaunchConfiguration('lift_max_height'),
        ' max_velocity_mps:=', LaunchConfiguration('lift_max_velocity_mps'),
        ' counts_per_meter:=', LaunchConfiguration('lift_counts_per_meter'),
        ' counts_per_revolution:=', LaunchConfiguration('lift_counts_per_revolution'),
        ' profile_acceleration:=', LaunchConfiguration('lift_profile_acceleration'),
        ' profile_deceleration:=', LaunchConfiguration('lift_profile_deceleration'),
        ' homing_method:=', LaunchConfiguration('lift_homing_method'),
        ' homing_speed_mps:=', LaunchConfiguration('lift_homing_speed_mps'),
        ' homing_low_speed_ratio:=', LaunchConfiguration('lift_homing_low_speed_ratio'),
        ' homing_acceleration:=', LaunchConfiguration('lift_homing_acceleration'),
        ' homing_timeout_sec:=', LaunchConfiguration('lift_homing_timeout_sec'),
        ' homing_configure_di:=', LaunchConfiguration('lift_homing_configure_di'),
        ' lower_switch_position_m:=', LaunchConfiguration('lift_lower_switch_position_m'),
        ' home_switch_position_m:=', LaunchConfiguration('lift_home_switch_position_m'),
        ' upper_switch_position_m:=', LaunchConfiguration('lift_upper_switch_position_m'),
        ' switch_position_tolerance_m:=', LaunchConfiguration('lift_switch_position_tolerance_m'),
        ' di_active_low:=', LaunchConfiguration('lift_di_active_low'),
        ' di6_not_func:=', LaunchConfiguration('lift_di6_not_func'),
        ' di4_homing_func:=', LaunchConfiguration('lift_di4_homing_func'),
        ' di5_pot_func:=', LaunchConfiguration('lift_di5_pot_func'),
        ' home_di_channel:=', LaunchConfiguration('lift_home_di_channel'),
        ' pot_di_channel:=', LaunchConfiguration('lift_pot_di_channel'),
        ' not_di_channel:=', LaunchConfiguration('lift_not_di_channel'),
        ' invert_command:=', LaunchConfiguration('lift_invert_command'),
        ' invert_feedback:=', LaunchConfiguration('lift_invert_feedback'),
        ' command_timeout_sec:=', LaunchConfiguration('lift_command_timeout_sec'),
        ' sdo_timeout_sec:=', LaunchConfiguration('lift_sdo_timeout_sec'),
        ' feedback_poll_rate_hz:=', LaunchConfiguration('lift_feedback_poll_rate_hz'),
        ' ros2_control:=true',
        ' control_mode:=', LaunchConfiguration('control_mode'),
        ' head_can_interface:=', LaunchConfiguration('head_can'),
        ' head_control_mode:=', LaunchConfiguration('head_control_mode'),
        ' enable_head:=', LaunchConfiguration('enable_head'),
    ])

    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='both',
        parameters=[
            robot_description,
            RewrittenYaml(
                source_file=PathJoinSubstitution([
                    FindPackageShare('openarmx_integrated_bringup'),
                    'config',
                    'integrated_controllers.yaml'
                ]),
                param_rewrites={
                    'wheel_radius': _wheel_radius,
                    'fl_pos_x':  _half_wheelbase,
                    'fl_pos_y':  _half_track,
                    'fr_pos_x':  _half_wheelbase,
                    'fr_pos_y': f'-{_half_track}',
                    'bl_pos_x': f'-{_half_wheelbase}',
                    'bl_pos_y':  _half_track,
                    'br_pos_x': f'-{_half_wheelbase}',
                    'br_pos_y': f'-{_half_track}',
                    'min_position_m': LaunchConfiguration('lift_min_height'),
                    'max_position_m': LaunchConfiguration('lift_max_height'),
                    'max_velocity_mps': LaunchConfiguration('lift_max_velocity_mps'),
                    'lower_switch_position_m': LaunchConfiguration('lift_lower_switch_position_m'),
                    'home_switch_position_m': LaunchConfiguration('lift_home_switch_position_m'),
                    'upper_switch_position_m': LaunchConfiguration('lift_upper_switch_position_m'),
                    'max_acceleration_mps2': lift_defaults['max_acceleration_mps2'],
                    'max_deceleration_mps2': lift_defaults['max_deceleration_mps2'],
                    'position_command_min_delta_m': lift_defaults['position_command_min_delta_m'],
                    'target_tolerance_m': lift_defaults['target_tolerance_m'],
                    'velocity_tolerance_mps': lift_defaults['velocity_tolerance_mps'],
                    'jog_target_lookahead_time_s': lift_defaults['jog_target_lookahead_time_s'],
                    'jog_min_target_lookahead_m': lift_defaults['jog_min_target_lookahead_m'],
                },
                convert_types=True,
            )
        ]
    )

    joint_state_broadcaster_spawner = TimerAction(
        period=2.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'joint_state_broadcaster',
                '-c', '/controller_manager',
                '--controller-manager-timeout', '120.0',
                '--service-call-timeout', '30.0',
                '--switch-timeout', '30.0',
            ],
            output='screen'
        )]
    )

    joint_state_broadcaster_recover = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'bash', '-lc',
                    'source install/setup.bash && '
                    'ros2 control set_controller_state joint_state_broadcaster inactive || true; '
                    'ros2 control set_controller_state joint_state_broadcaster active || true'
                ],
                cwd=os.path.expanduser('~/openflex_all/openflex_ws'),
                output='screen',
            )
        ],
    )

    chassis_controller_spawner = TimerAction(
        period=3.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'swerve_drive_controller',
                '-c', '/controller_manager',
                '--controller-manager-timeout', '120.0',
                '--service-call-timeout', '30.0',
                '--switch-timeout', '30.0',
            ],
            output='screen'
        )]
    )

    lift_velocity_controller_spawner = TimerAction(
        period=4.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'velocity_controller',
                '-c', '/controller_manager',
                '--controller-manager-timeout', '120.0',
                '--service-call-timeout', '30.0',
                '--switch-timeout', '30.0',
                '--inactive',
            ],
            output='screen'
        )]
    )

    lift_position_controller_spawner = TimerAction(
        period=4.5,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'lift_position_controller',
                '-c', '/controller_manager',
                '--controller-manager-timeout', '120.0',
                '--service-call-timeout', '30.0',
                '--switch-timeout', '30.0',
                '--inactive',
            ],
            output='screen'
        )]
    )

    lift_manual_position_controller_spawner = TimerAction(
        period=4.8,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'lift_manual_position_controller',
                '-c', '/controller_manager',
                '--controller-manager-timeout', '120.0',
                '--service-call-timeout', '30.0',
                '--switch-timeout', '30.0',
            ],
            output='screen'
        )]
    )

    lift_state_controller_spawner = TimerAction(
        period=4.5,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'lift_state_controller',
                '-c', '/controller_manager',
                '--controller-manager-timeout', '120.0',
                '--service-call-timeout', '30.0',
                '--switch-timeout', '30.0',
            ],
            output='screen'
        )]
    )

    left_arm_controller_spawner = TimerAction(
        period=6.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'left_forward_position_controller',
                '-c', '/controller_manager',
                '--controller-manager-timeout', '120.0',
                '--service-call-timeout', '30.0',
                '--switch-timeout', '30.0',
            ],
            output='screen'
        )]
    )

    right_arm_controller_spawner = TimerAction(
        period=7.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'right_forward_position_controller',
                '-c', '/controller_manager',
                '--controller-manager-timeout', '120.0',
                '--service-call-timeout', '30.0',
                '--switch-timeout', '30.0',
            ],
            output='screen'
        )]
    )

    head_controller_spawner = TimerAction(
        period=8.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'head_forward_position_controller',
                '-c', '/controller_manager',
                '--controller-manager-timeout', '120.0',
                '--service-call-timeout', '30.0',
                '--switch-timeout', '30.0',
            ],
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_head'))
        )]
    )

    effort_controller_spawner_func = OpaqueFunction(function=effort_controller_spawner)
    gravity_comp_node_launcher_func = OpaqueFunction(function=gravity_comp_node_launcher)

    delayed_effort_controller = TimerAction(
        period=7.5,
        actions=[effort_controller_spawner_func],
    )

    delayed_gravity_comp = TimerAction(
        period=8.5,
        actions=[gravity_comp_node_launcher_func],
    )

    forward_effort_group = GroupAction(
        condition=IfCondition(LaunchConfiguration('enable_forward_effort')),
        actions=[
            delayed_effort_controller,
            delayed_gravity_comp,
        ],
    )

    rviz_node = TimerAction(
        period=7.0,
        actions=[Node(
            package='rviz2',
            executable='rviz2',
            arguments=[
                '-d', LaunchConfiguration('rviz_config'),
                '--ros-args',
                '-p', ['pos_min:=', LaunchConfiguration('lift_min_height')],
                '-p', ['pos_max:=', LaunchConfiguration('lift_max_height')],
            ],
            # Keep the current desktop session display environment so RViz
            # can be launched correctly from the GUI or desktop shortcut.
            additional_env=rviz_env,
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_rviz'))
        )]
    )

    scene_publisher_node = TimerAction(
        period=2.0,
        actions=[Node(
            package='openarmx_integrated_bringup',
            executable='rviz_scene_publisher',
            name='rviz_scene_publisher',
            output='screen',
            parameters=[{
                'frame_id': 'odom',
            }],
            condition=IfCondition(LaunchConfiguration('use_scene'))
        )]
    )

    auto_homing_condition = IfCondition(PythonExpression([
        "'", LaunchConfiguration('auto_homing'), "' == 'true' and '",
        LaunchConfiguration('use_fake_hardware'), "' == 'false'"
    ]))

    auto_enable_lift_motor = TimerAction(
        period=LaunchConfiguration('auto_homing_enable_delay'),
        actions=[ExecuteProcess(
            cmd=[
                'ros2', 'service', 'call',
                '/lift_slide_driver/enable',
                'std_srvs/srv/Trigger',
                '{}'
            ],
            output='screen'
        )],
        condition=auto_homing_condition
    )

    auto_start_homing = TimerAction(
        period=LaunchConfiguration('auto_homing_start_delay'),
        actions=[ExecuteProcess(
            cmd=[
                'ros2', 'service', 'call',
                '/lift_slide_driver/start_homing',
                'std_srvs/srv/Trigger',
                '{}'
            ],
            output='screen'
        )],
        condition=auto_homing_condition
    )

    return LaunchDescription([
        use_fake_hardware_arg,
        use_rviz_arg,
        rviz_config_arg,
        use_scene_arg,
        chassis_steering_can_arg,
        chassis_driving_can_arg,
        left_arm_can_arg,
        right_arm_can_arg,
        lift_can_arg,
        head_can_arg,
        enable_head_arg,
        head_control_mode_arg,
        lift_node_id_arg,
        lift_calibration_file_arg,
        lift_min_height_arg,
        lift_max_height_arg,
        lift_max_velocity_mps_arg,
        lift_counts_per_meter_arg,
        lift_counts_per_revolution_arg,
        lift_profile_acceleration_arg,
        lift_profile_deceleration_arg,
        lift_homing_method_arg,
        lift_homing_speed_mps_arg,
        lift_homing_low_speed_ratio_arg,
        lift_homing_acceleration_arg,
        lift_homing_timeout_sec_arg,
        lift_homing_configure_di_arg,
        lift_lower_switch_position_arg,
        lift_home_switch_position_arg,
        lift_upper_switch_position_arg,
        lift_switch_position_tolerance_arg,
        lift_di_active_low_arg,
        lift_di6_not_func_arg,
        lift_di4_homing_func_arg,
        lift_di5_pot_func_arg,
        lift_home_di_channel_arg,
        lift_pot_di_channel_arg,
        lift_not_di_channel_arg,
        lift_invert_command_arg,
        lift_invert_feedback_arg,
        lift_command_timeout_sec_arg,
        lift_sdo_timeout_sec_arg,
        lift_feedback_poll_rate_hz_arg,
        control_mode_arg,
        enable_forward_effort_arg,
        auto_homing_arg,
        auto_homing_enable_delay_arg,
        auto_homing_start_delay_arg,
        robot_state_publisher_node,
        controller_manager_node,
        joint_state_broadcaster_spawner,
        joint_state_broadcaster_recover,
        chassis_controller_spawner,
        lift_velocity_controller_spawner,
        lift_position_controller_spawner,
        lift_manual_position_controller_spawner,
        lift_state_controller_spawner,
        left_arm_controller_spawner,
        right_arm_controller_spawner,
        head_controller_spawner,
        forward_effort_group,
        rviz_node,
        scene_publisher_node,
        auto_enable_lift_motor,
        auto_start_homing,
    ])
