#!/usr/bin/env python3
"""
OpenArmX 集成系统 VR 遥操作启动文件 (v2)

功能：
1. 启动 pico_pose_bridge - 接收VR手柄UDP数据，发布姿态和按键话题
2. 启动 waist_chassis_control_node - 腰部位置控制底盘平移+旋转+升降台
3. 启动 openarmx_teleop_vr - 双臂VR IK控制
4. (可选) vr_teleop_node - 摇杆底盘控制 (默认开启, enable_joystick_control:=false 关闭)
5. (可选) vr_lift_control_node - XY按键升降台 (默认开启, enable_button_lift:=false 关闭)
6. (可选) head_teleop_node - 头部VR遥控 (默认开启, enable_head_teleop:=false 关闭)

使用说明：
1. 先启动集成机器人系统:
   ros2 launch openarmx_integrated_bringup integrated_robot_bringup.launch.py use_fake_hardware:=true use_rviz:=true

2. 再启动VR遥操作:
   ros2 launch openarmx_integrated_bringup integrated_vr_teleop.launch.py

3. 腰部控制（启用后）:
   - 左手或右手食指 Trigger 任意一个超过阈值即可激活腰控
   - 前后移动腰部: 底盘前进/后退
   - 左右移动腰部: 底盘左右平移
   - 扭动腰部: 底盘旋转
   - 上下移动腰部: 升降台升降
   - 两个 Trigger 都松开: 全部停止

4. 默认已开启摇杆底盘控制和按键升降；如需关闭底盘:
   ros2 launch openarmx_integrated_bringup integrated_vr_teleop.launch.py enable_joystick_control:=false

5. 如需使用 Pico 下发的底盘线速度/角速度上限:
   ros2 launch openarmx_integrated_bringup integrated_vr_teleop.launch.py enable_joystick_control:=true vr_chassis:=true

当前默认按键约定：
- 左手摇杆：底盘前后/左右平移
- 右手摇杆：底盘旋转
- 右手 A：单击切换双臂速度档位（由 VR 发送端处理），双击切换底盘速度档位
- 右手 B：头部相对控制启停
- 左手 X/Y：升降台下降/上升
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def _launch_value(value):
    if isinstance(value, bool):
        return 'true' if value else 'false'
    if isinstance(value, list):
        return str(value)
    return str(value)


def _load_vr_defaults():
    defaults = {
        'max_linear_speed': 0.3,
        'boost_linear_speed': 0.3,
        'max_angular_speed': 0.3,
        'joystick_deadzone': 0.15,
        'linear_expo': 1.0,
        'angular_expo': 1.0,
        'estop_toggle_topic': '',
        'enable_waist_control': False,
        'vr_chassis': False,
        'enable_joystick_control': True,
        'enable_button_lift': True,
        'enable_head_teleop': True,
        'robot_type': 'auto',
        'waist_position_gain': 1.0,
        'waist_lift_gain': 1.0,
        'waist_angular_gain': 1.0,
        'waist_max_linear_vel': 0.5,
        'waist_lift_speed': 0.2,
        'arm_teleop_config_file': '',
        'left_axis_matrix': [0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0],
        'right_axis_matrix': [0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0],
        'left_orientation_matrix': [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        'right_orientation_matrix': [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        'head_startup_home_enabled': True,
        'head_startup_home_step_deg': 1.0,
        'head_enable_soft_limits': True,
        'head_yaw_soft_margin_deg': 10.0,
        'head_pitch_soft_margin_deg': 5.0,
        'head_soft_limit_blend_deg': 10.0,
        'enable_scurve': True,
        'acceleration_time': 0.5,
        'smoothness': 0.3,
    }
    config_path = os.path.join(
        get_package_share_directory('openarmx_integrated_bringup'),
        'config',
        'vr_teleop.yaml',
    )
    try:
        with open(config_path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f) or {}
        params = data.get('vr_teleop_defaults', {}).get('ros__parameters', {})
        for key in defaults:
            if key in params:
                defaults[key] = params[key]
    except Exception as exc:
        print(f'[integrated_vr_teleop] Failed to load {config_path}, using built-in defaults: {exc}')
    return {key: _launch_value(value) for key, value in defaults.items()}


def generate_launch_description():
    vr_defaults = _load_vr_defaults()
    default_vr_config_file = PathJoinSubstitution([
        FindPackageShare('openarmx_integrated_bringup'),
        'config',
        'vr_teleop.yaml',
    ])
    default_arm_teleop_config_file = PathJoinSubstitution([
        FindPackageShare('openarmx_teleop_vr'),
        'config',
        'teleop_params.yaml',
    ])

    # 声明参数
    declare_vr_config_file = DeclareLaunchArgument(
        'vr_config_file',
        default_value=default_vr_config_file,
        description='整机 VR 遥操作参数文件'
    )

    declare_max_linear_speed = DeclareLaunchArgument(
        'max_linear_speed',
        default_value=vr_defaults['max_linear_speed'],
        description='底盘最大线速度 (m/s)'
    )

    declare_max_angular_speed = DeclareLaunchArgument(
        'max_angular_speed',
        default_value=vr_defaults['max_angular_speed'],
        description='底盘最大角速度 (rad/s)'
    )

    declare_boost_linear_speed = DeclareLaunchArgument(
        'boost_linear_speed',
        default_value=vr_defaults['boost_linear_speed'],
        description='底盘加速模式最大线速度 (m/s)'
    )

    declare_joystick_deadzone = DeclareLaunchArgument(
        'joystick_deadzone',
        default_value=vr_defaults['joystick_deadzone'],
        description='摇杆死区 (0-1)'
    )

    declare_linear_expo = DeclareLaunchArgument(
        'linear_expo',
        default_value=vr_defaults['linear_expo'],
        description='摇杆线速度 expo 曲线，1.0=线性，越大越细'
    )

    declare_angular_expo = DeclareLaunchArgument(
        'angular_expo',
        default_value=vr_defaults['angular_expo'],
        description='摇杆角速度 expo 曲线，1.0=线性，越大越细'
    )

    declare_estop_toggle_topic = DeclareLaunchArgument(
        'estop_toggle_topic',
        default_value=vr_defaults['estop_toggle_topic'],
        description='底盘外部紧停切换话题；默认禁用，防止摇杆误触导致底盘锁死。'
                    '如需启用可设为 /pico_right_controller/joystick_click'
    )

    declare_enable_waist_control = DeclareLaunchArgument(
        'enable_waist_control',
        default_value=vr_defaults['enable_waist_control'],
        description='启用腰部位置控制底盘+升降台 (true/false)'
    )

    declare_vr_chassis = DeclareLaunchArgument(
        'vr_chassis',
        default_value=vr_defaults['vr_chassis'],
        description='Use VR-provided chassis linear/angular speed limits (true/false)'
    )

    declare_enable_joystick_control = DeclareLaunchArgument(
        'enable_joystick_control',
        default_value=vr_defaults['enable_joystick_control'],
        description='启用摇杆底盘控制 (默认开启)'
    )

    declare_enable_button_lift = DeclareLaunchArgument(
        'enable_button_lift',
        default_value=vr_defaults['enable_button_lift'],
        description='启用XY按键升降台控制 (默认开启)'
    )

    declare_waist_position_gain = DeclareLaunchArgument(
        'waist_position_gain',
        default_value=vr_defaults['waist_position_gain'],
        description='腰部位移→底盘位移比例系数'
    )

    declare_waist_lift_gain = DeclareLaunchArgument(
        'waist_lift_gain',
        default_value=vr_defaults['waist_lift_gain'],
        description='腰部Y位移→升降台位移比例系数'
    )

    declare_waist_angular_gain = DeclareLaunchArgument(
        'waist_angular_gain',
        default_value=vr_defaults['waist_angular_gain'],
        description='腰部yaw→底盘theta比例系数'
    )

    declare_waist_max_linear_vel = DeclareLaunchArgument(
        'waist_max_linear_vel',
        default_value=vr_defaults['waist_max_linear_vel'],
        description='底盘最大线速度 (m/s)'
    )

    declare_waist_lift_speed = DeclareLaunchArgument(
        'waist_lift_speed',
        default_value=vr_defaults['waist_lift_speed'],
        description='升降台最大速度'
    )

    declare_enable_head_teleop = DeclareLaunchArgument(
        'enable_head_teleop',
        default_value=vr_defaults['enable_head_teleop'],
        description='启用头部VR遥控 (默认开启)'
    )

    declare_arm_teleop_config_file = DeclareLaunchArgument(
        'arm_teleop_config_file',
        default_value=vr_defaults['arm_teleop_config_file'] or default_arm_teleop_config_file,
        description='双臂 VR IK 参数文件'
    )

    declare_robot_type = DeclareLaunchArgument(
        'robot_type',
        default_value=vr_defaults['robot_type'],
        description='整机末端类型：auto=随 VR 模式切换，gripper=固定 8 值，o6=固定 7 值'
    )

    declare_left_axis_matrix = DeclareLaunchArgument(
        'left_axis_matrix',
        default_value=vr_defaults['left_axis_matrix'],
        description='左手柄位置轴映射矩阵'
    )

    declare_right_axis_matrix = DeclareLaunchArgument(
        'right_axis_matrix',
        default_value=vr_defaults['right_axis_matrix'],
        description='右手柄位置轴映射矩阵'
    )

    declare_left_orientation_matrix = DeclareLaunchArgument(
        'left_orientation_matrix',
        default_value=vr_defaults['left_orientation_matrix'],
        description='左手柄姿态轴映射矩阵'
    )

    declare_right_orientation_matrix = DeclareLaunchArgument(
        'right_orientation_matrix',
        default_value=vr_defaults['right_orientation_matrix'],
        description='右手柄姿态轴映射矩阵'
    )

    declare_head_startup_home_enabled = DeclareLaunchArgument(
        'head_startup_home_enabled',
        default_value=vr_defaults['head_startup_home_enabled'],
        description='头部 VR 节点启动时是否自动回零'
    )

    declare_head_startup_home_step_deg = DeclareLaunchArgument(
        'head_startup_home_step_deg',
        default_value=vr_defaults['head_startup_home_step_deg'],
        description='头部上电回零每周期最大步长 (度)'
    )

    declare_head_enable_soft_limits = DeclareLaunchArgument(
        'head_enable_soft_limits',
        default_value=vr_defaults['head_enable_soft_limits'],
        description='启用头部软件软限位'
    )

    declare_head_yaw_soft_margin_deg = DeclareLaunchArgument(
        'head_yaw_soft_margin_deg',
        default_value=vr_defaults['head_yaw_soft_margin_deg'],
        description='头部 yaw 软限位安全余量 (度)'
    )

    declare_head_pitch_soft_margin_deg = DeclareLaunchArgument(
        'head_pitch_soft_margin_deg',
        default_value=vr_defaults['head_pitch_soft_margin_deg'],
        description='头部 pitch 软限位安全余量 (度)'
    )

    declare_head_soft_limit_blend_deg = DeclareLaunchArgument(
        'head_soft_limit_blend_deg',
        default_value=vr_defaults['head_soft_limit_blend_deg'],
        description='头部软限位平滑过渡区宽度 (度)'
    )

    # S型加减速参数
    declare_enable_scurve = DeclareLaunchArgument(
        'enable_scurve',
        default_value=vr_defaults['enable_scurve'],
        description='启用S型加减速平滑控制 (true/false)'
    )

    declare_acceleration_time = DeclareLaunchArgument(
        'acceleration_time',
        default_value=vr_defaults['acceleration_time'],
        description='加速时间：从0加速到最大速度的时间 (秒)'
    )

    declare_smoothness = DeclareLaunchArgument(
        'smoothness',
        default_value=vr_defaults['smoothness'],
        description='平滑度：S曲线的弯曲程度 (0-1)，越大越平滑'
    )

    # 1. pico_pose_bridge - VR姿态桥接节点
    pico_pose_bridge_node = Node(
        package='openflex_vr_bridge',
        executable='pico_pose_bridge_node',
        name='pico_pose_bridge',
        output='screen',
        parameters=[LaunchConfiguration('vr_config_file')]
    )

    # 2. vr_teleop_node - 摇杆底盘控制 (默认关闭)
    vr_teleop_node = Node(
        package='swerve_bringup',
        executable='vr_teleop_node',
        name='vr_teleop_chassis',
        output='screen',
        parameters=[
            LaunchConfiguration('vr_config_file'),
            {
                'max_linear_speed': ParameterValue(LaunchConfiguration('max_linear_speed'), value_type=float),
                'boost_linear_speed': ParameterValue(LaunchConfiguration('boost_linear_speed'), value_type=float),
                'max_angular_speed': ParameterValue(LaunchConfiguration('max_angular_speed'), value_type=float),
                'joystick_deadzone': ParameterValue(LaunchConfiguration('joystick_deadzone'), value_type=float),
                'linear_expo': ParameterValue(LaunchConfiguration('linear_expo'), value_type=float),
                'angular_expo': ParameterValue(LaunchConfiguration('angular_expo'), value_type=float),
                'estop_toggle_topic': LaunchConfiguration('estop_toggle_topic'),
                'use_vr_chassis_speed_config': ParameterValue(
                    LaunchConfiguration('vr_chassis'), value_type=bool),
                'enable_scurve': ParameterValue(LaunchConfiguration('enable_scurve'), value_type=bool),
                'acceleration_time': ParameterValue(LaunchConfiguration('acceleration_time'), value_type=float),
                'smoothness': ParameterValue(LaunchConfiguration('smoothness'), value_type=float),
            },
        ],
        condition=IfCondition(LaunchConfiguration('enable_joystick_control'))
    )

    # 3. vr_lift_control_node - XY按键升降台手动长按控制 (默认开启)
    vr_lift_control_node = Node(
        package='swerve_bringup',
        executable='vr_lift_control_node',
        name='vr_lift_control',
        output='screen',
        parameters=[LaunchConfiguration('vr_config_file')],
        condition=IfCondition(LaunchConfiguration('enable_button_lift'))
    )

    # 4. waist_chassis_control_node - 腰部增量位置控制底盘+升降台 (odom闭环)
    waist_chassis_control_node = Node(
        package='swerve_bringup',
        executable='waist_chassis_control_node',
        name='waist_chassis_control',
        output='screen',
        parameters=[
            LaunchConfiguration('vr_config_file'),
            {
                'position_gain': ParameterValue(LaunchConfiguration('waist_position_gain'), value_type=float),
                'lift_gain': ParameterValue(LaunchConfiguration('waist_lift_gain'), value_type=float),
                'angular_gain': ParameterValue(LaunchConfiguration('waist_angular_gain'), value_type=float),
                'max_linear_vel': ParameterValue(LaunchConfiguration('waist_max_linear_vel'), value_type=float),
                'lift_speed': ParameterValue(LaunchConfiguration('waist_lift_speed'), value_type=float),
            },
        ],
        condition=IfCondition(LaunchConfiguration('enable_waist_control'))
    )

    # 5. openarmx_teleop_vr - 双臂VR IK控制
    arm_teleop_node = Node(
        package='openarmx_teleop_vr',
        executable='openarmx_teleop_vr_node',
        name='openarmx_teleop_vr_node',
        output='screen',
        parameters=[
            LaunchConfiguration('vr_config_file'),
            LaunchConfiguration('arm_teleop_config_file'),
            {
                'urdf_path': PathJoinSubstitution([
                    FindPackageShare('openarmx_description'),
                    'urdf',
                    'robot',
                    'openarmx_robot.urdf',
                ]),
                'robot_type': LaunchConfiguration('robot_type'),
                'left_axis_matrix': LaunchConfiguration('left_axis_matrix'),
                'right_axis_matrix': LaunchConfiguration('right_axis_matrix'),
                'left_orientation_matrix': LaunchConfiguration('left_orientation_matrix'),
                'right_orientation_matrix': LaunchConfiguration('right_orientation_matrix'),
            }
        ]
    )

    # 6. head_teleop_node - 头部VR遥控 (默认开启)
    head_teleop_node = Node(
        package='openarmx_head_teleop_vr_pico',
        executable='head_teleop_node',
        name='openarmx_head_teleop_vr_pico_node',
        output='screen',
        parameters=[
            LaunchConfiguration('vr_config_file'),
            {
                'enable_soft_limits': ParameterValue(
                    LaunchConfiguration('head_enable_soft_limits'), value_type=bool),
                'yaw_soft_margin_deg': ParameterValue(
                    LaunchConfiguration('head_yaw_soft_margin_deg'), value_type=float),
                'pitch_soft_margin_deg': ParameterValue(
                    LaunchConfiguration('head_pitch_soft_margin_deg'), value_type=float),
                'soft_limit_blend_deg': ParameterValue(
                    LaunchConfiguration('head_soft_limit_blend_deg'), value_type=float),
                'startup_home_enabled': ParameterValue(
                    LaunchConfiguration('head_startup_home_enabled'), value_type=bool),
                'startup_home_step_deg': ParameterValue(
                    LaunchConfiguration('head_startup_home_step_deg'), value_type=float),
            },
        ],
        condition=IfCondition(LaunchConfiguration('enable_head_teleop'))
    )

    return LaunchDescription([
        # 参数声明
        declare_vr_config_file,
        declare_max_linear_speed,
        declare_max_angular_speed,
        declare_boost_linear_speed,
        declare_joystick_deadzone,
        declare_linear_expo,
        declare_angular_expo,
        declare_estop_toggle_topic,
        declare_enable_waist_control,
        declare_vr_chassis,
        declare_enable_joystick_control,
        declare_enable_button_lift,
        declare_enable_head_teleop,
        declare_arm_teleop_config_file,
        declare_robot_type,
        declare_left_axis_matrix,
        declare_right_axis_matrix,
        declare_left_orientation_matrix,
        declare_right_orientation_matrix,
        declare_head_startup_home_enabled,
        declare_head_startup_home_step_deg,
        declare_head_enable_soft_limits,
        declare_head_yaw_soft_margin_deg,
        declare_head_pitch_soft_margin_deg,
        declare_head_soft_limit_blend_deg,
        declare_waist_position_gain,
        declare_waist_lift_gain,
        declare_waist_angular_gain,
        declare_waist_max_linear_vel,
        declare_waist_lift_speed,
        declare_enable_scurve,
        declare_acceleration_time,
        declare_smoothness,

        # 节点启动
        pico_pose_bridge_node,
        vr_teleop_node,
        vr_lift_control_node,
        waist_chassis_control_node,
        arm_teleop_node,
        head_teleop_node,
    ])
