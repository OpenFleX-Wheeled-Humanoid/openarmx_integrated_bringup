# openarmx_integrated_bringup

[English](./README.md) | 中文

---

![封面](./image/cover.gif)

OpenFlex 集成机器人系统的启动文件和配置。

## 概述

本包提供 OpenArmX 集成机器人的主启动文件和控制器配置。该系统将四轮四转向（4W4S）全向底盘、线性升降模块、双 7 自由度+夹爪手臂和 2 自由度头部整合为单一 ros2_control 系统。

## 系统架构

```
base_link（底盘）
  -> lift_base_link -> lift_carriage_link
       -> 左臂（8 个关节）
       -> 右臂（8 个关节）
       -> 头部（2 个关节：俯仰 + 偏航）
```

## 控制器

| 控制器 | 类型 | 关节 |
|--------|------|------|
| `joint_state_broadcaster` | JointStateBroadcaster | 所有关节 |
| `swerve_drive_controller` | SwerveDriveController | 4 转向 + 4 驱动关节 |
| `left_forward_position_controller` | JointGroupPositionController | 左臂 8 关节 |
| `right_forward_position_controller` | JointGroupPositionController | 右臂 8 关节 |
| `head_forward_position_controller` | ForwardCommandController | 头部偏航 + 俯仰 |
| `velocity_controller` | ParamForwardingVelocityController | lift_joint（速度） |
| `lift_position_controller` | ParamForwardingPositionController | lift_joint（位置） |
| `lift_manual_position_controller` | LiftSlideManualPositionController | lift_joint（手动） |
| `lift_state_controller` | LiftSlideStateController | lift_joint（状态） |
| `left/right_forward_effort_controller` | ForwardCommandController | 手臂力矩（重力补偿） |


### `integrated_robot_bringup.launch.py`

启动完整系统：
- `robot_state_publisher` 加载集成 URDF（xacro）
- `ros2_control_node` 加载所有硬件接口
- 控制器按时序错峰启动
- 可选 RViz 可视化
- 可选重力补偿
- 可选升降台自动回零

### `integrated_vr_teleop.launch.py`

启动 VR 遥操作栈（Pico 姿态桥接 + 手臂遥操 + 底盘/升降/头部控制）。

## 使用方法

```bash
# 编译

---
cd ~/openflex_all/openflex_ws
colcon build --packages-select openarmx_integrated_bringup

# 真实硬件

---
ros2 launch openarmx_integrated_bringup integrated_robot_bringup.launch.py

# 仿真硬件

---
ros2 launch openarmx_integrated_bringup integrated_robot_bringup.launch.py use_fake_hardware:=true

# 自定义 CAN 分配

---
ros2 launch openarmx_integrated_bringup integrated_robot_bringup.launch.py \
  chassis_steering_can:=can5 chassis_driving_can:=can4 \
  left_arm_can:=can1 right_arm_can:=can0 \
  lift_can:=can3 head_can:=can2

# 带自动回零

---
ros2 launch openarmx_integrated_bringup integrated_robot_bringup.launch.py auto_homing:=true
```

## 启动文件

### `integrated_robot_o6_bringup.launch.py`

启动带两只 LinkerHand O6 的轮臂整机。默认使用机械臂与 O6 共总线：右臂/右手使用 `can0`，左臂/左手使用 `can1`。
如需将 O6 接到独立 CAN 接口，可显式切换为独立模式；默认共总线配置不会改变：

```bash
# 默认：共总线（右手 0x27，左手 0x28）
ros2 launch openarmx_integrated_bringup integrated_robot_o6_bringup.launch.py

# 仿真
ros2 launch openarmx_integrated_bringup integrated_robot_o6_bringup.launch.py \
  use_fake_hardware:=true \
  use_rviz:=true

# 独立 CAN：右手 can6，左手 can7
ros2 launch openarmx_integrated_bringup integrated_robot_o6_bringup.launch.py \
  o6_hardware_mode:=independent \
  right_o6_can_interface:=can6 \
  left_o6_can_interface:=can7
```

独立模式启动两个 `hands_hardware/o6_driver_node`，并通过 O6 状态映射器和状态合并器接入整机显示；不会加载共总线 O6 ros2_control 插件。启动前必须确认 `can6`、`can7` 已存在并配置为经典 CAN 1 Mbps。

## 关键参数

| 参数 | 默认值 | 描述 |
|------|--------|------|
| `use_fake_hardware` | `false` | 仿真硬件模式 |
| `use_rviz` | `true` | 启动 RViz |
| `control_mode` | `mit` | 电机控制模式（mit/csp） |
| `enable_head` | `true` | 启用头部硬件和控制器 |
| `enable_forward_effort` | `true` | 启用重力补偿 |
| `auto_homing` | `false` | 启动时自动升降台回零 |

## 发布话题（通过控制器）

| 话题 | 类型 | 描述 |
|------|------|------|
| `/joint_states` | `sensor_msgs/JointState` | 所有关节状态，100 Hz |
| `/cmd_vel` | `geometry_msgs/Twist` | 底盘速度输入 |
| `/tf` | TF2 | `odom -> base_link`（全向控制器发布） |

## 依赖

- `openarmx_integrated_description`
- `swerve_bringup` / `swerve_controller` / `swerve_hardware`
- `openarmx_hardware`
- `lift_slide_driver` / `lift_slide_bringup`
- `openarmx_gravity_comp`
- `controller_manager`、`forward_command_controller`、`joint_state_broadcaster`

## 许可证

本作品采用知识共享 署名-非商业性使用-相同方式共享 4.0 国际许可协议 (CC BY-NC-SA 4.0) 进行许可。

版权所有 (c) 2026 成都长数机器人有限公司 (Chengdu Changshu Robot Co., Ltd.)

详情请参阅 [LICENSE_CN.md](LICENSE) 文件或访问：http://creativecommons.org/licenses/by-nc-sa/4.0/

## 致谢

本包是 OpenArmX 机器人平台生态系统的一部分，专为协作机器人领域的研究和工业应用而开发。

---

## 📞 联系我们

### 成都长数机器人有限公司
**Chengdu Changshu Robotics Co., Ltd.**

| 联系方式 | 信息 |
|---------|------|
| 📧 邮箱 | openarmrobot@gmail.com |
| 📱 电话/微信 | +86-17746530375 |
| 🌐 官网 | <https://openarmx.com/> |
| 🌐 文档 | <http://docs.openarmx.com/> |
| 📍 地址 | 天津经济技术开发区西区新业八街11号华诚机械厂 |
| 👤 联系人 | 王先生 |
