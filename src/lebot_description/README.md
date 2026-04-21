# lebot_description

LeBot 机器人模型定义、Gazebo 仿真配置、控制器参数。

> **功能包定位**：这是整个项目的基础层，负责定义机器人的物理外观、传感器配置和仿真环境。所有其他功能包都依赖于本包提供的模型和配置。

----

## 目录

- [功能概述](#功能概述)
- [启动文件详解](#启动文件详解)
- [使用示例](#使用示例)
- [配置文件详解](#配置文件详解)
- [机器人物理参数](#机器人物理参数)
- [URDF/Xacro 文件结构](#urdfxacro-文件结构)
- [依赖说明](#依赖说明)
- [常见问题](#常见问题)
- [许可证](#许可证)

---

## 功能概述

本功能包提供四大核心功能：

### 1. URDF/Xacro 机器人模型

使用 Xacro（XML 宏）编写可复用的机器人描述文件：
- **底盘（base）**：定义机器人的主体结构和外观
- **轮子（wheels）**：4 个轮子的几何形状、关节类型（continuous）和摩擦参数
- **激光雷达（LiDAR）**：2 个激光雷达的安装位置和扫描参数
- **IMU**：惯性测量单元的安装位置和噪声特性

### 2. Gazebo 仿真配置

- **世界文件**：预置的房间环境，包含墙壁、障碍物和家具
- **传感器插件**：激光雷达和 IMU 的 Gazebo 仿真插件
- **物理参数**：摩擦系数、质量、惯性张量等

### 3. ros2_control 配置

- **硬件接口**：定义关节的命令接口（velocity）和状态接口（position, velocity）
- **控制器**：差速驱动控制器（diff_drive_controller）和关节状态广播器
- **Gazebo 集成**：通过 gazebo_ros2_control 插件实现仿真控制

### 4. EKF 传感器融合配置

- **输入源**：轮式里程计（odom）和 IMU（imu/data）
- **输出**：融合后的里程计（odom → base_footprint）
- **融合策略**：2D 平面模式，重点优化航向角（Yaw）估计

## 启动文件详解

### gazebo_sim.launch.py（核心启动文件）

这是最重要的启动文件，执行完整的机器人仿真流程：

```bash
ros2 launch lebot_description gazebo_sim.launch.py
```

**启动流程（按时间顺序）：**

| 时间 | 动作 | 说明 |
|------|------|------|
| 0s | Gazebo 服务器启动 | 加载 world 文件，初始化物理引擎 |
| 0s | Gazebo 客户端启动 | 打开 GUI 窗口 |
| 0s | robot_state_publisher | 解析 URDF，发布静态 TF 变换树 |
| 1s | spawn_entity.py | 将机器人模型插入 Gazebo 世界 |
| 2s | joint_state_broadcaster | 启动关节状态广播器（读取轮子角度） |
| 3s | diff_drive_controller | 启动差速驱动控制器（接收 cmd_vel） |
| 4s | laser_merger_node | 启动激光融合节点（合并双雷达） |
| 4s | ekf_node | 启动 EKF 融合节点（融合 odom + IMU） |

**常用参数：**

| 参数 | 默认值 | 说明 | 示例 |
|------|--------|------|------|
| `robot_namespace` | `robot` | 机器人命名前缀 | `robot_namespace:=bot1` |
| `x_pose` | 0.0 | 初始 X 坐标（米） | `x_pose:=-1.5` |
| `y_pose` | 0.0 | 初始 Y 坐标（米） | `y_pose:=2.0` |
| `yaw` | 0.0 | 初始朝向（弧度） | `yaw:=1.57`（90°） |
| `world` | `custom_room.world` | 世界文件路径 | `world:=empty.world` |
| `launch_gazebo` | true | 是否启动 Gazebo | `launch_gazebo:=false` |

### view_model.launch.py（模型查看）

无需启动 Gazebo，快速验证 URDF 模型：

```bash
ros2 launch lebot_description view_model.launch.py
```

**功能：**
- 加载 URDF 到 RViz 显示
- 启动 joint_state_publisher_gui，可用滑块手动控制轮子旋转
- 验证 TF 树结构是否正确

### rviz.launch.py（运行时可视化）

在仿真已运行时，单独启动 RViz 查看机器人状态：

```bash
ros2 launch lebot_description rviz.launch.py robot_namespace:=robot
```

**适用场景：**
- 仿真在其他终端运行，需要新开窗口监控
- 需要查看激光雷达点云、里程计轨迹等可视化数据

## 使用示例

### 示例 1：单机器人基础仿真

```bash
# 终端 1：启动仿真
ros2 launch lebot_description gazebo_sim.launch.py

# 终端 2：手动发送速度指令（让机器人前进）
ros2 topic pub /robot/cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.2}, angular: {z: 0.0}}' --rate 10

# 停止发送指令后，机器人会自动停止
```

### 示例 2：自定义初始位置

```bash
# 将机器人放在房间左上角，朝向房间中心
ros2 launch lebot_description gazebo_sim.launch.py \
  x_pose:=-4.0 \
  y_pose:=3.0 \
  yaw:=-0.785
```

### 示例 3：多机器人仿真（手动指定不同命名空间）

```bash
# 终端 1：启动主机器人
ros2 launch lebot_description gazebo_sim.launch.py \
  robot_namespace:=main_robot \
  x_pose:=0.0 y_pose:=0.0

# 终端 2：启动跟随机器人（不启动 Gazebo）
ros2 launch lebot_description gazebo_sim.launch.py \
  robot_namespace:=follower_robot \
  x_pose:=-1.0 y_pose:=0.0 \
  launch_gazebo:=false
```

> ⚠️ **注意**：手动启动多机器人时，需确保命名空间不同，且只有第一个机器人启动 Gazebo。

## 配置文件详解

### ros2_control.yaml

差速驱动控制器的核心参数：

```yaml
lebot_diff_drive_controller:
  ros__parameters:
    left_wheel_names: ["left_front_wheel_joint", "left_back_wheel_joint"]
    right_wheel_names: ["right_front_wheel_joint", "right_back_wheel_joint"]
    wheel_separation: 0.20        # 轮距（米），影响转弯半径
    wheel_radius: 0.05            # 轮半径（米），影响里程计算
    linear:
      x:
        has_velocity_limits: true
        max_velocity: 0.5         # 最大线速度（m/s）
    angular:
      z:
        has_velocity_limits: true
        max_velocity: 1.8         # 最大角速度（rad/s）
    publish_rate: 50.0            # 发布频率（Hz）
    cmd_vel_timeout: 0.5          # 指令超时时间（秒），超时后自动停止
```

### ekf_config.yaml

EKF 滤波器的融合策略配置：

```yaml
# 融合来源 1：轮式里程计（高频，短期准确，长期漂移）
odom0: /odom
odom0_config: [true, true, false,   # x, y, z 位置
               false, false, false,  # roll, pitch, yaw 角度
               true, true, false,    # x, y, z 线速度
               false, false, true,   # roll, pitch, yaw 角速度
               false, false, false]  # x, y, z 线加速度

# 融合来源 2：IMU（低频，长期稳定，短期噪声大）
imu0: /imu/data
imu0_config: [false, false, false,  # 不使用 IMU 位置
              false, false, false,  # 不使用 IMU 姿态（无磁力计）
              false, false, false,  # 不使用 IMU 线速度
              false, false, true,   # 仅使用 IMU 的 yaw 角速度
              false, false, false]

# 关键参数：增大航向角过程噪声，防止转向滞后
process_noise_covariance: [... 0.08, ...]  # yaw 噪声
```

**设计思路：**
- 位置（x, y）：主要依赖里程计（高频更新）
- 航向角（yaw）：融合里程计 yaw 角速度和 IMU yaw 角速度
- 轮式里程计提供短期精度，IMU 提供长期稳定性

### lebot.rviz

RViz 配置文件，预设了以下显示：
- 机器人模型（RobotModel）
- TF 坐标系树
- 激光雷达扫描（LaserScan）
- 里程计轨迹（Odometry）

## 机器人物理参数

| 参数 | 值 | 说明 |
|------|------|------|
| 车身尺寸 | 0.30 × 0.20 × 0.10 m | 长×宽×高，白色半透明外观 |
| 轮半径 | 0.05 m | 影响里程计算精度和爬坡能力 |
| 轮距 | 0.20 m | 左右轮中心距，决定最小转弯半径（约 0.1m） |
| 轮厚 | 0.04 m | 轮子宽度，影响地面接触面积 |
| 底盘质量 | 3.0 kg | 主体质量，影响惯性 |
| 单轮质量 | 0.3 kg | 4 个轮子总质量 1.2 kg |
| 最大线速度 | 0.5 m/s | 约 1.8 km/h，室内安全速度 |
| 最大角速度 | 1.8 rad/s | 约 103°/s，原地旋转一周约 3.5 秒 |

**差速运动学公式：**
```
# 给定目标线速度 v（m/s）和角速度 w（rad/s）
left_speed  = (v - w * wheel_separation / 2) / wheel_radius
right_speed = (v + w * wheel_separation / 2) / wheel_radius
```

## URDF/Xacro 文件结构

```
lebot.urdf.xacro              # 顶层入口：组合所有模块
├── base.urdf.xacro           # 底盘定义：base_footprint + base_link
├── wheel.urdf.xacro          # 轮子宏：可实例化为 4 个轮子
├── laser.urdf.xacro          # 激光雷达宏：实例化为 2 个雷达
├── imu.xacro                 # IMU 定义
├── lebot.ros2_control.xacro  # ros2_control 硬件接口定义
└── common_inertia.xacro      # 惯性计算工具宏
```

### 文件详解

**lebot.urdf.xacro**
- 顶层文件，被 launch 文件调用
- 包含所有子 xacro 文件
- 实例化 4 个轮子（左前、右前、左后、右后）
- 实例化 2 个激光雷达（前左、后右）
- 实例化 1 个 IMU

**base.urdf.xacro**
- `base_footprint`：地面投影点，用于导航定位
- `base_link`：底盘中心，所有部件的挂载点
- 底盘为白色半透明方盒，尺寸 0.30×0.20×0.10 m

**wheel.urdf.xacro**
- 圆柱体几何形状（半径 0.05 m，厚度 0.04 m）
- continuous 关节类型（可无限旋转）
- Gazebo 摩擦系数：mu1=1.0（纵向），mu2=0.18（横向）

**laser.urdf.xacro**
- 小圆柱体（半径 0.02 m，高度 0.02 m）
- fixed 关节（相对底盘静止）
- 对角线安装：前左 45°，后右 -135°

**lebot.ros2_control.xacro**
- 定义 4 个轮子关节的硬件接口
- 命令接口：velocity（速度控制）
- 状态接口：position, velocity（角度和角速度反馈）
- 加载 Gazebo ros2_control 插件

## 依赖说明

| 依赖包 | 用途 | 是否必需 |
|--------|------|----------|
| `robot_state_publisher` | 解析 URDF，发布 TF 变换树 | 必需 |
| `gazebo_ros` | Gazebo 与 ROS 2 的桥接 | 必需（仿真时） |
| `ros2_control` | 硬件接口框架 | 必需 |
| `ros2_controllers` | 差速控制器、关节状态广播器 | 必需 |
| `gazebo_ros2_control` | Gazebo 中的 ros2_control 插件 | 必需（仿真时） |
| `xacro` | URDF 宏处理 | 必需 |
| `laser_merger` | 激光雷达数据融合（本项目的其他包） | 必需 |
| `robot_localization` | EKF 传感器融合（本项目的其他包） | 必需 |

## 常见问题

### Q: Gazebo 中机器人不出现
**原因排查：**
1. 检查 `spawn_entity.py` 是否在 launch 文件中被正确调用
2. 查看 Gazebo 的实体列表：`ros2 service call /gazebo/get_entity_state gazebo_msgs/srv/GetEntityState '{name: "robot"}'`
3. 检查 URDF 路径是否正确

### Q: 机器人无法移动（cmd_vel 无响应）
**原因排查：**
1. 确认 `diff_drive_controller` 已加载：`ros2 control list_controllers`
2. 检查控制器状态是否为 `active`
3. 确认 `cmd_vel` 话题名称正确（带命名空间前缀）

### Q: 激光雷达数据异常
**原因排查：**
1. 确认 `laser_merger_node` 已启动
2. 检查 TF 变换是否正常：`ros2 run tf2_tools view_frames`
3. 查看原始雷达数据：`ros2 topic echo /robot/scan_front_left`

## 许可证

Apache-2.0
