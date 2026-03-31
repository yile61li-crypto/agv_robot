# LeBot - Dual Robot Autonomous Patrol Simulation System

> LeBot 双机器人自主巡逻仿真系统

基于 **ROS 2 Humble + Gazebo Classic** 的四轮差速机器人仿真平台，支持双机器人协同（导航巡逻 + 跟随），集成 SLAM 建图、Nav2 自主导航、中文语音播报等功能。

---

## 1. 系统概览

| 项目       | 说明                                                  |
| ---------- | ----------------------------------------------------- |
| **平台**     | Ubuntu 22.04 + ROS 2 Humble                          |
| **仿真器**   | Gazebo Classic 11                                     |
| **机器人**   | 四轮差速驱动, 双激光雷达 (前左 + 后右), IMU            |
| **导航**     | Nav2 (AMCL 定位 + NavFn 全局规划 + DWB 局部规划)      |
| **建图**     | SLAM Toolbox (异步模式)                                |
| **传感器融合** | robot_localization (EKF: 轮式里程计 + IMU)            |
| **协同**     | 主机器人 Nav2 巡逻 + 跟随机器人 Gazebo 姿态跟踪       |

### 系统架构

```
+------------------------------------------------------------------+
|                       Gazebo Simulation                          |
|                                                                  |
|   +-----------------+                  +-----------------+       |
|   |   main_robot    |                  | follower_robot  |       |
|   |   (Nav2 Nav)    | <--- follow ---  |  (Pose Control) |       |
|   +--------+--------+                  +--------+--------+       |
|            |                                    |                |
|     Dual LiDAR + IMU                     Dual LiDAR + IMU       |
+------------|------------------------------------|-----------------+
             |                                    |
      +------v------+                      +------v------+
      | laser_merger |                      | laser_merger |
      |  360 degree  |                      |  360 degree  |
      +------+------+                      +-------------+
             |
      +------v------+
      |   Nav2 Stack |
      | AMCL+Planner |
      | +Controller  |
      +------+------+
             |
      +------v------+
      | patrol_node  |
      | + speaker    |
      +-------------+
```

---

## 2. 功能包说明

### 2.1 `lebot_description` — 机器人模型与仿真环境

机器人 URDF 模型定义、Gazebo 传感器插件、控制器配置、EKF 配置。

```
lebot_description/
├── config/
│   ├── ekf_config.yaml            # EKF 传感器融合参数（轮式里程计 + IMU）
│   ├── lebot.rviz                 # 模型查看 RViz 配置
│   └── lebot_ros2_controller.yaml # ros2_control 差速控制器参数
├── launch/
│   ├── gazebo_sim.launch.py       # 核心：Gazebo 仿真 + 机器人生成 + 控制器加载
│   ├── view_model.launch.py       # 独立查看 URDF 模型
│   └── runtime_rviz.launch.py     # 运行时 RViz 可视化
├── urdf/lebot/
│   ├── lebot.urdf.xacro           # 顶层模型入口
│   ├── base.urdf.xacro            # 底盘
│   ├── wheel.urdf.xacro           # 轮子宏
│   ├── laser.urdf.xacro           # 激光雷达安装
│   ├── imu.xacro                  # IMU 传感器
│   ├── gazebo_sensor_plugin.xacro # Gazebo 传感器插件
│   ├── lebot.ros2_control.xacro   # ros2_control 硬件接口
│   └── common_inertia.xacro       # 惯性参数宏
└── world/
    ├── custom_room.world           # 仿真世界文件
    └── room/                       # 房间 Gazebo 模型
```

**机器人物理参数:**

| 参数       | 值                      |
| ---------- | ----------------------- |
| 车身尺寸   | 0.30 m x 0.20 m        |
| 轮距       | 0.20 m                  |
| 轮半径     | 0.05 m                  |
| 驱动方式   | 四轮差速 (左右各 2 轮)   |
| 最大线速度 | 0.5 m/s                 |
| 最大角速度 | 1.8 rad/s               |

### 2.2 `laser_merger` — 360° 激光数据融合

将前左、后右两个激光雷达的扫描数据通过 TF 变换合并为一个 360° `LaserScan` 消息。

```
laser_merger/
└── laser_merger/
    └── laser_merger_node.py    # 激光融合节点
```

**功能特点:**

- 基于 `message_filters.ApproximateTimeSynchronizer` 时间同步
- 通过 TF 将各雷达数据变换到 `base_link` 坐标系
- 内置车身自遮挡过滤, 消除安装位置导致的假障碍
- 可配置输出角度范围、频率、车身尺寸等参数

### 2.3 `lebot_navigation2` — 导航与建图

Nav2 导航栈配置、SLAM 建图、双机器人编排。

```
lebot_navigation2/
├── config/
│   ├── nav2_params.yaml           # Nav2 完整参数（AMCL/Planner/Controller/Costmap）
│   ├── slam_params.yaml           # SLAM Toolbox 参数
│   ├── nav2.rviz                  # 导航 RViz 配置
│   └── slam.rviz                  # 建图 RViz 配置
├── launch/
│   ├── main_robot_nav_follow.launch.py  # 主入口：双机器人 + 导航 + 跟随
│   ├── dual_robot_bringup.launch.py     # 双机器人 Gazebo 生成编排
│   ├── navigation2.launch.py            # Nav2 导航栈（定位 + 导航）
│   ├── runtime_localization.launch.py   # AMCL + map_server 生命周期管理
│   ├── runtime_navigation.launch.py     # Nav2 控制/规划/行为 生命周期管理
│   └── mapping.launch.py               # SLAM 建图（Gazebo + SLAM + 键盘遥控 + RViz）
└── maps/
    ├── room5.pgm / room5.yaml     # 当前默认地图
    └── ...                         # 其他版本地图存档
```

### 2.4 `autopatrol_robot` — 自主巡逻与跟随控制

巡逻逻辑、语音播报、跟随控制器、领航者姿态发布。

```
autopatrol_robot/
├── autopatrol_robot/
│   ├── patrol_node.py             # 多航点巡逻节点
│   ├── speaker.py                 # 中文语音播报服务节点（espeak）
│   ├── follower_controller.py     # 跟随控制器（基于 Gazebo 姿态）
│   └── entity_pose_publisher.py   # Gazebo 实体姿态发布器
├── config/
│   └── patrol_config.yaml         # 巡逻航点配置
└── launch/
    └── autopatrol.launch.py       # 独立巡逻启动文件
```

### 2.5 `autopatrol_interfaces` — 自定义服务接口

```
autopatrol_interfaces/
└── srv/
    └── SpeechText.srv    # 语音合成服务：string text → bool result
```

---

## 3. 快速开始

### 3.1 环境依赖

```bash
# ROS 2 Humble 基础包
sudo apt install ros-humble-desktop ros-humble-gazebo-ros-pkgs

# 导航与建图
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup \
                 ros-humble-slam-toolbox ros-humble-robot-localization

# ros2_control
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers \
                 ros-humble-gazebo-ros2-control

# 语音合成
pip3 install espeakng
sudo apt install espeak-ng

# TF 工具
pip3 install transforms3d
sudo apt install ros-humble-tf-transformations
```

### 3.2 编译

```bash
cd ~/lebot_ws
colcon build
source install/setup.bash
```

### 3.3 运行

#### 双机器人导航 + 跟随（主入口）

```bash
ros2 launch lebot_navigation2 main_robot_nav_follow.launch.py
```

启动后：
1. Gazebo 加载仿真世界，生成主机器人和跟随机器人
2. Nav2 导航栈激活（AMCL 定位 + 全局/局部规划）
3. RViz 打开导航可视化界面
4. 跟随控制器自动保持跟随机器人在主机器人身后 1m

在 RViz 中使用 "2D Goal Pose" 发送导航目标，主机器人导航，跟随机器人自动跟随。

#### SLAM 建图

```bash
ros2 launch lebot_navigation2 mapping.launch.py
```

使用键盘遥控机器人建图，完成后保存地图：

```bash
ros2 run nav2_map_server map_saver_cli -f ~/lebot_ws/src/lebot_navigation2/maps/my_map
```

#### 自主巡逻

```bash
# 先启动导航环境，再启动巡逻
ros2 launch autopatrol_robot autopatrol.launch.py
```

---

## 4. 核心节点说明

### 4.1 `laser_merger_node`

| 项目    | 值                                                                           |
| ------- | ---------------------------------------------------------------------------- |
| 订阅    | `/<ns>/scan_front_left`, `/<ns>/scan_back_right` (sensor_msgs/LaserScan)     |
| 发布    | `/<ns>/scan` (sensor_msgs/LaserScan)                                         |
| 依赖 TF | `<ns>/laser_front_left` -> `<ns>/base_link`, `<ns>/laser_back_right` -> `<ns>/base_link` |

### 4.2 `patrol_node`

| 项目     | 值                                                                        |
| -------- | ------------------------------------------------------------------------- |
| 基类     | `nav2_simple_commander.BasicNavigator`                                    |
| 服务调用 | `speech_text` (autopatrol_interfaces/SpeechText)                          |
| 参数     | `initial_point` [x, y, yaw], `target_points` [x1, y1, yaw1, x2, y2, yaw2, ...] |

### 4.3 `speaker`

| 项目 | 值                                               |
| ---- | ------------------------------------------------ |
| 服务 | `speech_text` (autopatrol_interfaces/SpeechText) |
| 引擎 | espeak-ng, 中文语音                               |

### 4.4 `follower_controller`

| 项目     | 值                                                              |
| -------- | --------------------------------------------------------------- |
| 订阅     | `/<main_ns>/follow_target_pose` (geometry_msgs/PoseStamped)     |
| 发布     | `/<follower_ns>/cmd_vel` (geometry_msgs/Twist)                  |
| 服务调用 | `/gazebo_state/get_entity_state` (gazebo_msgs/GetEntityState)   |
| 控制策略 | 比例控制 + EMA 低通平滑, 保持在领航者身后指定距离                  |

### 4.5 `entity_pose_publisher`

| 项目     | 值                                                            |
| -------- | ------------------------------------------------------------- |
| 发布     | `/<main_ns>/follow_target_pose` (geometry_msgs/PoseStamped)   |
| 服务调用 | `/gazebo_state/get_entity_state` (gazebo_msgs/GetEntityState) |
| 功能     | 以 15Hz 从 Gazebo 查询主机器人世界坐标姿态并发布                 |

---

## 5. 启动文件参数速查

### `main_robot_nav_follow.launch.py` (主入口)

| 参数                              | 默认值         | 说明                  |
| --------------------------------- | -------------- | --------------------- |
| `navigation_start_delay`          | 6.0            | Nav2 启动延迟 (秒)    |
| `main_robot_namespace`            | main_robot     | 主机器人命名空间       |
| `follower_robot_namespace`        | follower_robot | 跟随机器人命名空间     |
| `follower_start_delay`            | 5.0            | 跟随机器人生成延迟 (秒) |
| `follower_controller_start_delay` | 8.0            | 跟随控制器启动延迟 (秒) |
| `follow_distance`                 | 1.0            | 跟随距离 (米)          |
| `follower_linear_gain`            | 0.8            | 线速度比例增益         |
| `follower_angular_gain`           | 2.0            | 角速度比例增益         |
| `use_rviz`                        | true           | 是否启动 RViz          |

---

## 6. 项目目录总览

```
lebot_ws/
├── docs/                          # 开发文档
├── src/
│   ├── autopatrol_interfaces/     # 自定义 ROS 2 服务接口
│   ├── autopatrol_robot/          # 巡逻、语音、跟随控制节点
│   ├── laser_merger/              # 激光雷达数据融合
│   ├── lebot_description/         # 机器人模型、仿真环境、控制器配置
│   └── lebot_navigation2/         # Nav2 导航、SLAM 建图、双机器人编排
├── .gitignore
└── README.md                      # 本文档
```

---

## 7. 许可证

Apache-2.0
