# LeBot — 双机器人自主巡逻仿真系统

基于 **ROS 2 Humble + Gazebo Classic 11** 的四轮差速机器人仿真平台。支持双机器人协同（主机器人 Nav2 导航巡逻 + 跟随机器人姿态跟踪），集成 SLAM 建图、自主导航、360° 激光融合、EKF 传感器融合、中文语音播报等功能。

---

## 1. 系统概览

| 项目 | 说明 |
|------|------|
| **平台** | Ubuntu 22.04 + ROS 2 Humble |
| **仿真器** | Gazebo Classic 11 |
| **机器人** | 四轮差速驱动，双激光雷达（前左 + 后右），IMU |
| **导航** | Nav2（AMCL 定位 + NavFn 全局规划 + DWB 局部规划） |
| **建图** | SLAM Toolbox（异步模式） |
| **传感器融合** | robot_localization（EKF：轮式里程计 + IMU） |
| **协同** | 主机器人 Nav2 巡逻 + 跟随机器人 Gazebo 姿态跟踪 |

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
|     Dual LiDAR + IMU                     Dual LiDAR + IMU        |
+------------|------------------------------------|----------------+
             |                                    |
      +------v------+                      +------v------+
      | laser_merger|                      | laser_merger|
      |  360 degree |                      |  360 degree |
      +------+------+                      +-------------+
             |                                    |
      +------v------+                      +------v------+
      |  Nav2 Stack |                      |   EKF +      |
      | AMCL+Planner|                      |   cmd_vel    |
      | +Controller |                      +-------------+
      +------+------+
             |
      +------v------+
      | patrol_node |
      | + speaker   |
      +-------------+
```

---

## 2. 功能包说明

### 2.1 `lebot_description` — 机器人模型与仿真环境

负责机器人 URDF 模型定义、Gazebo 传感器插件配置、ros2_control 差速控制器参数、EKF 传感器融合参数，以及仿真世界文件。

#### 目录结构

```
lebot_description/
├── CMakeLists.txt
├── LICENSE
├── package.xml
├── config/
│   ├── ekf_config.yaml             # EKF 传感器融合参数（轮式里程计 + IMU）
│   ├── lebot.rviz                  # 模型查看 RViz 配置
│   └── lebot_ros2_controller.yaml  # ros2_control 差速控制器参数
├── launch/
│   ├── gazebo_sim.launch.py        # 核心：Gazebo 仿真 + 机器人生成 + 控制器加载
│   ├── view_model.launch.py        # 独立查看 URDF 模型（无需 Gazebo）
│   └── runtime_rviz.launch.py      # 运行时 RViz 可视化
├── urdf/lebot/
│   ├── lebot.urdf.xacro            # 顶层模型入口（组合所有子模块）
│   ├── base.urdf.xacro             # 底盘 link + base_footprint 关节
│   ├── wheel.urdf.xacro            # 轮子宏（可复用，实例化 4 个轮子）
│   ├── laser.urdf.xacro            # 激光雷达安装宏
│   ├── imu.xacro                   # IMU 传感器 + Gazebo 插件
│   ├── gazebo_sensor_plugin.xacro  # Gazebo 激光雷达插件宏
│   ├── lebot.ros2_control.xacro    # ros2_control 硬件接口定义
│   └── common_inertia.xacro        # 惯性参数计算宏（box/cylinder/sphere）
└── world/
    ├── custom_room.world            # 仿真世界文件（房间 + 家具 + 障碍物）
    └── room/                        # 房间 Gazebo 模型（model.config + model.sdf）
```

#### 各 URDF/Xacro 文件职责

| 文件 | 职责 |
|------|------|
| `lebot.urdf.xacro` | 顶层入口，include 所有子 xacro，调用宏实例化底盘、4 个轮子、2 个雷达、IMU、ros2_control、Gazebo 插件 |
| `base.urdf.xacro` | 定义 `base_footprint`（地面参考点）和 `base_link`（底盘本体），通过 `base_footprint_joint` 连接。底盘为 0.30×0.20×0.10 m 的白色半透明方盒，质量 3.0 kg |
| `wheel.urdf.xacro` | 轮子宏，每个轮子是 cylinder（半径 0.05 m，厚度 0.04 m），通过 `continuous` 关节挂在 `base_link` 上，Gazebo 中设摩擦系数 mu1=1.0, mu2=0.18 |
| `laser.urdf.xacro` | 雷达宏，每个雷达是小圆柱（半径 0.02 m，高度 0.02 m），通过 `fixed` 关节挂到 `base_link`，对角线安装（前左 45°、后右 -135°） |
| `imu.xacro` | IMU 传感器宏，扁平小盒子挂载在底盘顶部中心，内含 `libgazebo_ros_imu_sensor.so` 插件，100Hz 更新，高斯噪声模拟 |
| `gazebo_sensor_plugin.xacro` | 激光雷达 Gazebo 插件宏，使用 `libgazebo_ros_ray_sensor.so`，360 采样点/圈，范围 0.1~10 m，20Hz 更新，高斯噪声 stddev=0.005 |
| `lebot.ros2_control.xacro` | ros2_control 硬件接口，定义 4 个轮子关节的 velocity 命令接口和 position/velocity 状态接口，加载 `libgazebo_ros2_control.so` 插件 |
| `common_inertia.xacro` | 惯性计算宏：`box_inertia`、`cylinder_inertia`、`sphere_inertia`，根据质量/尺寸自动计算惯性张量 |

#### 各配置文件职责

| 文件 | 职责 |
|------|------|
| `ekf_config.yaml` | EKF 节点参数：融合轮式里程计（位置+线速度+角速度Z）和 IMU（角速度Z），2D 模式，Yaw 过程噪声调大至 0.08 防止转向滞后 |
| `lebot_ros2_controller.yaml` | 差速控制器参数：轮距 0.20 m，轮半径 0.05 m，最大线速度 0.5 m/s，最大角速度 1.8 rad/s，50Hz 发布，cmd_vel 超时 0.5 s |

#### 机器人物理参数

| 参数 | 值 |
|------|------|
| 车身尺寸 | 0.30 m × 0.20 m × 0.10 m |
| 轮距（左右轮中心距） | 0.20 m |
| 轮半径 | 0.05 m |
| 轮厚度 | 0.04 m |
| 驱动方式 | 四轮差速（左右各 2 轮，同侧同速） |
| 底盘质量 | 3.0 kg |
| 单轮质量 | 0.3 kg |
| 最大线速度 | 0.5 m/s |
| 最大角速度 | 1.8 rad/s |

#### 雷达安装位置

| 雷达 | 位置 (x, y, z) | 朝向 (yaw) | 话题 |
|------|----------------|------------|------|
| 前左 `front_left` | (0.140, 0.090, 0.048) | 0.785 rad (45°) | `/<ns>/scan_front_left` |
| 后右 `back_right` | (-0.140, -0.090, 0.048) | -2.356 rad (-135°) | `/<ns>/scan_back_right` |

两雷达对角线安装，各覆盖约 180°，合并后实现 360° 全向感知。

---

### 2.2 `laser_merger` — 360° 激光数据融合

将前左、后右两个激光雷达的扫描数据通过 TF 变换合并为一个 360° `LaserScan` 消息，供 Nav2 和 SLAM 使用。

#### 目录结构

```
laser_merger/
├── package.xml
├── setup.cfg
├── setup.py
├── resource/
└── laser_merger/
    └── laser_merger_node.py    # 激光融合节点
```

#### 工作原理

1. **时间同步**：使用 `message_filters.ApproximateTimeSynchronizer` 对齐两路雷达时间戳（容差 `slop_sec`）
2. **坐标变换**：通过 TF 将各雷达数据从各自 frame 变换到 `base_link`
3. **自遮挡过滤**：变换后检查点是否落在车身矩形内（`car_length × car_width + margin`），过滤掉打到自身的假障碍
4. **角度分箱**：将所有有效点按角度分箱到均匀网格（默认 1° 间隔），同一 bin 取最近距离
5. **输出**：发布完整的 360° `LaserScan`

#### 可配置参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `front_left_topic` | `scan_front_left` | 前左雷达话题 |
| `back_right_topic` | `scan_back_right` | 后右雷达话题 |
| `output_topic` | `scan` | 合并输出话题 |
| `base_frame` | `base_link` | 目标坐标系（自动加命名空间前缀） |
| `car_length` | 0.30 | 车身长度（与 URDF 一致） |
| `car_width` | 0.20 | 车身宽度（与 URDF 一致） |
| `body_filter_margin_x` | 0.0 | 自遮挡过滤 X 方向余量 |
| `body_filter_margin_y` | 0.0 | 自遮挡过滤 Y 方向余量 |
| `output_angle_min_deg` | -180.0 | 输出最小角度 |
| `output_angle_max_deg` | 180.0 | 输出最大角度 |
| `output_angle_increment_deg` | 1.0 | 输出角度分辨率 |
| `range_min` | 0.1 | 有效距离下限 |
| `range_max` | 10.0 | 有效距离上限 |
| `scan_frequency_hz` | 10.0 | 输出扫描频率 |
| `slop_sec` | 0.05 | 时间同步容差（秒） |

---

### 2.3 `lebot_navigation2` — 导航与建图

Nav2 导航栈配置、SLAM 建图、双机器人编排启动。

#### 目录结构

```
lebot_navigation2/
├── CMakeLists.txt
├── package.xml
├── config/
│   ├── nav2_params.yaml           # Nav2 完整参数（AMCL/Planner/Controller/Costmap）
│   ├── slam_params.yaml           # SLAM Toolbox 参数
│   ├── nav2.rviz                  # 导航 RViz 配置
│   └── slam.rviz                  # 建图 RViz 配置
├── launch/
│   ├── main_robot_nav_follow.launch.py  # 主入口：双机器人 + 导航 + 跟随
│   ├── dual_robot_bringup.launch.py     # 双机器人 Gazebo 生成编排
│   ├── navigation2.launch.py            # Nav2 导航栈（定位 + 导航 + lifecycle 管理）
│   ├── runtime_localization.launch.py   # AMCL + map_server 节点
│   ├── runtime_navigation.launch.py     # Nav2 控制/规划/行为节点
│   └── mapping.launch.py               # SLAM 建图（Gazebo + SLAM + 键盘遥控 + RViz）
└── maps/
    ├── room5.yaml / room5.pgm     # 当前默认导航地图
    └── room1~7                    # 其他版本地图存档
```

#### 各启动文件职责

| 启动文件 | 职责 | 何时使用 |
|----------|------|----------|
| `main_robot_nav_follow.launch.py` | **系统主入口**。按顺序启动：①双机器人仿真 ②延迟 6s 后启动 Nav2 导航栈 ③延迟 8s 后启动跟随控制器 | 双机器人导航+跟随场景 |
| `dual_robot_bringup.launch.py` | 双机器人编排。启动主机器人（含 Gazebo）→ 延迟 5s 后启动跟随机器人（不启动 Gazebo）→ 延迟 8s 后启动跟随相关节点 | 被 `main_robot_nav_follow` 调用，通常不单独使用 |
| `navigation2.launch.py` | Nav2 导航栈。包含定位（`runtime_localization`）+ 导航（`runtime_navigation`）+ 统一 lifecycle_manager + RViz | 被 `main_robot_nav_follow` 延迟调用，也可单独用于已运行的仿真 |
| `runtime_localization.launch.py` | 定位节点：`map_server`（加载地图）+ `amcl`（自适应蒙特卡洛定位） | 被 `navigation2` 调用 |
| `runtime_navigation.launch.py` | 导航节点：controller + smoother + planner + behavior + bt_navigator + waypoint_follower + velocity_smoother | 被 `navigation2` 调用 |
| `mapping.launch.py` | SLAM 建图。启动 Gazebo 仿真 + SLAM Toolbox + 键盘遥控 + RViz | 需要新建或更新地图时使用 |

#### 启动文件调用链

```
main_robot_nav_follow.launch.py
├── dual_robot_bringup.launch.py
│   ├── gazebo_sim.launch.py (主机器人, launch_gazebo=true)
│   │   ├── Gazebo 服务器 + 客户端
│   │   ├── robot_state_publisher × 2
│   │   ├── spawn_entity.py
│   │   ├── controller 加载 (joint_state_broadcaster → diff_drive_controller)
│   │   ├── laser_merger_node
│   │   └── ekf_node
│   ├── gazebo_sim.launch.py (跟随机器人, launch_gazebo=false, 延迟5s)
│   │   └── (同上，但不启动 Gazebo)
│   ├── entity_pose_publisher (延迟8s)
│   └── follower_controller (延迟8s)
└── navigation2.launch.py (延迟6s)
    ├── runtime_localization.launch.py
    │   ├── map_server
    │   └── amcl
    ├── runtime_navigation.launch.py
    │   ├── controller_server
    │   ├── smoother_server
    │   ├── planner_server
    │   ├── behavior_server
    │   ├── bt_navigator
    │   ├── waypoint_follower
    │   └── velocity_smoother
    ├── lifecycle_manager (统一管理所有节点生命周期)
    └── rviz2
```

---

### 2.4 `autopatrol_robot` — 自主巡逻与跟随控制

巡逻逻辑、中文语音播报、跟随控制器、领航者姿态发布。

#### 目录结构

```
autopatrol_robot/
├── package.xml
├── setup.cfg
├── setup.py
├── resource/
├── autopatrol_robot/
│   ├── __init__.py
│   ├── patrol_node.py             # 多航点巡逻节点
│   ├── speaker.py                 # 中文语音播报服务节点（espeak-ng）
│   ├── follower_controller.py     # 跟随控制器（基于 Gazebo 姿态）
│   └── entity_pose_publisher.py   # Gazebo 实体姿态发布器
├── config/
│   └── patrol_config.yaml         # 巡逻航点配置
└── launch/
    └── autopatrol.launch.py       # 巡逻启动文件
```

#### 各节点职责

| 节点 | 职责 | 话题/服务 |
|------|------|-----------|
| `patrol_node` | 继承 `BasicNavigator`，循环导航到配置的航点列表，每到达一个航点调用语音服务播报 | 调用 `speech_text` 服务；参数 `initial_point`、`target_points` |
| `speaker` | 语音播报服务节点，使用 espeak-ng 中文语音引擎 | 提供 `speech_text` 服务（`SpeechText.srv`：`string text → bool result`） |
| `follower_controller` | 跟随控制器：订阅领航者目标姿态，查询跟随者当前姿态，比例控制 + EMA 平滑输出 `cmd_vel` | 订阅 `/<main_ns>/follow_target_pose`；发布 `/<follower_ns>/cmd_vel`；调用 `/gazebo_state/get_entity_state` |
| `entity_pose_publisher` | 领航者姿态发布器：定时从 Gazebo 查询主机器人世界坐标姿态并发布 | 发布 `/<main_ns>/follow_target_pose`；调用 `/gazebo_state/get_entity_state` |

#### 跟随控制算法

1. `entity_pose_publisher` 以 15Hz 查询 Gazebo 获取主机器人世界坐标姿态，发布到 `/<main_ns>/follow_target_pose`
2. `follower_controller` 以 10Hz 控制频率：
   - 订阅领航者姿态，计算目标点 = 领航者位置 − `follow_distance` × 领航者朝向单位向量
   - 查询 Gazebo 获取跟随者当前世界坐标姿态
   - 将世界坐标系下的位置误差转换到跟随者本体坐标系
   - 计算距离误差和航向误差
   - 比例控制：线速度 = `linear_gain` × 前向误差，角速度 = `angular_gain` × 航向误差
   - EMA 低通滤波（smoothing_factor=0.3）平滑速度输出，防止抖动
   - 限幅：最大前进 0.5 m/s，最大后退 0.3 m/s，最大角速度 1.8 rad/s

#### 巡逻航点配置

`config/patrol_config.yaml` 定义巡逻航点：

```yaml
patrol_node:
  ros__parameters:
    initial_point: [0.0, 0.0, 0.0]       # 初始位姿 [x, y, yaw]
    target_points: [                        # 巡逻航点列表，每 3 个一组 [x, y, yaw]
      0.0,  0.0,  0.0,
      1.0,  2.0,  3.14,
      -4.5, 1.5,  1.57,
      -8.0, -5.0, 1.57,
      1.0,  -5.0, 3.14,
    ]
```

---

### 2.5 `autopatrol_interfaces` — 自定义服务接口

```
autopatrol_interfaces/
├── CMakeLists.txt
├── package.xml
└── srv/
    └── SpeechText.srv    # 语音合成服务接口
```

**`SpeechText.srv` 定义：**

```
string text      # 请求：要播报的文本
---
bool result      # 响应：是否播报成功
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
sudo apt install espeak-ng
pip3 install espeakng

# TF 工具
pip3 install transforms3d
sudo apt install ros-humble-tf-transformations

# 键盘遥控（建图时使用）
sudo apt install ros-humble-teleop-twist-keyboard

# 模型查看（可选，view_model.launch.py 需要）
sudo apt install ros-humble-joint-state-publisher-gui
```

### 3.2 编译与 Source

```bash
cd ~/WorkSpace/lebot_ws
colcon build
source install/setup.bash
```

> **注意**：每次打开新终端都需 `source install/setup.bash`，或将其加入 `~/.bashrc`：
> ```bash
> echo "source ~/WorkSpace/lebot_ws/install/setup.bash" >> ~/.bashrc
> ```

---

### 3.3 单机器人仿真（Gazebo + 控制器）

最基础的启动方式，只启动一个机器人 + Gazebo 仿真环境 + 控制器：

```bash
ros2 launch lebot_description gazebo_sim.launch.py
```

**启动流程：**

1. 启动 Gazebo 服务器 + 客户端，加载 `custom_room.world`
2. 启动 `robot_state_publisher`（发布 TF）+ `rviz_robot_description_publisher`（供 RViz 显示模型）
3. 调用 `spawn_entity.py` 将机器人模型生成到 Gazebo（默认位置 x=0, y=0, z=0.03）
4. 机器人生成成功后，链式加载控制器：
   - 先加载 `lebot_joint_state_broadcaster`（发布关节状态）
   - 再加载 `lebot_diff_drive_controller`（差速驱动控制器，订阅 `cmd_vel`）
5. 启动 `laser_merger_node`（合并双雷达为 360° scan）
6. 启动 `ekf_node`（融合里程计 + IMU）

**启动后手动控制机器人：**

```bash
# 在另一个终端
ros2 topic pub /robot/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2}, angular: {z: 0.0}}" --rate 10
```

**可覆盖参数：**

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `model` | `lebot.urdf.xacro` | 机器人模型文件路径 |
| `world` | `custom_room.world` | Gazebo 世界文件路径 |
| `robot_namespace` | `robot` | 机器人命名空间 |
| `use_sim_time` | `true` | 是否使用仿真时间 |
| `launch_gazebo` | `true` | 是否启动 Gazebo（双机器人时第二个设为 false） |
| `x_pose` | `0.0` | 初始 X 坐标 |
| `y_pose` | `0.0` | 初始 Y 坐标 |
| `z_pose` | `0.03` | 初始 Z 坐标 |
| `yaw` | `0.0` | 初始偏航角 |

**示例 — 自定义初始位置：**

```bash
ros2 launch lebot_description gazebo_sim.launch.py \
  x_pose:=-2.0 y_pose:=1.0 yaw:=1.57
```

---

### 3.4 查看 URDF 模型（无需 Gazebo）

在不启动仿真的情况下，通过 RViz + GUI 滑块查看和测试 URDF 模型：

```bash
ros2 launch lebot_description view_model.launch.py
```

启动后 RViz 打开，左侧有 GUI 滑块可手动转动轮子关节。

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `robot_namespace` | `robot` | 机器人命名空间 |

---

### 3.5 运行时 RViz 可视化

在仿真已运行的情况下，单独启动 RViz 查看机器人状态：

```bash
ros2 launch lebot_description runtime_rviz.launch.py
```

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `robot_namespace` | `robot` | 机器人命名空间 |
| `publish_robot_description` | `false` | 是否同时启动 RViz 专用 robot_description 发布器 |

---

### 3.6 双机器人导航 + 跟随（主入口）

完整系统启动，包含双机器人仿真 + Nav2 导航 + 跟随控制：

```bash
ros2 launch lebot_navigation2 main_robot_nav_follow.launch.py
```

**启动流程（按时间顺序）：**

| 时间 | 事件 |
|------|------|
| 0s | 启动主机器人（Gazebo + robot_state_publisher + spawn + 控制器 + laser_merger + EKF） |
| 0s | 启动 Gazebo 客户端（GUI） |
| 5s | 启动跟随机器人（不启动 Gazebo，其余同主机器人） |
| 6s | 启动 Nav2 导航栈（AMCL 定位 + 全局/局部规划 + lifecycle 管理 + RViz） |
| 8s | 启动跟随控制器（entity_pose_publisher + follower_controller） |

**启动后操作：**

1. 在 RViz 中使用 **"2D Pose Estimate"** 设置主机器人初始位置（与 Gazebo 中位置对应）
2. 使用 **"2D Goal Pose"** 发送导航目标，主机器人自主导航
3. 跟随机器人自动保持在主机器人身后 1m

**可覆盖参数：**

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `model` | `lebot.urdf.xacro` | 机器人模型文件路径 |
| `world` | `custom_room.world` | Gazebo 世界文件路径 |
| `map` | `room5.yaml` | 导航地图文件路径 |
| `params_file` | `nav2_params.yaml` | Nav2 参数文件路径 |
| `use_sim_time` | `true` | 使用仿真时间 |
| `use_rviz` | `true` | 是否启动导航 RViz |
| `launch_gazebo` | `true` | 是否启动 Gazebo |
| `navigation_start_delay` | `6.0` | Nav2 启动延迟（秒） |
| `main_robot_namespace` | `main_robot` | 主机器人命名空间 |
| `main_x_pose` | `0.0` | 主机器人初始 X |
| `main_y_pose` | `0.0` | 主机器人初始 Y |
| `main_z_pose` | `0.03` | 主机器人初始 Z |
| `main_yaw` | `0.0` | 主机器人初始偏航角 |
| `follower_robot_namespace` | `follower_robot` | 跟随机器人命名空间 |
| `follower_x_pose` | `-1.0` | 跟随机器人初始 X |
| `follower_y_pose` | `0.0` | 跟随机器人初始 Y |
| `follower_z_pose` | `0.03` | 跟随机器人初始 Z |
| `follower_yaw` | `0.0` | 跟随机器人初始偏航角 |
| `follower_start_delay` | `5.0` | 跟随机器人生成延迟（秒） |
| `use_follower_controller` | `true` | 是否启动跟随控制器 |
| `follower_controller_start_delay` | `8.0` | 跟随控制器启动延迟（秒） |
| `follow_distance` | `1.0` | 跟随距离（米） |
| `follower_control_rate` | `10.0` | 跟随控制频率（Hz） |
| `follower_linear_gain` | `0.8` | 线速度比例增益 |
| `follower_angular_gain` | `2.0` | 角速度比例增益 |
| `follower_max_linear_speed` | `0.5` | 跟随最大前进速度 |
| `follower_max_reverse_speed` | `0.3` | 跟随最大后退速度 |
| `follower_max_angular_speed` | `1.8` | 跟随最大角速度 |
| `leader_pose_publish_rate` | `15.0` | 领航者姿态发布频率（Hz） |
| `gazebo_state_service` | `/gazebo_state/get_entity_state` | Gazebo 实体状态查询服务 |
| `state_reference_frame` | `world` | Gazebo 状态查询参考坐标系 |

**示例 — 调整跟随距离和增益：**

```bash
ros2 launch lebot_navigation2 main_robot_nav_follow.launch.py \
  follow_distance:=2.0 \
  follower_linear_gain:=1.0 \
  follower_angular_gain:=2.5
```

---

### 3.7 SLAM 建图

创建新地图或更新现有地图：

```bash
ros2 launch lebot_navigation2 mapping.launch.py
```

**启动流程：**

1. 启动 Gazebo 仿真 + 单机器人（命名空间 `robot`）
2. 启动 SLAM Toolbox（异步建图模式）
3. 启动键盘遥控节点（在新终端窗口中）
4. 启动 RViz（建图可视化）

**操作步骤：**

1. 在键盘遥控终端中使用 `i/j/k/l/,` 键控制机器人移动
2. 驾驶机器人遍历整个房间，观察 RViz 中地图逐渐构建
3. 建图完成后保存地图：

```bash
# 在另一个终端
ros2 run nav2_map_server map_saver_cli \
  -f ~/WorkSpace/lebot_ws/src/lebot_navigation2/maps/my_map
```

这会生成 `my_map.yaml`（元数据）和 `my_map.pgm`（栅格图）两个文件。

**可覆盖参数：**

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `use_sim_time` | `true` | 使用仿真时间 |
| `robot_namespace` | `robot` | 机器人命名空间 |
| `use_rviz` | `true` | 是否启动建图 RViz |
| `use_keyboard` | `true` | 是否启动键盘遥控 |
| `keyboard_terminal_prefix` | `x-terminal-emulator -e` | 键盘终端前缀（改用其他终端可设为 `gnome-terminal --` 等） |
| `slam_params_file` | `slam_params.yaml` | SLAM Toolbox 参数文件路径 |

---

### 3.8 自主巡逻

让机器人自动沿预设航点循环巡逻，并在每个航点播报语音：

**第一步：启动导航环境**（双机器人或单机器人均可）

```bash
# 方式 A：双机器人导航 + 跟随
ros2 launch lebot_navigation2 main_robot_nav_follow.launch.py

# 方式 B：单机器人导航（需先启动仿真，再启动导航）
ros2 launch lebot_description gazebo_sim.launch.py
# 另一个终端启动 Nav2
ros2 launch lebot_navigation2 navigation2.launch.py namespace:=robot
```

**第二步：启动巡逻节点**

```bash
# 在另一个终端
ros2 launch autopatrol_robot autopatrol.launch.py
```

**启动流程：**

1. 启动 `patrol_node`（读取 `patrol_config.yaml` 中的航点，循环导航）
2. 启动 `speaker`（语音播报服务节点）

巡逻节点会依次导航到每个航点，到达后语音播报，然后前往下一个航点，循环往复。

**修改巡逻航点：** 编辑 `src/autopatrol_robot/config/patrol_config.yaml`，修改后需重新编译：

```bash
colcon build --packages-select autopatrol_robot
source install/setup.bash
```

---

## 4. 核心话题一览

### 单机器人（命名空间 `robot`）

| 话题 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `/robot/cmd_vel` | `geometry_msgs/Twist` | 订阅 | 差速控制器速度指令 |
| `/robot/odom` | `nav_msgs/Odometry` | 发布 | 轮式里程计 |
| `/robot/scan` | `sensor_msgs/LaserScan` | 发布 | 360° 合并激光数据 |
| `/robot/scan_front_left` | `sensor_msgs/LaserScan` | 发布 | 前左雷达原始数据 |
| `/robot/scan_back_right` | `sensor_msgs/LaserScan` | 发布 | 后右雷达原始数据 |
| `/robot/imu/data` | `sensor_msgs/Imu` | 发布 | IMU 数据 |
| `/robot/joint_states` | `sensor_msgs/JointState` | 发布 | 关节状态（4 个轮子） |

### 双机器人（命名空间 `main_robot` / `follower_robot`）

| 话题 | 类型 | 说明 |
|------|------|------|
| `/main_robot/cmd_vel` | `geometry_msgs/Twist` | 主机器人速度指令 |
| `/follower_robot/cmd_vel` | `geometry_msgs/Twist` | 跟随机器人速度指令（由 follower_controller 发布） |
| `/main_robot/follow_target_pose` | `geometry_msgs/PoseStamped` | 主机器人世界坐标姿态（由 entity_pose_publisher 发布） |
| `/main_robot/scan` | `sensor_msgs/LaserScan` | 主机器人 360° 激光 |
| `/follower_robot/scan` | `sensor_msgs/LaserScan` | 跟随机器人 360° 激光 |

### 服务

| 服务 | 类型 | 说明 |
|------|------|------|
| `/gazebo_state/get_entity_state` | `gazebo_msgs/GetEntityState` | 查询 Gazebo 中实体姿态（跟随控制用） |
| `speech_text` | `autopatrol_interfaces/SpeechText` | 中文语音播报 |

---

## 5. TF 坐标系树

单机器人（命名空间 `robot`）的 TF 树结构：

```
map
 └── robot/odom
      └── robot/base_footprint
           └── robot/base_link
                ├── robot/front_left_link
                ├── robot/back_right_link
                ├── robot/imu_link
                ├── robot/left_front_wheel_link
                ├── robot/right_front_wheel_link
                ├── robot/left_back_wheel_link
                └── robot/right_back_wheel_link
```

- `map → odom`：由 AMCL 发布（导航时）或 SLAM 发布（建图时）
- `odom → base_footprint`：由 EKF 节点发布（融合里程计 + IMU）
- `base_footprint → base_link`：由 `robot_state_publisher` 发布（static）
- `base_link → 各传感器/轮子 link`：由 `robot_state_publisher` 发布（static）

---

## 6. 常见问题

### Q: 编译报 CMakeCache.txt 路径错误

工作区被移动后 `build/` 目录中残留旧路径缓存。清除后重新编译：

```bash
rm -rf build/ install/ log/
colcon build
```

### Q: spawn_entity 报 "Entity already exists"

世界文件 `custom_room.world` 中内嵌了 robot 模型定义，与 `spawn_entity.py` 冲突。需从世界文件中移除内嵌的 robot 模型，机器人应只由 launch 文件通过 URDF 动态生成。

### Q: Gazebo 中机器人掉落或抖动

检查 `z_pose` 参数，确保机器人初始高度略高于轮子半径（默认 0.03 m = 轮半径 0.05 m − 底盘半高 0.05 m + 间隙 0.03 m）。

### Q: 导航时机器人不移动

1. 确认 Nav2 lifecycle 节点已激活：`ros2 lifecycle list /robot/controller_server`
2. 确认 AMCL 已定位：RViz 中粒子云应收敛到机器人位置
3. 使用 "2D Pose Estimate" 设置初始位置后再发目标

### Q: 跟随机器人不动

1. 确认 `entity_pose_publisher` 和 `follower_controller` 已启动（延迟 8s 后）
2. 检查 Gazebo state 服务是否可用：`ros2 service call /gazebo_state/get_entity_state gazebo_msgs/srv/GetEntityState "{name: 'main_robot', reference_frame: 'world'}"`

---

## 7. 项目目录总览

```
lebot_ws/
├── docs/                              # 开发文档
│   ├── MULTI_ROBOT_FOLLOW_ARCHITECTURE_NOTES.md
│   └── WORKSPACE_DIFF_AND_NAMESPACE_NOTES.md
├── src/
│   ├── autopatrol_interfaces/         # 自定义 ROS 2 服务接口（SpeechText.srv）
│   ├── autopatrol_robot/              # 巡逻、语音、跟随控制节点
│   ├── laser_merger/                  # 激光雷达数据融合
│   ├── lebot_description/             # 机器人模型、仿真环境、控制器配置
│   └── lebot_navigation2/             # Nav2 导航、SLAM 建图、双机器人编排
├── .gitignore
└── README.md                          # 本文档
```

---

## 8. 许可证

Apache-2.0
