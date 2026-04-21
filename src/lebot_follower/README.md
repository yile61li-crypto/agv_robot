# lebot_follower

LeBot 跟随控制器，负责领航者姿态发布和跟随机器人控制。

> **功能包定位**：这是项目的协同控制层，通过 Gazebo 实体状态服务实现双机器人协同，不依赖复杂导航栈，适合初学者理解机器人控制基础。

----

## 目录

- [功能概述](#功能概述)
- [节点详解](#节点详解)
- [使用说明](#使用说明)
- [参数详解](#参数详解)
- [算法原理详解](#算法原理详解)
- [依赖](#依赖)
- [许可证](#许可证)

---

## 功能概述

本功能包实现基于 Gazebo 姿态查询的跟随控制，包含两个核心节点：

### 1. entity_pose_publisher（领航者姿态发布）

定期查询 Gazebo 中的领航者（主机器人）实体状态，将其世界坐标姿态发布为 ROS 话题。

**工作流程：**
1. 连接 Gazebo 的 `/gazebo_state/get_entity_state` 服务
2. 以指定频率（默认 15Hz）发送查询请求
3. 获取主机器人在世界坐标系中的位姿（位置和朝向）
4. 转换为 `geometry_msgs/PoseStamped` 消息发布

### 2. follower_controller（跟随控制器）

基于领航者姿态和自身姿态，计算跟随速度指令。

**控制算法流程：**
1. 订阅领航者姿态话题
2. 查询自身在 Gazebo 中的姿态
3. 计算目标跟随点（领航者后方指定距离）
4. 计算位置和航向误差
5. 比例控制输出速度
6. EMA 平滑和限幅后发布

## 节点详解

### entity_pose_publisher

**输入：**
- Gazebo 服务：`/gazebo_state/get_entity_state`

**输出：**
- 话题：`/<main_ns>/follow_target_pose`
- 类型：`geometry_msgs/PoseStamped`
- 频率：可配置（默认 15Hz）

**核心代码逻辑：**
```python
# 查询领航者姿态
request = GetEntityState.Request()
request.name = self.entity_name          # 如 "main_robot"
request.reference_frame = "world"        # 世界坐标系

# 异步调用服务
future = self.client.call_async(request)
response = await future

# 转换为 PoseStamped 发布
pose_msg = PoseStamped()
pose_msg.header.frame_id = "world"
pose_msg.pose = response.state.pose
self.publisher.publish(pose_msg)
```

### follower_controller

**输入：**
- 话题：`/<main_ns>/follow_target_pose`（领航者姿态）
- 服务：`/gazebo_state/get_entity_state`（自身姿态查询）

**输出：**
- 话题：`/<follower_ns>/cmd_vel`
- 类型：`geometry_msgs/Twist`
- 频率：可配置（默认 10Hz）

**核心控制循环：**
```python
# 1. 计算目标点（领航者后方 follow_distance 处）
target_x = leader_x - follow_distance * cos(leader_yaw)
target_y = leader_y - follow_distance * sin(leader_yaw)

# 2. 计算误差（转换到跟随者本体坐标系）
dx = target_x - follower_x
dy = target_y - follower_y
error_x = dx * cos(follower_yaw) + dy * sin(follower_yaw)   # 纵向误差
error_y = -dx * sin(follower_yaw) + dy * cos(follower_yaw)  # 横向误差
error_yaw = atan2(error_y, error_x)                         # 航向误差

# 3. 比例控制
linear_vel = linear_gain * error_x
angular_vel = angular_gain * error_yaw

# 4. EMA 平滑
smoothed_linear = alpha * linear_vel + (1-alpha) * prev_linear
smoothed_angular = alpha * angular_vel + (1-alpha) * prev_angular

# 5. 限幅并发布
linear_vel = clamp(smoothed_linear, -max_reverse, max_linear)
angular_vel = clamp(smoothed_angular, -max_angular, max_angular)
```

## 使用说明

### 自动启动（推荐）

跟随节点通常由 `dual_robot_bringup.launch.py` 自动启动（延迟 8 秒）：

```bash
ros2 launch lebot_navigation2 main_robot_nav_follow.launch.py
```

启动时间线：
- 0s：主机器人启动
- 5s：跟随机器人启动
- 6s：Nav2 导航栈启动
- 8s：跟随控制器启动（本包节点）

### 手动启动（调试）

如需单独调试跟随功能，在 Gazebo 运行后手动启动：

```bash
# 终端 1：启动领航者姿态发布
ros2 run lebot_follower entity_pose_publisher \
  --ros-args \
  -p entity_name:=main_robot \
  -p pose_topic:=/main_robot/follow_target_pose \
  -p publish_rate:=15.0

# 终端 2：启动跟随控制器
ros2 run lebot_follower follower_controller \
  --ros-args \
  -p target_pose_topic:=/main_robot/follow_target_pose \
  -p follower_entity_name:=follower_robot \
  -p follow_distance:=1.0 \
  -p linear_gain:=0.8 \
  -p angular_gain:=2.0 \
  -p control_rate:=10.0
```

### 动态调整参数

启动后可通过 rqt 或命令行动态调整：

```bash
# 修改跟随距离
ros2 param set /follower_controller follow_distance 2.0

# 修改线速度增益（增大响应更快，但可能抖动）
ros2 param set /follower_controller linear_gain 1.2

# 修改角速度增益
ros2 param set /follower_controller angular_gain 2.5
```

## 参数详解

### entity_pose_publisher 参数

| 参数名 | 默认值 | 类型 | 说明 |
|--------|--------|------|------|
| `entity_name` | `main_robot` | string | Gazebo 实体名称，必须与 spawn_entity 时设置的名称一致 |
| `pose_topic` | `/main_robot/follow_target_pose` | string | 输出姿态话题名称，包含命名空间前缀 |
| `reference_frame` | `world` | string | 参考坐标系，通常为 world |
| `publish_rate` | `15.0` | double | 发布频率（Hz），建议 10~30Hz |

### follower_controller 参数

**目标与定位参数**
| 参数名 | 默认值 | 类型 | 说明 |
|--------|--------|------|------|
| `target_pose_topic` | `/main_robot/follow_target_pose` | string | 领航者姿态话题 |
| `follower_entity_name` | `follower_robot` | string | 跟随机器人在 Gazebo 中的实体名 |
| `reference_frame` | `world` | string | Gazebo 查询的参考坐标系 |

**控制参数**
| 参数名 | 默认值 | 类型 | 说明 | 调节建议 |
|--------|--------|------|------|----------|
| `follow_distance` | `1.0` | double | 跟随距离（米） | 0.5~2.0，过小易碰撞 |
| `linear_gain` | `0.8` | double | 线速度比例增益 | 0.5~1.5，过大易抖动 |
| `angular_gain` | `2.0` | double | 角速度比例增益 | 1.0~3.0，过大易超调 |
| `control_rate` | `10.0` | double | 控制频率（Hz） | 10~20Hz，无需过高 |

**平滑与限幅参数**
| 参数名 | 默认值 | 类型 | 说明 |
|--------|--------|------|------|
| `smoothing_factor` | `0.3` | double | EMA 平滑系数 α（0~1），越大越跟隨當前值，越小越平滑 |
| `max_linear_speed` | `0.5` | double | 最大前进速度（m/s） |
| `max_reverse_speed` | `0.3` | double | 最大后退速度（m/s） |
| `max_angular_speed` | `1.8` | double | 最大角速度（rad/s） |

### 参数调节指南

**跟随距离（follow_distance）**
- 太近（<0.5m）：主机器人转向时容易碰撞
- 适中（1.0m）：平衡安全和响应性
- 太远（>2.0m）：跟随响应变慢，容易丢失

**增益调节（linear_gain / angular_gain）**
- 增益过低：跟随响应慢，无法跟上主机器人
- 增益适中：平滑跟随，保持设定距离
- 增益过高：抖动明显，速度频繁变化

**EMA 平滑（smoothing_factor）**
- α = 0.1：非常平滑，但响应慢
- α = 0.3：平衡（默认推荐）
- α = 0.7：响应快，但可能抖动

## 算法原理详解

### 坐标系转换

跟随控制的核心是坐标系转换，将世界坐标系下的误差转换到跟随者本体坐标系：

```
世界坐标系 (world)
    │
    │ 主机器人位姿 (x_L, y_L, θ_L)
    │
    ▼
┌─────────────────────┐
│                     │
│   · 目标点          │  ← 目标点 = (x_L - d·cos(θ_L), y_L - d·sin(θ_L))
│    \                │
│     \ 跟随距离 d    │
│      \              │
│       ● 主机器人    │
│        θ_L          │
└─────────────────────┘
         │
         │ 误差向量 (dx, dy)
         ▼
┌─────────────────────┐
│                     │
│       ● 跟随机器人  │  ← 跟随者位姿 (x_F, y_F, θ_F)
│        θ_F          │
│                     │
└─────────────────────┘

本体坐标系误差计算：
error_x = dx·cos(θ_F) + dy·sin(θ_F)   # 纵向距离误差
error_y = -dx·sin(θ_F) + dy·cos(θ_F) # 横向偏移误差
error_θ = atan2(error_y, error_x)     # 朝向误差
```

### 比例控制（P 控制）

最简单的闭环控制形式，输出与误差成正比：

```
v = Kp_linear × error_x
ω = Kp_angular × error_θ
```

**优点：**
- 实现简单，易于理解和调试
- 响应快速，无明显延迟

**缺点：**
- 存在稳态误差（无法完全消除误差）
- 增益过高会导致系统振荡

### EMA 指数移动平均平滑

用于平滑速度输出，防止抖动：

```
smoothed_v = α × current_v + (1-α) × prev_smoothed_v
```

其中 α ∈ [0, 1] 是平滑系数：
- α 接近 1：更相信当前值，响应快
- α 接近 0：更相信历史值，更平滑

**在本项目中的应用：**
由于 Gazebo 仿真中存在一定的噪声，直接使用原始控制输出会导致速度频繁波动。通过 EMA 平滑后，机器人运动更加平稳。

### 控制流程图

```
┌──────────────┐
│ 主机器人移动 │
└──────┬───────┘
       │
       ▼
┌──────────────┐     ┌──────────────────┐
│ entity_pose  │────▶│ 目标点计算       │
│ _publisher   │     │ (后方d米处)      │
└──────────────┘     └────────┬─────────┘
                                │
                                ▼
┌──────────────┐     ┌──────────────────┐
│ 跟随者姿态   │────▶│ 误差计算         │
│ 查询         │     │ (世界→本体坐标)  │
└──────────────┘     └────────┬─────────┘
                                │
                                ▼
                       ┌──────────────────┐
                       │ 比例控制         │
                       │ (v = Kp × error) │
                       └────────┬─────────┘
                                │
                                ▼
                       ┌──────────────────┐
                       │ EMA 平滑         │
                       └────────┬─────────┘
                                │
                                ▼
                       ┌──────────────────┐
                       │ 限幅输出         │
                       └────────┬─────────┘
                                │
                                ▼
                       ┌──────────────────┐
                       │ cmd_vel 发布     │
                       └──────────────────┘
```

## 依赖

| 依赖包 | 用途 | 安装命令 |
|--------|------|----------|
| `rclpy` | ROS 2 Python 客户端库 | `ros-humble-rclpy` |
| `geometry_msgs` | 位姿、速度消息 | `ros-humble-geometry-msgs` |
| `gazebo_msgs` | Gazebo 服务接口 | `ros-humble-gazebo-msgs` |

## 许可证

Apache-2.0
