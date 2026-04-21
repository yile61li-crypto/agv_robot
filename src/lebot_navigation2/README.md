# lebot_navigation2

LeBot 导航与建图配置包，包含 Nav2 导航栈和 SLAM 建图。

> **功能包定位**：这是整个项目的导航核心层，负责机器人自主定位、路径规划、SLAM 建图以及双机器人的协同启动编排。基于 ROS 2 Nav2 导航栈构建，提供完整的自主导航能力。

> **关于路径变量**：本文档中使用 `<WORKSPACE_PATH>` 表示工作空间路径，请根据实际路径替换（如 `~/WorkSpace/lebot_ws` 或 `/home/<username>/lebot_ws`）。使用 `~` 表示当前用户的主目录。

---

## 目录

- [功能概述](#功能概述)
- [启动文件详解](#启动文件详解)
- [使用教程](#使用教程)
- [配置文件详解](#配置文件详解)
- [可调参数列表](#可调参数列表)
- [依赖说明](#依赖说明)
- [常见问题](#常见问题)
- [许可证](#许可证)

---

## 功能概述

### 1. Nav2 导航栈配置

完整的 Navigation2 配置，包含：
- **AMCL（自适应蒙特卡洛定位）**：基于激光雷达和地图实现机器人位姿估计
- **Map Server**：加载和提供静态地图
- **全局规划（NavFn）**：使用 A* 算法规划全局路径
- **局部规划（DWB）**：动态窗口法（Dynamic Window Approach）避障和轨迹优化
- **行为树导航（BT Navigator）**：支持 NavigateToPose、NavigateThroughPoses 等行为
- **速度平滑（Velocity Smoother）**：平滑输出速度指令，保护电机
- **代价地图（Costmap）**：全局和局部代价地图配置

### 2. 双机器人启动编排

通过分层 launch 文件实现复杂的多机器人协同启动：
- **时序控制**：精确控制各节点启动顺序和延迟
- **命名空间隔离**：主机器人（main_robot）和跟随机器人（follower_robot）完全隔离
- **依赖管理**：确保依赖服务就绪后才启动后续节点

### 3. SLAM 建图

基于 SLAM Toolbox 的异步建图：
- **扫描匹配**：使用 Ceres 优化进行位姿图优化
- **闭环检测**：自动检测重访区域并优化地图
- **地图保存**：支持保存为 PGM/YAML 格式供导航使用

### 4. 生命周期管理

使用 Nav2 的 Lifecycle 机制：
- **统一配置**：所有导航节点通过 lifecycle_manager 统一激活
- **状态监控**：实时监控各节点状态，失败时自动处理

## 启动文件详解

### main_robot_nav_follow.launch.py（系统主入口）

这是整个系统最常用的启动文件，一键启动完整双机器人系统：

```bash
ros2 launch lebot_navigation2 main_robot_nav_follow.launch.py
```

**完整启动流程：**

```
0.0s  ├─ 启动 Gazebo 服务器 + 客户端
      ├─ 启动主机器人（robot_state_publisher + spawn_entity + 控制器 + laser_merger + EKF）
      │
5.0s  ├─ 延迟 5 秒后启动跟随机器人（不启动 Gazebo）
      │
6.0s  ├─ 延迟 6 秒后启动 Nav2 导航栈
      │   ├─ runtime_localization.launch.py（map_server + amcl）
      │   ├─ runtime_navigation.launch.py（controller_server + planner_server + ...）
      │   ├─ lifecycle_manager
      │   └─ rviz2
      │
8.0s  └─ 延迟 8 秒后启动跟随控制器（entity_pose_publisher + follower_controller）
```

**关键参数说明：**

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `main_robot_namespace` | `main_robot` | 主机器人命名空间 |
| `follower_robot_namespace` | `follower_robot` | 跟随机器人命名空间 |
| `navigation_start_delay` | `6.0` | Nav2 启动延迟（秒） |
| `follower_start_delay` | `5.0` | 跟随机器人启动延迟（秒） |
| `follower_controller_start_delay` | `8.0` | 跟随控制器启动延迟（秒） |
| `map` | `room_05.yaml` | 导航地图文件 |

### dual_robot_bringup.launch.py（双机器人编排）

通常不单独使用，被 `main_robot_nav_follow.launch.py` 调用：

```bash
ros2 launch lebot_navigation2 dual_robot_bringup.launch.py
```

**功能：**
- 启动两个 `gazebo_sim.launch.py` 实例
- 第一个启动 Gazebo，第二个复用已有的 Gazebo
- 启动跟随控制相关节点

### navigation2.launch.py（纯导航）

在 Gazebo 已运行的情况下单独启动 Nav2：

```bash
# 先启动 Gazebo
ros2 launch lebot_description gazebo_sim.launch.py

# 再启动导航
ros2 launch lebot_navigation2 navigation2.launch.py namespace:=robot
```

**启动内容：**
- AMCL 定位（需要先用 2D Pose Estimate 设置初始位置）
- 全局/局部规划器
- 代价地图
- 速度控制器
- RViz 可视化

### mapping.launch.py（SLAM 建图）

用于创建新地图：

```bash
ros2 launch lebot_navigation2 mapping.launch.py
```

**启动内容：**
- Gazebo 仿真环境
- SLAM Toolbox（异步模式）
- 键盘遥控节点（在独立终端）
- RViz 建图可视化

**建图后保存：**
```bash
ros2 run nav2_map_server map_saver_cli -f ~/my_map
```

## 使用教程

### 教程 1：完整双机器人系统启动

```bash
# 步骤 1：启动完整系统（所有节点自动按序启动）
ros2 launch lebot_navigation2 main_robot_nav_follow.launch.py

# 步骤 2：等待约 10 秒，确保所有节点启动完成
# 查看 RViz 界面，确认看到两个机器人模型

# 步骤 3：在 RViz 中使用 "2D Pose Estimate" 工具
# - 点击工具栏按钮
# - 在主机器人位置点击并拖动，设置初始朝向
# - 观察激光雷达是否与地图匹配

# 步骤 4：使用 "2D Goal Pose" 发送导航目标
# - 在地图上点击目标位置
# - 主机器人开始规划路径并移动
# - 跟随机器人自动跟在主机器人身后
```

### 教程 2：SLAM 建图实操

```bash
# 步骤 1：启动建图环境
ros2 launch lebot_navigation2 mapping.launch.py

# 步骤 2：在键盘控制终端中，使用以下按键控制机器人
# i: 前进    k: 停止    ,: 后退
# j: 左转    l: 右转
# 驾驶机器人遍历整个房间

# 步骤 3：观察 RViz 中的地图逐渐构建
# 确保机器人访问所有区域，包括角落

# 步骤 4：保存地图（在新终端中执行）
ros2 run nav2_map_server map_saver_cli \
  -f <WORKSPACE_PATH>/src/lebot_navigation2/maps/my_map
# 示例：如工作空间为 ~/WorkSpace/lebot_ws，则使用 ~/WorkSpace/lebot_ws/src/...

# 步骤 5：地图文件会自动生成
# - my_map.pgm: 栅格图像（黑=障碍，白=自由，灰=未知）
# - my_map.yaml: 元数据文件（分辨率、原点等）
```

### 教程 3：切换导航地图

支持通过命令行参数动态切换地图，无需修改配置文件。

**使用内置地图（相对路径）：**
```bash
# 切换到 room_03 地图
ros2 launch lebot_navigation2 main_robot_nav_follow.launch.py map:=room_03.yaml

# 切换到 room_default 地图
ros2 launch lebot_navigation2 main_robot_nav_follow.launch.py map:=room_default.yaml
```

**使用自定义地图（绝对路径）：**
```bash
# 加载自定义地图（完整路径）
# 方式 1：使用 ~ 表示主目录（推荐）
ros2 launch lebot_navigation2 main_robot_nav_follow.launch.py \
  map:=~/my_custom_map.yaml

# 方式 2：使用完整路径（将 <username> 替换为实际用户名）
ros2 launch lebot_navigation2 main_robot_nav_follow.launch.py \
  map:=/home/<username>/my_custom_map.yaml
```

**地图参数传递链：**
```
main_robot_nav_follow.launch.py (map:=xxx.yaml)
    ↓
navigation2.launch.py (map)
    ↓
runtime_localization.launch.py (yaml_filename)
    ↓
map_server (加载地图)
```

### 教程 4：调整导航参数

```bash
# 示例：使用自定义地图和参数
ros2 launch lebot_navigation2 main_robot_nav_follow.launch.py \
  map:=room_03.yaml \
  params_file:=my_nav2_params.yaml \
  follow_distance:=2.0 \
  follower_linear_gain:=1.0

# 参数说明：
# - map: 指定导航地图，支持命令行变更
# - params_file: 自定义 Nav2 参数文件
# - follow_distance: 跟随距离（米）
# - follower_linear_gain: 跟随线速度增益（越大反应越快）
```

### 教程 5：调试导航问题

```bash
# 检查 Nav2 节点状态
ros2 lifecycle list /main_robot/controller_server

# 查看导航状态
ros2 topic echo /main_robot/navigate_to_pose/_action/status

# 检查代价地图
ros2 topic echo /main_robot/local_costmap/costmap

# 查看规划路径
ros2 topic echo /main_robot/plan
```


## 配置文件详解

### nav2_params.yaml

Nav2 导航栈的核心参数配置，主要包含：

**AMCL（定位）**
```yaml
amcl:
  ros__parameters:
    min_particles: 500        # 最小粒子数
    max_particles: 2000       # 最大粒子数
    laser_min_range: 0.1      # 激光最小范围
    laser_max_range: 10.0     # 激光最大范围
    update_min_d: 0.05        # 最小移动距离触发更新
    update_min_a: 0.05        # 最小旋转角度触发更新
```

**全局代价地图（Global Costmap）**
```yaml
global_costmap:
  ros__parameters:
    plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
    static_layer:
      map_subscribe_transient_local: True
    obstacle_layer:
      observation_sources: scan
      scan:
        topic: /<robot_namespace>/scan
        data_type: LaserScan
    inflation_layer:
      inflation_radius: 0.55    # 膨胀半径（米）
      cost_scaling_factor: 3.0  # 代价衰减因子
```

**局部规划器（DWB）**
```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      min_vel_x: 0.0
      max_vel_x: 0.5
      max_vel_theta: 1.8
```

### slam_params.yaml

SLAM Toolbox 的异步建图配置：

```yaml
slam_toolbox:
  ros__parameters:
    solver_plugin: solver_plugins::CeresSolver
    ceres_linear_solver: SPARSE_NORMAL_CHOLESKY
    ceres_preconditioner: SCHUR_JACOBI
    ceres_trust_strategy: LEVENBERG_MARQUARDT
    
    # 扫描匹配参数
    minimum_travel_distance: 0.5    # 最小移动距离触发扫描匹配
    minimum_travel_heading: 0.5     # 最小旋转角度触发扫描匹配
    
    # 闭环检测
    loop_search_space_dimension: 8.0  # 闭环搜索空间
    loop_search_space_resolution: 0.05
    do_loop_closing: true             # 启用闭环检测
```

### 地图文件

- **room_05.yaml / room_05.pgm**：默认导航地图
- **room_01~07**：其他版本地图存档
- **自定义地图**：通过 SLAM 建图生成，或从外部导入

## 可调参数列表

### 主启动文件参数（main_robot_nav_follow.launch.py）

**机器人位置参数**
| 参数 | 默认值 | 说明 |
|------|--------|------|
| `main_x_pose` | `0.0` | 主机器人初始 X 坐标 |
| `main_y_pose` | `0.0` | 主机器人初始 Y 坐标 |
| `main_yaw` | `0.0` | 主机器人初始朝向 |
| `follower_x_pose` | `-1.0` | 跟随机器人初始 X 坐标 |
| `follower_y_pose` | `0.0` | 跟随机器人初始 Y 坐标 |
| `follower_yaw` | `0.0` | 跟随机器人初始朝向 |

**跟随控制参数**
| 参数 | 默认值 | 说明 |
|------|--------|------|
| `follow_distance` | `1.0` | 跟随距离（米），建议 0.5~2.0 |
| `follower_linear_gain` | `0.8` | 线速度比例增益，建议 0.5~1.5 |
| `follower_angular_gain` | `2.0` | 角速度比例增益，建议 1.0~3.0 |
| `follower_max_linear_speed` | `0.5` | 最大线速度限制 |
| `follower_max_angular_speed` | `1.8` | 最大角速度限制 |

**时序控制参数**
| 参数 | 默认值 | 说明 |
|------|--------|------|
| `navigation_start_delay` | `6.0` | Nav2 启动延迟（秒） |
| `follower_start_delay` | `5.0` | 跟随机器人启动延迟（秒） |
| `follower_controller_start_delay` | `8.0` | 跟随控制器启动延迟（秒） |
| `leader_pose_publish_rate` | `15.0` | 领航者姿态发布频率（Hz） |
| `follower_control_rate` | `10.0` | 跟随控制频率（Hz） |

**文件路径参数**
| 参数 | 默认值 | 说明 |
|------|--------|------|
| `map` | `room_05.yaml` | 导航地图文件 |
| `params_file` | `nav2_params.yaml` | Nav2 参数文件 |
| `world` | `custom_room.world` | Gazebo 世界文件 |

## 依赖说明

| 依赖包 | 用途 | 安装命令 |
|--------|------|----------|
| `nav2_bringup` | Nav2 导航栈启动包 | `ros-humble-nav2-bringup` |
| `nav2_common` | Nav2 公共工具 | `ros-humble-navigation2` |
| `slam_toolbox` | SLAM 建图 | `ros-humble-slam-toolbox` |
| `robot_localization` | EKF 传感器融合 | `ros-humble-robot-localization` |
| `lebot_description` | 机器人模型和仿真（本仓库） | - |
| `lebot_follower` | 跟随控制（本仓库） | - |

## 常见问题

### Q: 导航时机器人不移动
**排查步骤：**
1. 确认 RViz 中已使用 "2D Pose Estimate" 设置初始位置
2. 检查 AMCL 是否收敛：`ros2 topic echo /main_robot/amcl_pose`
3. 确认 Nav2 lifecycle 状态：`ros2 lifecycle list /main_robot/controller_server`
4. 查看规划器日志：`ros2 topic echo /main_robot/plan` 是否有规划路径

### Q: 跟随机器人不动或抖动
**排查步骤：**
1. 确认跟随控制器已启动：`ros2 node list | grep follower`
2. 检查领航者姿态话题：`ros2 topic echo /main_robot/follow_target_pose`
3. 检查跟随速度指令：`ros2 topic echo /follower_robot/cmd_vel`
4. 调整增益参数：减小 `follower_linear_gain` 可减少抖动

### Q: 地图与实际环境不匹配
**排查步骤：**
1. 检查地图原点：`ros2 topic echo /main_robot/map_metadata`
2. 确认 AMCL 初始化位置正确
3. 如地图有误，重新进行 SLAM 建图

### Q: Gazebo 中机器人模型消失或异常
**排查步骤：**
1. 检查 Gazebo 物理仿真是否稳定
2. 查看模型是否与其他物体碰撞
3. 重启 Gazebo 仿真：`killall gzserver gzclient`

## 许可证

Apache-2.0
