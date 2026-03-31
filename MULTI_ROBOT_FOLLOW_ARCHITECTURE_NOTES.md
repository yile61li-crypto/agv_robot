# 多机器人跟随架构说明

## 1. 文档目的

这份文档用于记录当前 `lebot_ws` 中多机器人跟随方案的演进、官方资料结论、现有实现结构、当前已知问题以及后续验证与调试方向。

这不是最终用户手册，而是一份面向后续开发和排障的技术说明。

---

## 2. 当前目标

当前目标是让双机器人仿真具备如下能力：

- 主机器人和跟随机器人以独立命名空间运行
- 双机器人在 Gazebo 中稳定生成并各自加载控制器
- 跟随机器人默认保持在主机器人后方约 `1.0m`
- 跟随逻辑不依赖错误的跨树 TF 假设
- 方案尽量对齐 ROS 2 / Gazebo / Nav2 官方设计思路，避免继续堆临时补丁

---

## 3. 官方资料调研结论

### 3.1 Gazebo Classic + ROS 2 的状态接口结论

根据 Gazebo Classic 官方 ROS 2 概览与本机 `gazebo_ros` 安装中的官方示例：

- ROS 1 时代的大型 `gazebo_ros_api_plugin` 在 ROS 2 中被拆分成多个更聚焦的小插件
- 在 ROS 2 中，不应该默认假设所有状态接口天然可用
- 官方提供了 `gazebo_ros_state` 世界插件来暴露实体状态能力

在本机 `/opt/ros/humble/share/gazebo_ros/worlds/gazebo_ros_state_demo.world` 中，可以直接看到官方示例：

- 加载插件：`libgazebo_ros_state.so`
- 暴露服务：`get_entity_state`、`set_entity_state`
- 还能发布 remap 后的 model/link states topic

这说明在 ROS 2 Gazebo Classic 下，更稳妥的状态获取方式是：

- 显式加载 `gazebo_ros_state` 插件
- 通过 `GetEntityState` 服务获取实体在 `world` 或指定参考系下的姿态

而不是默认依赖：

- `/model_states`
- `/gazebo/model_states`

持续、稳定、可消费地输出期望数据。

### 3.2 Nav2 官方关于跟随的结论

Nav2 官方 `Dynamic Object Following` 文档给出的关键点：

- Nav2 有专门的 `Following Server`
- 它的目标输入有两种标准形式：
  - `pose_topic`
  - `tracked_frame`
- `pose_topic` 的消息类型是：`geometry_msgs/PoseStamped`
- `tracked_frame` 依赖目标 frame 能被当前机器人通过 TF 树正确访问
- 跟随控制底层复用了官方控制律 `SmoothControlLaw`

这说明官方推荐的抽象层是：

- 跟随模块应该消费标准化目标输入
- 目标输入可以是 pose topic
- 而不是让跟随控制器直接耦合到 Gazebo 内部状态 topic

### 3.3 Nav2 官方关于多机器人的结论

Nav2 多机器人资料中，较明确的做法包括：

- `cloned_tb3_simulation_launch.py`
- `unique_tb3_simulation_launch.py`

其核心架构思想是：

- 多机器人通过 `namespace` 隔离
- 每个机器人运行各自独立的导航栈/参数集
- 顶层 launch 仅作为 orchestrator 负责统一编排

这与当前工程的双机器人总控思路是一致的。

---

## 4. 当前工程与官方思路的对照

### 4.1 已经和官方思路一致的部分

当前工程已经具备以下正确方向：

- 使用独立的双机器人编排 launch：`dual_robot_bringup.launch.py`
- 复用单机器人 Gazebo 模板 launch：`gazebo_sim.launch.py`
- 使用独立命名空间：`main_robot`、`follower_robot`
- 将跟随机器人延后生成，避免启动时序竞争
- 尽量将多机器人逻辑放在总控层，而不是继续膨胀单机器人 launch

### 4.2 之前不符合官方思路的部分

之前尝试过两条跟随输入路径：

#### 路径 A：跨树 TF

旧方案通过以下 frame 直接查 TF：

- `main_robot/base_link`
- `follower_robot/base_link`

问题在于：

- 双机器人默认属于两棵独立 TF 树
- Gazebo 并不会自动构造一个让两者互相可查的统一 TF 关系
- 因此会出现典型报错：
  - `Tf has two or more unconnected trees`

这个失败是结构性问题，不是简单参数问题。

#### 路径 B：直接订阅 `ModelStates`

后续尝试过：

- `/model_states`
- `/gazebo/model_states`

虽然在 `ros2 topic list` 中能看到这些 topic 名字，但在当前工程环境中：

- `ros2 topic echo --once` 没有拿到有效数据
- follower 节点也一直拿不到 leader / follower 的姿态

结合官方资料和本地 world/launch 结构分析，这条路线的问题在于：

- 工程中当时没有显式加载官方状态插件
- 直接把跟随算法依赖到 Gazebo 内部状态 topic，耦合过深
- 不符合官方更推荐的“标准输入 + 状态服务”思路

---

## 5. 当前已实现的官方对齐方案

当前代码已经重构为更接近官方设计的链路。

### 5.1 世界文件显式加载状态插件

文件：

- `src/lebot_description/world/custom_room.world`

新增了官方状态插件：

- `libgazebo_ros_state.so`

当前配置：

- namespace：`/gazebo_state`
- update_rate：`30.0`

理论上它应提供：

- `/gazebo_state/get_entity_state`
- `/gazebo_state/set_entity_state`

### 5.2 新增 `entity_pose_publisher`

文件：

- `src/autopartol_robot/autopatrol_robot/entity_pose_publisher.py`

作用：

- 调用官方服务 `/gazebo_state/get_entity_state`
- 读取指定实体的姿态
- 发布为标准 `PoseStamped`

当前默认语义：

- 查询实体：主机器人
- 发布 topic：`/<main_robot_namespace>/follow_target_pose`
- 参考系：`world`

这一步的目的，是把 Gazebo 内部状态转换成上层跟随模块可消费的标准 ROS 输入。

### 5.3 重构 `follower_controller`

文件：

- `src/autopartol_robot/autopatrol_robot/follower_controller.py`

现在 follower 的工作方式是：

- 订阅目标 pose：`PoseStamped`
- 通过 `/gazebo_state/get_entity_state` 获取自身姿态
- 以 `world` 为公共参考系计算误差
- 输出 `cmd_vel` 到跟随机器人

当前 follower 不再依赖：

- `ModelStates`
- 跨树 TF

它对外依赖的输入只有两类：

- 标准目标 pose topic
- 官方 Gazebo 实体状态服务

这与官方 Follow Server 的 `pose_topic` 输入思路一致。

### 5.4 双机器人总控中的跟随链路

文件：

- `src/lebot_navigation2/launch/dual_robot_bringup.launch.py`

当前总控中新增了两类 support node：

- `leader_pose_publisher`
- `follower_controller`

其逻辑关系为：

1. 主机器人和跟随机器人按原有总控逻辑生成
2. `leader_pose_publisher` 调用 Gazebo 状态服务，持续发布主机器人目标姿态
3. `follower_controller` 订阅该目标姿态，并查询跟随机器人自身姿态
4. follower 计算相对误差并输出 `/follower_robot/cmd_vel`

---

## 6. 当前运行时数据流

当前方案的运行时数据流如下：

1. `gazebo.launch.py` 启动 Gazebo
2. world 中的 `gazebo_ros_state` 插件被加载
3. Gazebo 提供：`/gazebo_state/get_entity_state`
4. `entity_pose_publisher` 定时查询 `main_robot` 在 `world` 下的 pose
5. `entity_pose_publisher` 发布 `PoseStamped` 到 `/main_robot/follow_target_pose`
6. `follower_controller` 订阅这个 topic
7. `follower_controller` 调用 `/gazebo_state/get_entity_state` 获取 `follower_robot` 自己的 pose
8. `follower_controller` 计算 follower 应到达的目标点
9. `follower_controller` 发布 `/follower_robot/cmd_vel`
10. `lebot_diff_drive_controller` 接收速度并驱动跟随机器人运动

---

## 7. 当前还未最终验证的点

虽然链路已经按官方思路重构，但仍需要在运行时逐项确认以下项目：

### 7.1 状态插件是否真实生效

需要确认以下服务确实出现：

```bash
ros2 service list | grep gazebo_state
```

理想结果至少包括：

```bash
/gazebo_state/get_entity_state
/gazebo_state/set_entity_state
```

如果服务没有出现，说明：

- world 插件未加载成功
- 或 Gazebo 启动方式未正确带起该插件

### 7.2 主机器人目标 pose 是否能持续发布

需要确认：

```bash
ros2 topic echo /main_robot/follow_target_pose --once
```

如果没有输出，重点检查：

- `entity_pose_publisher` 是否已启动
- `/gazebo_state/get_entity_state` 是否可用
- 查询实体名是否与 Gazebo 内实际 entity 名一致

### 7.3 follower 是否能接收命令

需要确认：

```bash
ros2 topic echo /follower_robot/cmd_vel
```

如果目标 pose 和 follower pose 都正常，理论上这里应能看到控制输出。

### 7.4 follower 底层 controller subscriber 是否已激活

之前日志中曾出现：

- `Can't accept new commands. subscriber is inactive`

这通常发生在 diff drive controller 尚未完全激活时。

需要确认 follower controller 的启动时机与底盘 controller 的激活顺序是否匹配。

---

## 8. 当前已知高风险点

### 8.1 Gazebo 内部 entity 名与 namespace 名未必永远等价

当前默认假设：

- 主机器人 Gazebo entity 名 = `main_robot_namespace`
- 跟随机器人 Gazebo entity 名 = `follower_robot_namespace`

在当前 spawn 方式下这是合理的，因为 `spawn_entity.py` 的 `-entity` 参数使用了 namespace。

但后续如果改动 spawn 逻辑，必须重新确认：

- Gazebo entity 名
- ROS namespace
- follower 参数中的 `entity_name`

三者是否仍然一致。

### 8.2 当前 follower 仍是简化控制律

虽然输入链路已经更官方化，但 follower 的控制算法本体目前仍是简化版：

- 位置误差 + 朝向误差
- 线速度和角速度的比例控制
- 保持 leader 后方一个固定距离

这并不等价于完整 Nav2 Following Server。

当前控制器的优点是：

- 实现轻量
- 易于调试
- 不强依赖完整 Nav2 行为树接入

但缺点是：

- 避障能力不等于 Nav2 级别
- 对急转向、倒车、目标丢失的鲁棒性有限
- 还没有官方 Following Server 的 recovery 行为

### 8.3 world 参考系与导航参考系是不同概念

当前跟随计算使用：

- `world`

这是 Gazebo 仿真参考系。

而 Nav2 / SLAM / localization 常用的是：

- `map`
- `odom`
- `base_link`

当前方案是仿真内部跟随方案，因此使用 `world` 是合理的。

但后续如果希望把跟随逻辑迁移到更通用的导航层，需要重新设计参考系与目标输入来源。

---

## 9. 为什么当前方案比之前更合理

当前方案比旧方案合理的原因在于：

- 不再假设双机器人处在同一棵 TF 树
- 不再直接绑定不可靠的 `ModelStates` topic 作为唯一输入源
- 使用了本机官方 Gazebo ROS 示例中真实存在的 `gazebo_ros_state` 插件路径
- 使用标准 `PoseStamped` 作为 leader 目标输入
- 与 Nav2 官方 `pose_topic` 跟随输入风格一致
- 保持了现有双机器人 namespace 架构，不需要重写整套多机器人启动系统

---

## 10. 建议的验证顺序

建议按下面顺序验证，不要一次性把所有问题搅在一起。

### 第一步：确认状态插件服务

```bash
colcon build
source install/setup.bash
ros2 launch lebot_navigation2 dual_robot_bringup.launch.py
ros2 service list | grep gazebo_state
```

### 第二步：确认 leader pose provider

```bash
ros2 topic echo /main_robot/follow_target_pose --once
```

### 第三步：确认 follower 命令输出

```bash
ros2 topic echo /follower_robot/cmd_vel
```

### 第四步：人工控制主机器人

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/main_robot/cmd_vel
```

### 第五步：观察 follower 实际行为

重点看：

- 是否能开始移动
- 是否大致保持后方 `1m`
- 是否出现大角度振荡
- 是否在主机器人转弯时出现过冲

---

## 11. 后续可能的演进方向

### 方向 A：继续保留轻量 follower controller

适合当前阶段快速验证多机器人跟随链路。

下一步主要做：

- 调整线速度/角速度增益
- 增加目标丢失保护
- 增加启动期 controller 未激活时的等待机制
- 增加更详细日志，输出 follower 当前误差

### 方向 B：进一步向 Nav2 官方 Following Server 靠拢

如果后续确认本机 Nav2 版本中 Following Server 可直接稳定使用，可以继续演进为：

- 保留 `entity_pose_publisher` 作为上游目标提供者
- 将 follower 端从自写控制器替换成更官方的 Following Server 输入方式

这会让架构更标准，但接入成本会更高。

### 方向 C：将跟随逻辑提升到更通用的导航层

如果未来不只是 Gazebo 仿真跟随，而是希望迁移到实机或多种定位源，需要考虑：

- 目标 pose 改由更通用来源提供
- 将 `world` 参考切换为 `map` / `odom` 体系
- 结合 obstacle avoidance、局部规划器与动态约束

---

## 12. 当前结论

当前多机器人架构已经基本形成：

- 多机器人启动编排方式基本正确
- namespace 隔离思路正确
- 原始跨树 TF 跟随方案不可行
- 直接依赖 `ModelStates` 的方案在当前工程中不可靠
- 当前已重构为更符合官方思路的：
  - `gazebo_ros_state`
  - `GetEntityState`
  - `PoseStamped` 目标输入
  - follower 自主查询自身姿态

也就是说，当前最关键的工作已经从“盲目猜话题名”转移到了：

- 验证官方状态服务是否真实跑通
- 验证 leader pose provider 是否稳定发布
- 验证 follower controller 是否能正确输出控制命令

只要这三步通了，后续调参和行为优化就会容易很多。

---

## 13. 相关关键文件

### 启动与世界

- `src/lebot_navigation2/launch/dual_robot_bringup.launch.py`
- `src/lebot_description/launch/gazebo_sim.launch.py`
- `src/lebot_description/world/custom_room.world`

### 跟随链路

- `src/autopartol_robot/autopatrol_robot/entity_pose_publisher.py`
- `src/autopartol_robot/autopatrol_robot/follower_controller.py`
- `src/autopartol_robot/setup.py`
- `src/autopartol_robot/package.xml`

### 参考说明

- `WORKSPACE_DIFF_AND_NAMESPACE_NOTES.md`

---

## 14. 建议后面阅读顺序

如果后面要慢慢回看，建议阅读顺序如下：

1. 先看第 3 节：官方资料结论
2. 再看第 4 节：旧方案为什么不行
3. 再看第 5 节：当前新方案怎么接起来的
4. 然后看第 7 节和第 10 节：怎么验证
5. 最后看第 11 节：后续演进方向
