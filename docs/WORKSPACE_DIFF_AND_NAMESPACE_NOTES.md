# `/home/lyl/lebot_ws` 与 `/home/lyl/lebot_ws_d` 工作空间差异说明

## 1. 范围说明

本说明对比的是两个工作空间的 `src/` 源码树：

- 当前版本：`/home/lyl/lebot_ws/src`
- 单机版本：`/home/lyl/lebot_ws_d/src` 

不比较 `build/`、`install/`、`log/`。

另外，`__pycache__` 这类自动生成文件不计入“功能性改动”，只在原始差异扫描中出现一次，文中不作为有效改动讨论。

---

## 2. 工作空间级别总览

两个工作空间包含的包集合相同，没有包级别的增删：

- `autopatrol_robot`
- `autopatrol_interfaces`
- `laser_merger`
- `lebot_description`
- `lebot_navigation2`

**注意**: 单机版包名为 `autopartol_robot` (拼写错误), 当前版已修正为 `autopatrol_robot`。

其中：

- **未发现源码差异的包**
  - `autopatrol_interfaces`

- **存在源码差异的包**
  - `autopatrol_robot` (新增跟随链路节点 + 包名修正)
  - `laser_merger`
  - `lebot_description`
  - `lebot_navigation2`

---

## 3. 包级差异清单

## 3.1 `autopatrol_robot`

### 3.1.1 包名修正

单机版目录名为 `autopartol_robot` (拼写错误), 当前版已重命名为 `autopatrol_robot`。`setup.py`、`setup.cfg`、`package.xml` 中的包名同步修正。

### 3.1.2 新增文件

- `autopatrol_robot/entity_pose_publisher.py` — 从 Gazebo 获取主机器人姿态并发布为 `PoseStamped`
- `autopatrol_robot/follower_controller.py` — 跟随控制器, 订阅目标姿态 + 查询自身姿态, 输出 `cmd_vel`

### 3.1.3 改动意义

这两个节点构成了双机器人跟随链路的核心:

1. `entity_pose_publisher` 以 15Hz 调用 `/gazebo_state/get_entity_state` 获取主机器人世界坐标姿态, 发布到 `/main_robot/follow_target_pose`
2. `follower_controller` 订阅该目标姿态, 同时查询跟随机器人自身姿态, 通过比例控制 + EMA 平滑输出 `/follower_robot/cmd_vel`

这使得跟随逻辑不再依赖跨树 TF 或不可靠的 `ModelStates` topic。

---

## 3.2 `laser_merger`

### 3.2.1 发生变化的文件

- `laser_merger/laser_merger_node.py`（修改）

### 3.2.2 改动明细

#### A. 输入/输出话题由绝对名改为相对名

单机版默认参数：

- `front_left_topic: /scan_front_left`
- `back_right_topic: /scan_back_right`
- `output_topic: /scan`

当前版默认参数：

- `front_left_topic: scan_front_left`
- `back_right_topic: scan_back_right`
- `output_topic: scan`

**意义**：

- 单机版写成绝对话题，节点无论放到什么命名空间，都会直接订阅全局 `/scan_front_left`、`/scan_back_right`、发布 `/scan`
- 当前版改成相对话题后，如果节点本身运行在 `/main_robot` 命名空间下，就会自然解析为：
  - `/main_robot/scan_front_left`
  - `/main_robot/scan_back_right`
  - `/main_robot/scan`

这一步是整个多机器人/命名空间改造里最典型、最重要的改动之一。

#### B. `base_frame` 从裸名改成“运行时自动补命名空间”

当前版新增了 `resolve_frame_name()`，会把：

- `base_link`

在节点有命名空间时解析成：

- `main_robot/base_link`

**意义**：

- 话题名可以依赖 ROS namespace 自动展开
- 但 TF frame 不是 ROS topic，它只是字符串
- 因此 frame 不能只写 `base_link` 然后期待系统自动替你补成 `main_robot/base_link`
- 必须手动在程序或参数层把 frame name 变成真正存在的 TF frame 名

---

## 3.3 `lebot_description`

### 3.3.1 新增文件

- `launch/runtime_rviz.launch.py`

### 3.3.2 修改文件

- `config/ekf_config.yaml`
- `config/lebot.rviz`
- `config/lebot_ros2_controller.yaml`
- `launch/gazebo_sim.launch.py`
- `launch/view_model.launch.py`
- `package.xml`
- `urdf/lebot/base.urdf.xacro`
- `urdf/lebot/gazebo_sensor_plugin.xacro`
- `urdf/lebot/imu.xacro`
- `urdf/lebot/laser.urdf.xacro`
- `urdf/lebot/lebot.ros2_control.xacro`
- `urdf/lebot/lebot.urdf.xacro`
- `urdf/lebot/wheel.urdf.xacro`

### 3.3.3 改动主线

`lebot_description` 的改动本质上是：

- 让机器人本体、Gazebo 插件、`robot_state_publisher`、EKF、RViz、激光融合节点
- 全部从单机默认命名
- 变成以 `robot_namespace` 为入口的命名空间化系统

### 3.3.4 核心改动明细

#### A. `gazebo_sim.launch.py` 新增 `robot_namespace` 入口

当前版新增了 `robot_namespace` 启动参数，默认值是：

- `main_robot`

并把该参数贯穿到：

- xacro 展开
- `robot_state_publisher`
- `spawn_entity.py`
- controller manager 加载命令
- `laser_merger`
- `robot_localization` EKF

**这是整个命名空间改造的总入口。**

#### B. xacro/URDF 被改成支持 `robot_namespace` 与 `link_prefix`

从当前代码可以看出，`lebot.urdf.xacro` 和相关传感器 xacro 已支持：

- `robot_namespace`
- `link_prefix`
- `frame_prefix`

例如：

- 激光插件宏接收 `robot_namespace`、`frame_prefix`
- IMU 插件 frame name 使用带前缀的 link 名称
- 底盘与传感器 link 名也支持 `link_prefix`

**意义**：

- 单机版里 link / frame 默认是 `base_link`、`odom`、`front_left_link` 之类
- 当前版则可以形成：
  - `main_robot/base_link`
  - `main_robot/base_footprint`
  - `main_robot/front_left_link`

#### C. `robot_state_publisher` 从“默认全局”改成“节点在命名空间内，但 TF 仍发布到全局 `/tf` / `/tf_static`”

当前版 `gazebo_sim.launch.py` 中：

- `robot_state_publisher` 节点运行在 `robot_namespace` 下
- 但显式 remap：
  - `tf -> /tf`
  - `tf_static -> /tf_static`

同时使用：

- `frame_prefix = <robot_namespace>/`

**意义**：

- 节点命名空间和 TF 话题命名空间不是一回事
- 本项目最终选择的是：
  - **frame 名带命名空间前缀**
  - **TF 话题保持全局 `/tf` / `/tf_static`**

这比“把 TF 话题也塞进命名空间”更稳定，因为 ROS2 生态里很多节点默认都看全局 TF。

#### D. 为 RViz 单独加了一套 robot description 发布通道

当前版新增了 RViz 专用的 `robot_state_publisher`：

- 发布专用 `robot_description`
- 使用空 `frame_prefix`
- remap 到：
  - `rviz_robot_description_tf`
  - `rviz_robot_description_tf_static`

**意义**：

- Gazebo/控制/TF 的实际运行链路和 RViz 仅显示用途分离
- 避免 RViz 展示与真实控制链路互相干扰

#### E. `spawn_entity.py` 改成按命名空间生成机器人

当前版生成实体时使用了：

- `-entity <robot_namespace>`
- `-robot_namespace <robot_namespace>`
- `-topic spawn_robot_description`

**意义**：

- Gazebo 内机器人实例本身就是带命名空间的
- 控制器、插件、传感器节点都能落到同一个机器人命名空间树下

#### F. 控制器管理器路径改成命名空间路径

当前版通过表达式构造：

- `/<robot_namespace>/controller_manager`

再传给 `ros2 control load_controller`。

**意义**：

- 单机版默认 controller manager 在全局或默认路径下
- 多机器人/命名空间版本必须精确指定对应机器人自己的 controller manager

#### G. EKF 参数模板化

`config/ekf_config.yaml` 当前版使用了：

- `odom_frame: <robot_namespace>/odom`
- `base_link_frame: <robot_namespace>/base_link`
- `world_frame: <robot_namespace>/odom`

并在 launch 中用 `ReplaceString` 注入真正命名空间。

同时 EKF 节点运行在命名空间内，并 remap：

- `odometry/filtered -> odom`
- `tf -> /tf`
- `tf_static -> /tf_static`

**意义**：

- 保证 EKF 输出的 `odom`、TF frame、节点命名空间三者一致
- 防止单机版的 `odom/base_link` 与多机器人场景冲突

#### H. `view_model.launch.py`、`runtime_rviz.launch.py`、`config/lebot.rviz` 改成命名空间感知

从当前代码可见，RViz 相关文件已支持：

- `robot_namespace`
- `ReplaceString` 替换 RViz 配置里的命名空间占位符

**意义**：

- 单机版 RViz 常订阅全局 `/scan`、`/odom`、`/robot_description`
- 命名空间版如果不改 RViz，界面经常会“看不见机器人/激光/地图”，即使底层数据实际存在

#### I. `lebot_ros2_controller.yaml` 支持 TF prefix 机制

当前版控制器配置出现了：

- `tf_frame_prefix_enable: true`
- `tf_frame_prefix: ""`

这说明控制器链路也被改造成可兼容前缀/命名空间设计。

---

## 3.4 `lebot_navigation2`

### 3.4.1 新增文件

- `config/nav2.rviz`
- `config/slam_params.yaml`
- `launch/runtime_localization.launch.py`
- `launch/runtime_navigation.launch.py`
- `launch/dual_robot_bringup.launch.py` — 双机器人编排总控
- `launch/main_robot_nav_follow.launch.py` — 导航+跟随主入口
- `launch/mapping.launch.py` — SLAM 建图入口
- `maps/room5.pgm` / `room5.yaml` — 统一默认地图
- `maps/room6.pgm` / `room6.yaml`

**注意**: 单机版曾有 `launch/slam.launch.py`, 当前版已删除 (功能合并到 `mapping.launch.py`)。

### 3.4.2 修改文件

- `config/nav2_params.yaml`
- `launch/navigation2.launch.py`
- `package.xml`

### 3.4.3 改动主线

`lebot_navigation2` 的变化最大，核心目标是：

- 从单机版 Nav2 配置
- 迁移到支持 `main_robot` 命名空间的 Nav2 + SLAM + RViz 方案
- 并绕开上游 `nav2_bringup` 在该项目里引出的 TF 问题
- 新增双机器人编排和导航+跟随的统一入口

### 3.4.4 核心改动明细

#### A. `navigation2.launch.py` 从“直接包含上游 bringup”改为“包含本地运行时 launch”

单机版：

- 直接 `IncludeLaunchDescription(nav2_bringup/launch/bringup_launch.py)`
- RViz 直接加载上游 `nav2_default_view.rviz`
- 没有 `namespace` 启动参数

当前版：

- 新增 `namespace` 参数，默认 `main_robot`
- 新增 `use_rviz` 参数
- 不再直接走上游 `bringup_launch.py`
- 改为先后包含：
  - `runtime_localization.launch.py`
  - `runtime_navigation.launch.py`
- RViz 改为加载包内的 `config/nav2.rviz`
- 启动时设置 `LIBGL_ALWAYS_SOFTWARE=1`

**意义**：

- 上游 bringup 在此项目里会引入不合适的 TF remap 行为
- 本地拆分 launch 后，命名空间、参数替换、cmd_vel remap、lifecycle 控制都可精确掌控
- RViz 也必须切到本地命名空间版配置，否则默认只看全局 `/map`、`/scan`

#### B. 新增 `runtime_localization.launch.py`

这是当前版新引入的“本地最小定位启动链”，负责启动：

- `map_server`
- `amcl`
- `lifecycle_manager_localization`

其关键技术点：

- `ReplaceString`：把参数文件中的 `<robot_namespace>` 替换为真实 namespace
- `RewrittenYaml(root_key=namespace)`：让 Nav2 参数树落在命名空间根下
- `yaml_filename` 在 launch 时注入

**意义**：

- 单机版参数写法默认是平铺的
- 命名空间版需要把参数树、frame、话题、生命周期节点全部挂在同一个 namespace 下

#### C. 新增 `runtime_navigation.launch.py`

这是当前版新引入的“本地最小导航启动链”，负责启动：

- `controller_server`
- `smoother_server`
- `planner_server`
- `behavior_server`
- `bt_navigator`
- `waypoint_follower`
- `velocity_smoother`
- `lifecycle_manager_navigation`

关键点：

- 同样使用 `ReplaceString + RewrittenYaml`
- `cmd_vel` 做了显式重映射：
  - `controller_server: cmd_vel -> cmd_vel_nav`
  - `velocity_smoother: cmd_vel -> cmd_vel_nav`
  - `velocity_smoother: cmd_vel_smoothed -> cmd_vel`

**意义**：

- 将局部规划输出和最终底盘控制分离
- 保留速度平滑器这一层
- 避免多个节点同时直接抢占最终 `/cmd_vel`

#### D. `nav2_params.yaml` 被整体改造成“模板化命名空间参数文件”

单机版大量 frame/topic 是裸名或全局名，例如：

- `base_footprint`
- `base_link`
- `odom`
- `/scan`
- `/odom`

当前版改成：

- `base_frame_id: <robot_namespace>/base_footprint`
- `odom_frame_id: <robot_namespace>/odom`
- `robot_base_frame: <robot_namespace>/base_link`
- `global_frame: <robot_namespace>/odom`（行为树局部相关处）
- `scan` 相关观测源改成 `/<robot_namespace>/scan`

具体上至少覆盖了：

- `amcl`
- `bt_navigator`
- `local_costmap`
- `global_costmap`
- `behavior_server`

**意义**：

- frame 名本身必须唯一，不能继续沿用单机版的裸名
- 激光输入在嵌套 costmap 节点下，使用相对名 `scan` 很容易被解析到错误路径
- 因此障碍层最终使用了绝对的 `/<robot_namespace>/scan`

#### E. 本次还保留了一个导航行为调参差异

当前版相对于单机版，还保留了：

- `local_costmap.inflation_radius: 0.30`
- `global_costmap.inflation_radius: 0.30`

单机版为：

- `0.20`

这是为改善拐角切弯、贴墙问题做的保留调参。

#### F. SLAM 建图启动重构

单机版的 `lebot_navigation2` 里没有本地 SLAM 启动/参数文件。

当前版新增了 `config/slam_params.yaml`, 已写成命名空间化格式:

- `odom_frame: main_robot/odom`
- `base_frame: main_robot/base_footprint`
- `scan_topic: /main_robot/scan`

启动文件方面, 早期新增了 `launch/slam.launch.py`, 后来整合为功能更完整的 `launch/mapping.launch.py` (包含 Gazebo + SLAM + 键盘遥控 + RViz), 并删除了冗余的 `slam.launch.py`。

**意义**：

- SLAM 从一开始就不再依赖单机默认 frame/topic
- 直接对接当前命名空间化机器人
- `mapping.launch.py` 提供一键建图体验

#### G. 新增 `nav2.rviz`

单机版导航直接用上游：

- `nav2_default_view.rviz`

当前版新增本地：

- `config/nav2.rviz`

并改成订阅相对话题，如：

- `map`
- `scan`
- `global_costmap/costmap`
- `local_costmap/costmap`
- `plan`
- `local_plan`
- `particle_cloud`

以及补齐当前项目需要的默认显示项：

- `Global Costmap`
- `Downsampled Costmap`
- `Local Costmap`
- `Local Plan`
- `Polygon`
- `VoxelGrid`
- `Amcl Particle Swarm`

**意义**：

- 节点在 `main_robot` 命名空间内时，RViz 用相对话题才会自动解析到 `/main_robot/...`
- 如果继续使用官方默认视图里的 `/map`、`/scan` 等全局话题，RViz 会出现“地图没加载/局部规划不显示”的假象

#### H. 新增 `dual_robot_bringup.launch.py` — 双机器人编排

单机版没有多机器人编排能力。当前版新增了专用的双机器人总控 launch:

- 组合两个 `gazebo_sim.launch.py` 实例 (main_robot + follower_robot)
- 第一个实例启动 Gazebo, 第二个使用 `launch_gazebo:=false`
- 通过 `follower_start_delay` (默认 5s) 延迟生成跟随机器人, 避免启动竞争
- 通过 `follower_controller_start_delay` (默认 8s) 延迟启动跟随控制器

#### I. 新增 `main_robot_nav_follow.launch.py` — 统一入口

这是当前系统的**主入口**, 将所有子系统编排在一起:

- 调用 `dual_robot_bringup.launch.py` 生成双机器人
- 调用 `navigation2.launch.py` 启动 Nav2 导航栈
- 启动 `entity_pose_publisher` 和 `follower_controller`
- 支持参数透传: 命名空间、延迟时间、跟随距离、增益等

#### J. `package.xml` 依赖增加

当前版相对于单机版新增依赖：

- `nav2_common` — 用于 `ReplaceString`、`RewrittenYaml`
- `slam_toolbox` — 用于把 SLAM 启动链内置到本包

---

## 4. 逐包文件差异总表

## 4.1 无差异包

- `autopatrol_interfaces`

## 4.2 有差异包

### `autopatrol_robot` (原 `autopartol_robot`)

- 重命名
  - 目录 `autopartol_robot/` → `autopatrol_robot/`

- 新增
  - `autopatrol_robot/entity_pose_publisher.py`
  - `autopatrol_robot/follower_controller.py`

### `laser_merger`

- 修改
  - `laser_merger/laser_merger_node.py`

### `lebot_description`

- 新增
  - `launch/runtime_rviz.launch.py`

- 修改
  - `config/ekf_config.yaml`
  - `config/lebot.rviz`
  - `config/lebot_ros2_controller.yaml`
  - `launch/gazebo_sim.launch.py`
  - `launch/view_model.launch.py`
  - `package.xml`
  - `urdf/lebot/base.urdf.xacro`
  - `urdf/lebot/gazebo_sensor_plugin.xacro`
  - `urdf/lebot/imu.xacro`
  - `urdf/lebot/laser.urdf.xacro`
  - `urdf/lebot/lebot.ros2_control.xacro`
  - `urdf/lebot/lebot.urdf.xacro`
  - `urdf/lebot/wheel.urdf.xacro`

### `lebot_navigation2`

- 新增
  - `config/nav2.rviz`
  - `config/slam_params.yaml`
  - `launch/runtime_localization.launch.py`
  - `launch/runtime_navigation.launch.py`
  - `launch/dual_robot_bringup.launch.py`
  - `launch/main_robot_nav_follow.launch.py`
  - `launch/mapping.launch.py`
  - `maps/room5.pgm` / `room5.yaml`
  - `maps/room6.pgm` / `room6.yaml`

- 删除
  - `launch/slam.launch.py` (功能合并到 `mapping.launch.py`)

- 修改
  - `config/nav2_params.yaml`
  - `launch/navigation2.launch.py`
  - `package.xml`

---

## 5. 本次改动中我对“命名空间”的理解总结

下面这部分不是文件 diff，而是这次从单机版迁移到命名空间版后，真正总结出的规律。

## 5.1 ROS2 命名空间主要影响“节点名、相对话题、相对服务、参数树”

如果一个节点运行在：

- `/main_robot`

那么它内部写相对话题：

- `scan`

最终通常会解析到：

- `/main_robot/scan`

但如果写的是绝对话题：

- `/scan`

那么无论节点在哪个命名空间，都会直接指向全局 `/scan`。

**结论**：

- 想让节点自动跟随机器人命名空间，就尽量用相对话题
- 想强制指向某个唯一来源，就用绝对话题

## 5.2 TF frame 不是 ROS topic，不能指望 namespace 自动帮你补前缀

这是这次最关键的认识之一。

例如：

- `base_link`
- `odom`
- `base_footprint`

这些是 frame 字符串，不会因为节点在 `/main_robot` 下，就自动变成：

- `main_robot/base_link`
- `main_robot/odom`

**结论**：

- frame 名必须显式写成带前缀的字符串
- 或通过 `frame_prefix` / `ReplaceString` / xacro 参数手动生成

## 5.3 TF 话题是否命名空间化，要非常谨慎

在这个项目里，实践结果表明：

- **更稳妥的方案是：TF 话题保持全局 `/tf`、`/tf_static`**
- **真正区分机器人的是 frame 名本身，例如 `main_robot/base_link`**

原因：

- ROS2 里很多组件默认都期待从全局 TF 树读取变换
- 如果把 `/tf` 也 remap 到某个私有命名空间，而系统里其他节点仍在发全局 TF，就会出现“彼此看不见 TF”的问题

这也是当前版不再直接依赖上游 `nav2_bringup` 默认 TF remap 的关键原因。

## 5.4 RViz 必须和实际 namespace 对齐，否则会出现“数据有，画面没有”

这是本次调试里非常典型的一类问题。

上游默认 RViz 配置常写：

- `/map`
- `/scan`
- `/plan`
- `/local_costmap/costmap`

但当前系统真正的数据在：

- `/main_robot/map`
- `/main_robot/scan`
- `/main_robot/plan`
- `/main_robot/local_costmap/costmap`

如果 RViz 不改，就会出现：

- 地图明明加载了，但 RViz 看不到
- 局部规划明明在跑，但显示层没内容

**结论**：

- RViz 也必须做命名空间化
- 最稳妥的方式是：让 RViz 节点本身进入机器人 namespace，并尽量订阅相对话题

## 5.5 嵌套节点内部的相对话题，未必会按你直觉解析

例如 Nav2 的：

- `local_costmap`
- `global_costmap`

它们本身就有更深一级的节点层次。

所以像 obstacle layer 里的：

- `topic: scan`

在复杂命名空间下，可能不会落到你以为的 `/main_robot/scan`。

这也是为什么当前版最终把 costmap 障碍层激光输入明确写成：

- `/<robot_namespace>/scan`

**结论**：

- 普通节点里，相对话题通常很好用
- 但在 Nav2 这种层级深、内部再封装一层的节点里，关键传感器输入最好用明确的绝对目标路径

## 5.6 单机版迁移到命名空间版，必须“全链路一致”

本次改动证明，命名空间不是只改一个 launch 文件就能完成的。

真正要同时对齐的环节包括：

- Gazebo 实体生成
- xacro / URDF / 传感器插件
- `robot_state_publisher`
- `ros2_control`
- `laser_merger`
- EKF
- AMCL
- Nav2 costmap / planner / controller / behavior server
- SLAM
- RViz

只改其中一两层，系统就可能出现各种“半通不通”的问题：

- TF 看不见
- 地图看不见
- 激光进不去 obstacle layer
- 局部规划不显示
- controller manager 找错路径

## 5.7 参数模板化是命名空间迁移的核心手段

当前版里最有效的两个工具是：

- `ReplaceString`
- `RewrittenYaml`

它们解决的是：

- 同一份参数文件如何在不同机器人 namespace 下复用
- 同一份 Nav2 参数如何自动落到 `/main_robot` 这样的根参数树下

**结论**：

- 多机器人/命名空间版最忌讳复制很多份几乎相同的 yaml
- 最好的做法是写模板，再在 launch 时注入 namespace

---

## 6. 与“单机版”相比，本次改造的本质变化

可以把这次迁移概括成一句话：

> **单机版默认依赖全局唯一的 frame、topic、TF、RViz 视图；当前版则把整台机器人改造成了一个以 `main_robot` 为边界的命名空间化系统，但仍保留全局 TF 作为跨组件共享的公共总线。**

换句话说，本次不是简单“改了几个名字”，而是完成了以下架构变化：

- 从“默认全局唯一机器人”
- 迁移到“机器人实例可被 namespace 隔离”
- 并在 TF、RViz、Nav2、Gazebo、EKF、SLAM 之间重新建立一致性

---

## 7. 建议如何使用本说明

如果后续还要继续把别的单机包迁移到命名空间版，建议按下面顺序检查：

1. **先看 frame 名**
   - 是否还在用裸 `base_link` / `odom`

2. **再看话题名**
   - 哪些应该是相对名
   - 哪些必须是绝对名

3. **再看 TF**
   - 是否错误地把 `/tf`、`/tf_static` 私有化了

4. **再看 RViz**
   - 是否仍在订阅单机版全局话题

5. **最后看 Nav2 / SLAM 的 yaml**
   - 是否已做模板化替换

---

## 8. 结论

从源代码对比结果看，两个工作空间的真正功能差异主要集中在三处：

- `laser_merger`：把传感器融合节点改成可随命名空间运行
- `lebot_description`：把机器人模型、Gazebo、控制、EKF、RViz 全部改成命名空间兼容
- `lebot_navigation2`：把 Nav2、SLAM、RViz、参数模板、局部/全局代价地图输入链条改成命名空间兼容

而本次最重要的命名空间经验可以概括为三句话：

- **话题可以依赖 namespace 自动展开，frame 不会。**
- **TF 最好保持全局总线，真正区分机器人的是 frame 名。**
- **命名空间迁移必须全链路一致，尤其是 Gazebo、TF、EKF、Nav2、RViz 之间。**
