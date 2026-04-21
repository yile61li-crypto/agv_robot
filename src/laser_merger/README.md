# laser_merger

双激光雷达数据融合节点，将前左、后右两个 180° 激光雷达合并为 360° 全向扫描数据。

> **功能包定位**：这是项目的感知层，解决单雷达视野受限问题，通过融合实现 360° 全向感知，为导航和避障提供完整环境信息。

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

本节点实现多激光雷达数据融合，核心解决以下问题：

### 1. 时间同步

两个雷达数据发布时间不同步，使用 ROS 2 的 `message_filters.ApproximateTimeSynchronizer` 实现软同步。

- **策略**：允许时间容差 `slop_sec`（默认 0.05 秒）
- **实现**：缓存两路数据，当时间戳差小于容差时触发回调
- **优势**：不要求硬件同步，实现简单

### 2. 坐标变换

每个雷达扫描点需要转换到机器人本体坐标系（`base_link`）：

- **输入**：雷达局部坐标系下的极坐标点 (range, angle)
- **变换**：通过 TF 获取雷达到 `base_link` 的变换矩阵
- **输出**：`base_link` 坐标系下的笛卡尔坐标点 (x, y)

### 3. 自遮挡过滤

雷达会检测到机器人本体部分（如轮子、底盘），这些是假障碍需要过滤：

- **过滤区域**：车身矩形范围 `car_length × car_width` + 边距
- **判定条件**：点在车身坐标范围内的被过滤

### 4. 角度分箱合并

将变换后的所有点按角度分箱到固定分辨率网格：

- **分箱**：每个 1° 为一个 bin（可配置）
- **合并**：同一 bin 内的点取最近距离
- **输出**：完整的 360° 激光扫描，可用于导航

## 节点详解

### laser_merger_node

**输入话题：**
| 话题 | 类型 | 说明 |
|------|------|------|
| `/<ns>/scan_front_left` | `sensor_msgs/LaserScan` | 前左雷达原始数据 |
| `/<ns>/scan_back_right` | `sensor_msgs/LaserScan` | 后右雷达原始数据 |

**输出话题：**
| 话题 | 类型 | 说明 |
|------|------|------|
| `/<ns>/scan` | `sensor_msgs/LaserScan` | 融合后的 360° 扫描 |

**TF 变换：**
- 监听 `front_left_link` → `base_link`
- 监听 `back_right_link` → `base_link`

**核心代码流程：**
```python
# 1. 时间同步回调
def merge_scans(self, front_scan, back_scan):
    # 2. TF 变换
    fl_angles, fl_ranges = transform_scan_to_base(front_scan)
    br_angles, br_ranges = transform_scan_to_base(back_scan)
    
    # 3. 自遮挡过滤
    fl_angles, fl_ranges = filter_body_points(fl_angles, fl_ranges)
    br_angles, br_ranges = filter_body_points(br_angles, br_ranges)
    
    # 4. 角度分箱合并
    merged_ranges = bin_and_merge(fl_angles, fl_ranges, 
                                   br_angles, br_ranges)
    
    # 5. 发布
    publish_merged_scan(merged_ranges)
```

## 使用说明

### 自动启动（推荐）

随 `gazebo_sim.launch.py` 自动启动：

```bash
ros2 launch lebot_description gazebo_sim.launch.py
```

启动流程：
- 0s：Gazebo 和机器人启动
- 4s：laser_merger_node 自动启动

### 手动启动（调试）

```bash
ros2 run laser_merger laser_merger_node \
  --ros-args \
  -p front_left_topic:=scan_front_left \
  -p back_right_topic:=scan_back_right \
  -p output_topic:=scan \
  -p base_frame:=base_link \
  -p car_length:=0.30 \
  -p car_width:=0.20
```

### 验证融合效果

```bash
# 查看原始雷达数据
tmux split-window 'ros2 topic echo /robot/scan_front_left'
tmux split-window 'ros2 topic echo /robot/scan_back_right'

# 查看融合后的数据
ros2 topic echo /robot/scan

# RViz 可视化
ros2 launch lebot_description rviz.launch.py
# 在 RViz 中添加 LaserScan 显示，对比前后差异
```

## 参数详解

### 输入输出参数

| 参数名 | 默认值 | 类型 | 说明 |
|--------|--------|------|------|
| `front_left_topic` | `scan_front_left` | string | 前左雷达输入话题 |
| `back_right_topic` | `scan_back_right` | string | 后右雷达输入话题 |
| `output_topic` | `scan` | string | 融合后输出话题 |
| `base_frame` | `base_link` | string | 目标坐标系（TF 变换目标） |

### 车身过滤参数

| 参数名 | 默认值 | 类型 | 说明 |
|--------|--------|------|------|
| `car_length` | `0.30` | double | 车身长度（米），用于自遮挡过滤 |
| `car_width` | `0.20` | double | 车身宽度（米），用于自遮挡过滤 |
| `body_filter_margin_x` | `0.0` | double | X 方向过滤边距（米） |
| `body_filter_margin_y` | `0.0` | double | Y 方向过滤边距（米） |

### 分箱合并参数

| 参数名 | 默认值 | 类型 | 说明 |
|--------|--------|------|------|
| `output_angle_min_deg` | `-180.0` | double | 输出最小角度（度） |
| `output_angle_max_deg` | `180.0` | double | 输出最大角度（度） |
| `output_angle_increment_deg` | `1.0` | double | 输出角度分辨率（度） |
| `range_min` | `0.1` | double | 有效距离下限（米） |
| `range_max` | `10.0` | double | 有效距离上限（米） |

### 时序参数

| 参数名 | 默认值 | 类型 | 说明 |
|--------|--------|------|------|
| `scan_frequency_hz` | `20.0` | double | 输出频率（Hz） |
| `slop_sec` | `0.05` | double | 时间同步容差（秒），两雷达数据时间差小于此值才融合 |

## 算法原理详解

### 坐标系转换流程

```
雷达坐标系 (front_left_link)
    │
    │ 扫描点 (r, θ)
    ▼
┌─────────────────────┐
│ 1. 极坐标 → 笛卡尔  │
│    x = r × cos(θ)   │
│    y = r × sin(θ)   │
└────────┬────────────┘
         │
         ▼
┌─────────────────────┐
│ 2. TF 变换到 base   │
│    通过 TF 获取变换矩阵 T
│    [x', y', 1] = T × [x, y, 1]
└────────┬────────────┘
         │
         ▼
    base_link 坐标系
         │
         ▼
┌─────────────────────┐
│ 3. 笛卡尔 → 极坐标  │
│    r' = √(x'² + y'²)
│    θ' = atan2(y', x')
└─────────────────────┘
```

### 自遮挡过滤算法

```python
def is_body_point(x, y, car_length, car_width, margin):
    """
    判断点是否在车身范围内
    车身中心在原点，长轴沿 X 方向
    """
    half_length = (car_length + margin) / 2
    half_width = (car_width + margin) / 2
    
    return (abs(x) <= half_length and abs(y) <= half_width)

# 过滤
filtered_x = []
filtered_y = []
for x, y in zip(points_x, points_y):
    if not is_body_point(x, y, car_length, car_width, margin):
        filtered_x.append(x)
        filtered_y.append(y)
```

### 角度分箱合并算法

```python
def bin_and_merge(angles, ranges, angle_min, angle_max, angle_increment):
    """
    将点按角度分箱，每个 bin 保留最近距离
    """
    num_bins = int((angle_max - angle_min) / angle_increment)
    merged_ranges = [inf] * num_bins
    
    for angle, range_val in zip(angles, ranges):
        # 计算所属 bin
        bin_idx = int((angle - angle_min) / angle_increment)
        
        if 0 <= bin_idx < num_bins:
            # 保留最近距离
            if range_val < merged_ranges[bin_idx]:
                merged_ranges[bin_idx] = range_val
    
    # 将 inf 替换为 range_max（表示无检测）
    for i in range(num_bins):
        if merged_ranges[i] == inf:
            merged_ranges[i] = range_max
    
    return merged_ranges
```

### 数据融合流程图

```
┌─────────────┐      ┌─────────────┐
│ 前左雷达    │      │ 后右雷达    │
│ 原始扫描    │      │ 原始扫描    │
└──────┬──────┘      └──────┬──────┘
       │                    │
       │ 时间同步           │
       └────────┬───────────┘
                │
                ▼
       ┌──────────────────┐
       │ 极坐标→笛卡尔    │
       │ (r,θ) → (x,y)    │
       └────────┬───────────┘
                │
                ▼
       ┌──────────────────┐
       │ TF 变换到        │
       │ base_link 坐标系 │
       └────────┬───────────┘
                │
                ▼
       ┌──────────────────┐
       │ 自遮挡过滤       │
       │ (去除车身点)     │
       └────────┬───────────┘
                │
                ▼
       ┌──────────────────┐
       │ 角度分箱合并     │
       │ (每角度取最近)   │
       └────────┬───────────┘
                │
                ▼
       ┌──────────────────┐
       │ 发布融合扫描     │
       │ /scan            │
       └──────────────────┘
```

## 依赖

| 依赖包 | 用途 | 安装命令 |
|--------|------|----------|
| `rclpy` | ROS 2 Python 客户端库 | `ros-humble-rclpy` |
| `tf2_ros` | TF 变换库 | `ros-humble-tf2-ros` |
| `sensor_msgs` | 激光扫描消息 | `ros-humble-sensor-msgs` |
| `message_filters` | 时间同步 | `ros-humble-message-filters` |
| `tf2_geometry_msgs` | TF 与消息转换 | `ros-humble-tf2-geometry-msgs` |

## 许可证

Apache-2.0
