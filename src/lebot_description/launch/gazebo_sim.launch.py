import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import ReplaceString
import os

def create_controller_event_handlers(context, robot_namespace, spawn_entity_node):
    resolved_robot_namespace = context.perform_substitution(robot_namespace)
    controller_manager_name = f'/{resolved_robot_namespace}/controller_manager'

    load_joint_state_controller = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             '--controller-manager', controller_manager_name,
             'lebot_joint_state_broadcaster'],
        output='screen'
    )

    load_diff_drive_controller = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             '--controller-manager', controller_manager_name,
             'lebot_diff_drive_controller'],
        output='screen'
    )

    return [
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity_node,
                on_exit=[load_joint_state_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_diff_drive_controller],
            )
        ),
    ]

def generate_launch_description():
    """
    生成 LeBot 机器人的 Gazebo 仿真启动描述。
    流程：加载模型 -> 启动 Gazebo -> 生成机器人 -> 链式加载控制器 -> 启动传感器融合节点
    """
    
    # --- 1. 路径配置与常量定义 ---
    robot_name = "lebot"
    # 获取 lebot_description 功能包的共享目录路径
    pkg_lebot_desc = get_package_share_directory('lebot_description')
    
    # 构建默认的文件路径
    default_model_path = os.path.join(pkg_lebot_desc, 'urdf/lebot/lebot.urdf.xacro')
    default_world_path = os.path.join(pkg_lebot_desc, 'world/custom_room.world')
    model = LaunchConfiguration('model')
    robot_namespace = LaunchConfiguration('robot_namespace')
    world = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_gazebo_flag = LaunchConfiguration('launch_gazebo')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')
    yaw = LaunchConfiguration('yaw')

    # --- 2. 声明启动参数 (Launch Arguments) ---
    # 允许用户通过命令行修改模型路径，例如: ros2 launch pkg file.py model:=/path/to/other.xacro
    declare_model_arg = DeclareLaunchArgument(
        name='model', 
        default_value=default_model_path,
        description='机器人模型文件的路径 (支持 .urdf 或 .xacro)'
    )
    declare_robot_namespace_arg = DeclareLaunchArgument(
        name='robot_namespace',
        default_value='robot',
        description='机器人命名空间'
    )
    declare_world_arg = DeclareLaunchArgument(
        name='world',
        default_value=default_world_path,
        description='Gazebo 世界文件路径'
    )
    declare_use_sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='是否使用仿真时间'
    )
    declare_launch_gazebo_arg = DeclareLaunchArgument(
        name='launch_gazebo',
        default_value='true',
        description='是否启动 Gazebo 世界'
    )
    declare_x_pose_arg = DeclareLaunchArgument(
        name='x_pose',
        default_value='0.0',
        description='机器人初始 X 坐标'
    )
    declare_y_pose_arg = DeclareLaunchArgument(
        name='y_pose',
        default_value='0.0',
        description='机器人初始 Y 坐标'
    )
    declare_z_pose_arg = DeclareLaunchArgument(
        name='z_pose',
        default_value='0.03',
        description='机器人初始 Z 坐标'
    )
    declare_yaw_arg = DeclareLaunchArgument(
        name='yaw',
        default_value='0.0',
        description='机器人初始偏航角'
    )

    # --- 3. 处理机器人描述 (Xacro 解析) ---
    # 使用 Command 调用 xacro 命令行工具，将 .xacro 宏文件解析为完整的 URDF 字符串
    # ParameterValue 确保该值被正确识别为字符串类型的 ROS 参数
    robot_description_content = ParameterValue(
        Command([
            'xacro ', model,
            ' robot_namespace:=', robot_namespace
        ]),
        value_type=str
    )
    rviz_link_prefix = PythonExpression(["'", robot_namespace, "/'"])
    rviz_robot_description_content = ParameterValue(
        Command([
            'xacro ', model,
            ' robot_namespace:=', robot_namespace,
            ' link_prefix:=', rviz_link_prefix
        ]),
        value_type=str
    )
    frame_prefix = PythonExpression(["'", robot_namespace, "/'"])
    odom_frame = PythonExpression(["'", robot_namespace, "/odom'"])
    base_link_frame = PythonExpression(["'", robot_namespace, "/base_link'"])
    ekf_config = ReplaceString(
        source_file=os.path.join(pkg_lebot_desc, 'config', 'ekf_config.yaml'),
        replacements={'<robot_namespace>': robot_namespace}
    )

    # --- 4. 启动 Robot State Publisher 节点 ---
    # 该节点订阅 joint_states 并发布 /tf 变换，是机器人运动学解算的核心
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=robot_namespace,
        parameters=[{
            'robot_description': robot_description_content, 
            'frame_prefix': frame_prefix,
            'use_sim_time': use_sim_time  # 重要：告诉节点使用 Gazebo 的仿真时间而非系统时间
        }],
        remappings=[
            ('robot_description', 'spawn_robot_description'),
            ('tf', '/tf'),
            ('tf_static', '/tf_static')
        ],
        output='screen'
    )
    rviz_robot_description_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='rviz_robot_description_publisher',
        namespace=robot_namespace,
        parameters=[{
            'robot_description': rviz_robot_description_content,
            'frame_prefix': '',
            'use_sim_time': use_sim_time
        }],
        remappings=[('tf', 'rviz_robot_description_tf'), ('tf_static', 'rviz_robot_description_tf_static')],
        output='screen'
    )

    # --- 5. 启动 Gazebo 仿真环境 ---
    # 包含官方的 gazebo_ros 启动文件，并注入自定义参数
    launch_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('gazebo_ros'), '/launch/gazebo.launch.py'
        ]),
        condition=IfCondition(launch_gazebo_flag),
        launch_arguments={
            'world': world,                   # 加载自定义地图
            'verbose': 'true',                # 输出详细日志以便调试
            'max_step_size': '0.001',         # 物理引擎步长 1ms，提高仿真精度
            'real_time_factor': '1.0',        # 尝试保持 1:1 实时仿真
            'real_time_update_rate': '1000'   # 目标更新频率 1000Hz
        }.items()
    )

    # --- 6. 在 Gazebo 中生成机器人实体 ---
    # 读取 /robot_description 参数，将模型实例化到仿真世界中
    spawn_entity_node = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        namespace=robot_namespace,
        arguments=[
            '-topic', 'spawn_robot_description',
            '-entity', robot_namespace,
            '-robot_namespace', robot_namespace,
            '-x', x_pose, '-y', y_pose, '-z', z_pose, '-Y', yaw
        ],
        output='screen'
    )
    controller_event_handlers = OpaqueFunction(
        function=lambda context: create_controller_event_handlers(
            context,
            robot_namespace,
            spawn_entity_node,
        )
    )

    # --- 8. 启动激光雷达数据融合节点 ---
    # 将多个激光雷达的数据合并为一个 360 度的扫描数据，用于 SLAM 或导航
    laser_merger_node = launch_ros.actions.Node(
        package='laser_merger',
        executable='laser_merger_node',
        name='laser_merger',
        parameters=[{
            'use_sim_time': use_sim_time,
            'output_angle_min_deg': -180.0,   # 输出范围：-180 度
            'output_angle_max_deg': 180.0,    # 输出范围：+180 度 (全覆盖)
            'front_left_topic': 'scan_front_left',  # 输入话题 1
            'back_right_topic': 'scan_back_right',  # 输入话题 2
            'output_topic': 'scan',          # 合并后的输出话题
            'car_length': 0.30,               # 车身长度 (与URDF一致)
            'car_width': 0.20,                # 车身宽度 (与URDF一致)
            'body_filter_margin_x': 0.05,
            'body_filter_margin_y': 0.04,
            'scan_frequency_hz': 20.0,        # 输出频率
            'slop_sec': 0.03                  # 时间同步容差 (秒)
        }],
        namespace=robot_namespace,
        output='screen'
    )

    # 启动 EKF 融合节点
    ekf_node = launch_ros.actions.Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_config,
            {
                'use_sim_time': use_sim_time,
                'odom_frame': odom_frame,
                'base_link_frame': base_link_frame,
                'world_frame': odom_frame
            }
        ],
        namespace=robot_namespace,
        remappings=[
            ('tf', '/tf'),
            ('tf_static', '/tf_static'),
            ('odometry/filtered', 'odom'),
        ]
    )

    # --- 9. 定义启动顺序与事件回调 ---
    return launch.LaunchDescription([
        # 1. 首先声明参数
        declare_model_arg,
        declare_robot_namespace_arg,
        declare_world_arg,
        declare_use_sim_time_arg,
        declare_launch_gazebo_arg,
        declare_x_pose_arg,
        declare_y_pose_arg,
        declare_z_pose_arg,
        declare_yaw_arg,
        
        # 2. 启动 Gazebo 服务器 (后台运行)
        launch_gazebo,
        
        # 3. 启动状态发布节点 (一旦 Gazebo 启动即可运行，准备接收描述)
        robot_state_publisher_node,
        rviz_robot_description_publisher_node,
        
        # 4. 生成机器人实体
        spawn_entity_node,
        
        # 5. 事件驱动：生成机器人后按顺序加载控制器
        controller_event_handlers,
        
        # 7. 启动激光融合节点 (可独立启动，不依赖控制器加载完成，但依赖仿真时间)
        laser_merger_node,
        ekf_node,
    ])