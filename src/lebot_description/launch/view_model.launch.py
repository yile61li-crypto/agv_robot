import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue  # <--- 1. 新增导入
from launch.substitutions import PythonExpression
from nav2_common.launch import ReplaceString
import os

def generate_launch_description():
    """
    生成 LeBot 机器人的本地可视化启动描述。
    用途：在不启动 Gazebo 仿真的情况下，通过 RViz2 和 GUI 滑块手动测试 URDF 模型和关节运动。
    """

    # --- 1. 获取功能包路径 ---
    pkg_share = FindPackageShare(package='lebot_description').find('lebot_description')

    # --- 2. 定义关键文件路径 ---
    xacro_path = os.path.join(pkg_share, 'urdf/lebot/lebot.urdf.xacro')
    rviz_config_path = os.path.join(pkg_share, 'config/lebot.rviz')
    declare_robot_namespace_arg = DeclareLaunchArgument(
        name='robot_namespace',
        default_value='robot',
        description='机器人命名空间'
    )

    # --- 3. 处理机器人描述 (Xacro -> URDF) ---
    # 使用 Command 调用 xacro 命令行工具解析模型
    link_prefix = PythonExpression(["'", LaunchConfiguration('robot_namespace'), "/'"])
    robot_description_content = Command([
        'xacro ', xacro_path,
        ' robot_namespace:=', LaunchConfiguration('robot_namespace'),
        ' link_prefix:=', link_prefix
    ])
    rviz_config = ReplaceString(
        source_file=rviz_config_path,
        replacements={
            '<robot_namespace>': LaunchConfiguration('robot_namespace'),
            '<robot_tf_prefix>': ''
        }
    )
    
    # <--- 2. 新增：强制转换为字符串，防止被误解析为 YAML
    # 这是解决 "Unable to parse ... as yaml" 错误的关键
    robot_description = ParameterValue(robot_description_content, value_type=str)

    # --- 4. 构建启动描述 ---
    return launch.LaunchDescription([
        declare_robot_namespace_arg,
        
        # 节点 A: 关节状态发布器 (GUI 版)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            namespace=LaunchConfiguration('robot_namespace'),
            output='screen', 
            parameters=[{'use_sim_time': False}]
        ),

        # 节点 B: 机器人状态发布器
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=LaunchConfiguration('robot_namespace'),
            output='screen',
            parameters=[{
                # <--- 3. 修改：使用包裹后的 robot_description 变量
                'robot_description': robot_description, 
                'frame_prefix': '',
                'use_sim_time': False
            }],
            remappings=[('tf', '/tf'), ('tf_static', '/tf_static')]
        ),

        # 节点 C: RViz2 可视化工具
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        )
    ])