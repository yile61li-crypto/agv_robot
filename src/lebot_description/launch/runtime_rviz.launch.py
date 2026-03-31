import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import ReplaceString


def generate_launch_description():
    pkg_share = get_package_share_directory('lebot_description')
    xacro_path = os.path.join(pkg_share, 'urdf/lebot/lebot.urdf.xacro')
    rviz_config_path = os.path.join(pkg_share, 'config/lebot.rviz')

    robot_namespace = LaunchConfiguration('robot_namespace')
    declare_robot_namespace_arg = DeclareLaunchArgument(
        'robot_namespace',
        default_value='robot',
        description='机器人命名空间'
    )
    declare_publish_robot_description_arg = DeclareLaunchArgument(
        'publish_robot_description',
        default_value='false',
        description='是否同时启动 RViz 专用 robot_description 发布器'
    )

    link_prefix = PythonExpression(["'", robot_namespace, "/'"])
    robot_description_content = Command([
        'xacro ', xacro_path,
        ' robot_namespace:=', robot_namespace,
        ' link_prefix:=', link_prefix
    ])
    robot_description = ParameterValue(robot_description_content, value_type=str)

    rviz_config = ReplaceString(
        source_file=rviz_config_path,
        replacements={
            '<robot_namespace>': robot_namespace,
            '<robot_tf_prefix>': ''
        }
    )

    robot_description_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='rviz_robot_description_publisher',
        namespace=robot_namespace,
        condition=IfCondition(LaunchConfiguration('publish_robot_description')),
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'frame_prefix': '',
            'use_sim_time': True,
        }],
        remappings=[
            ('tf', 'rviz_robot_description_tf'),
            ('tf_static', 'rviz_robot_description_tf_static'),
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        declare_robot_namespace_arg,
        declare_publish_robot_description_arg,
        robot_description_publisher,
        rviz_node,
    ])
