import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString


def generate_launch_description():
    lebot_description_dir = get_package_share_directory('lebot_description')
    lebot_navigation2_dir = get_package_share_directory('lebot_navigation2')

    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_namespace = LaunchConfiguration('robot_namespace')
    use_rviz = LaunchConfiguration('use_rviz')
    use_keyboard = LaunchConfiguration('use_keyboard')
    keyboard_terminal_prefix = LaunchConfiguration('keyboard_terminal_prefix')
    slam_params_file = LaunchConfiguration('slam_params_file')

    slam_rviz_config = ReplaceString(
        source_file=os.path.join(lebot_navigation2_dir, 'config', 'slam.rviz'),
        replacements={'<robot_namespace>': robot_namespace}
    )
    configured_slam_params = ReplaceString(
        source_file=slam_params_file,
        replacements={'<robot_namespace>': robot_namespace}
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation clock'),
        DeclareLaunchArgument('robot_namespace', default_value='robot', description='Robot namespace'),
        DeclareLaunchArgument('use_rviz', default_value='true', description='Whether to launch RViz'),
        DeclareLaunchArgument('use_keyboard', default_value='true', description='Whether to launch keyboard teleop in a separate terminal'),
        DeclareLaunchArgument('keyboard_terminal_prefix', default_value='x-terminal-emulator -e', description='Terminal prefix used to launch keyboard teleop'),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=os.path.join(lebot_navigation2_dir, 'config', 'slam_params.yaml'),
            description='Full path to the SLAM parameters file'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(lebot_description_dir, 'launch', 'gazebo_sim.launch.py')
            ),
            launch_arguments={
                'robot_namespace': robot_namespace,
                'model': os.path.join(lebot_description_dir, 'urdf', 'lebot', 'lebot.urdf.xacro'),
            }.items(),
        ),
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[configured_slam_params, {'use_sim_time': use_sim_time}],
        ),
        SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'),
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_twist_keyboard',
            namespace=robot_namespace,
            condition=IfCondition(use_keyboard),
            prefix=keyboard_terminal_prefix,
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            condition=IfCondition(use_rviz),
            arguments=['-d', slam_rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),
    ])
