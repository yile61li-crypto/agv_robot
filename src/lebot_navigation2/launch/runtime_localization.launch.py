import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import ReplaceString, RewrittenYaml


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    lifecycle_nodes = ['map_server', 'amcl']
    remappings = [('tf', '/tf'), ('tf_static', '/tf_static')]

    configured_params_file = ReplaceString(
        source_file=params_file,
        replacements={'<robot_namespace>': namespace}
    )

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=configured_params_file,
            root_key=namespace,
            param_rewrites={
                'use_sim_time': use_sim_time,
                'yaml_filename': map_yaml_file,
            },
            convert_types=True,
        ),
        allow_substs=True,
    )

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        DeclareLaunchArgument('namespace', default_value='robot', description='Top-level namespace'),
        DeclareLaunchArgument('map', description='Full path to map yaml file to load'),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation clock'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                get_package_share_directory('lebot_navigation2'),
                'config',
                'nav2_params.yaml',
            ),
            description='Full path to the ROS2 parameters file to use for localization nodes',
        ),
        DeclareLaunchArgument('autostart', default_value='true', description='Automatically startup the nav2 stack'),
        DeclareLaunchArgument('use_respawn', default_value='false', description='Whether to respawn if a node crashes'),
        DeclareLaunchArgument('log_level', default_value='info', description='Log level'),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            namespace=namespace,
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            remappings=remappings,
            arguments=['--ros-args', '--log-level', log_level],
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            namespace=namespace,
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            remappings=remappings,
            arguments=['--ros-args', '--log-level', log_level],
        ),
    ])
