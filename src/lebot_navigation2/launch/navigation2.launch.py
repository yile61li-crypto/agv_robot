import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # 获取与拼接默认路径
    lebot_navigation2_dir = get_package_share_directory(
        'lebot_navigation2')
    rviz_config_path = os.path.join(lebot_navigation2_dir, 'config', 'nav2.rviz')
    
    # 创建 Launch 配置
    use_sim_time = launch.substitutions.LaunchConfiguration(
        'use_sim_time', default='true')
    namespace = launch.substitutions.LaunchConfiguration(
        'namespace', default='robot')
    use_rviz = launch.substitutions.LaunchConfiguration(
        'use_rviz', default='true')
    map_yaml_path = launch.substitutions.LaunchConfiguration(
        'map', default=os.path.join(lebot_navigation2_dir, 'maps', 'room_05.yaml'))
    nav2_param_path = launch.substitutions.LaunchConfiguration(
        'params_file', default=os.path.join(lebot_navigation2_dir, 'config', 'nav2_params.yaml'))

    return launch.LaunchDescription([
        # 声明新的 Launch 参数
        launch.actions.DeclareLaunchArgument('use_sim_time', default_value=use_sim_time,
                                             description='Use simulation (Gazebo) clock if true'),
        launch.actions.DeclareLaunchArgument('namespace', default_value=namespace,
                                             description='Top-level namespace'),
        launch.actions.DeclareLaunchArgument('use_rviz', default_value=use_rviz,
                                             description='Whether to launch RViz'),
        launch.actions.DeclareLaunchArgument('map', default_value=map_yaml_path,
                                             description='Full path to map file to load'),
        launch.actions.DeclareLaunchArgument('params_file', default_value=nav2_param_path,
                                             description='Full path to param file to load'),

        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [lebot_navigation2_dir, '/launch', '/runtime_localization.launch.py']),
            launch_arguments={
                'namespace': namespace,
                'map': map_yaml_path,
                'use_sim_time': use_sim_time,
                'params_file': nav2_param_path}.items(),
        ),
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [lebot_navigation2_dir, '/launch', '/runtime_navigation.launch.py']),
            launch_arguments={
                'namespace': namespace,
                'use_sim_time': use_sim_time,
                'params_file': nav2_param_path}.items(),
        ),
        # 统一的 lifecycle_manager：按顺序激活定位节点再激活导航节点，消除竞态条件
        launch_ros.actions.Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            namespace=namespace,
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'autostart': True},
                {'node_names': [
                    'map_server',
                    'amcl',
                    'controller_server',
                    'smoother_server',
                    'planner_server',
                    'behavior_server',
                    'bt_navigator',
                    'waypoint_follower',
                    'velocity_smoother',
                ]},
            ],
        ),
        launch.actions.SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'),
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            namespace=namespace,
            arguments=['-d', rviz_config_path],
            condition=launch.conditions.IfCondition(use_rviz),
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])