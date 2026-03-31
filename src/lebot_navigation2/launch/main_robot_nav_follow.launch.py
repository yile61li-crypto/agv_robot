import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    lebot_navigation2_dir = get_package_share_directory('lebot_navigation2')
    lebot_description_dir = get_package_share_directory('lebot_description')

    dual_robot_bringup_launch = os.path.join(lebot_navigation2_dir, 'launch', 'dual_robot_bringup.launch.py')
    navigation_launch = os.path.join(lebot_navigation2_dir, 'launch', 'navigation2.launch.py')
    default_model_path = os.path.join(lebot_description_dir, 'urdf', 'lebot', 'lebot.urdf.xacro')
    default_world_path = os.path.join(lebot_description_dir, 'world', 'custom_room.world')
    default_map_path = os.path.join(lebot_navigation2_dir, 'maps', 'room5.yaml')
    default_nav2_params_path = os.path.join(lebot_navigation2_dir, 'config', 'nav2_params.yaml')

    model = LaunchConfiguration('model')
    world = LaunchConfiguration('world')
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    launch_gazebo = LaunchConfiguration('launch_gazebo')
    navigation_start_delay = LaunchConfiguration('navigation_start_delay')

    main_robot_namespace = LaunchConfiguration('main_robot_namespace')
    main_x_pose = LaunchConfiguration('main_x_pose')
    main_y_pose = LaunchConfiguration('main_y_pose')
    main_z_pose = LaunchConfiguration('main_z_pose')
    main_yaw = LaunchConfiguration('main_yaw')

    follower_robot_namespace = LaunchConfiguration('follower_robot_namespace')
    follower_x_pose = LaunchConfiguration('follower_x_pose')
    follower_y_pose = LaunchConfiguration('follower_y_pose')
    follower_z_pose = LaunchConfiguration('follower_z_pose')
    follower_yaw = LaunchConfiguration('follower_yaw')
    follower_start_delay = LaunchConfiguration('follower_start_delay')

    use_follower_controller = LaunchConfiguration('use_follower_controller')
    follower_controller_start_delay = LaunchConfiguration('follower_controller_start_delay')
    follow_distance = LaunchConfiguration('follow_distance')
    follower_control_rate = LaunchConfiguration('follower_control_rate')
    follower_linear_gain = LaunchConfiguration('follower_linear_gain')
    follower_angular_gain = LaunchConfiguration('follower_angular_gain')
    follower_max_linear_speed = LaunchConfiguration('follower_max_linear_speed')
    follower_max_reverse_speed = LaunchConfiguration('follower_max_reverse_speed')
    follower_max_angular_speed = LaunchConfiguration('follower_max_angular_speed')
    leader_pose_publish_rate = LaunchConfiguration('leader_pose_publish_rate')
    gazebo_state_service = LaunchConfiguration('gazebo_state_service')
    state_reference_frame = LaunchConfiguration('state_reference_frame')

    dual_robot_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(dual_robot_bringup_launch),
        launch_arguments={
            'model': model,
            'world': world,
            'use_sim_time': use_sim_time,
            'launch_gazebo': launch_gazebo,
            'main_robot_namespace': main_robot_namespace,
            'main_x_pose': main_x_pose,
            'main_y_pose': main_y_pose,
            'main_z_pose': main_z_pose,
            'main_yaw': main_yaw,
            'follower_robot_namespace': follower_robot_namespace,
            'follower_x_pose': follower_x_pose,
            'follower_y_pose': follower_y_pose,
            'follower_z_pose': follower_z_pose,
            'follower_yaw': follower_yaw,
            'follower_start_delay': follower_start_delay,
            'use_follower_controller': use_follower_controller,
            'follower_controller_start_delay': follower_controller_start_delay,
            'follow_distance': follow_distance,
            'follower_control_rate': follower_control_rate,
            'follower_linear_gain': follower_linear_gain,
            'follower_angular_gain': follower_angular_gain,
            'follower_max_linear_speed': follower_max_linear_speed,
            'follower_max_reverse_speed': follower_max_reverse_speed,
            'follower_max_angular_speed': follower_max_angular_speed,
            'leader_pose_publish_rate': leader_pose_publish_rate,
            'gazebo_state_service': gazebo_state_service,
            'state_reference_frame': state_reference_frame,
        }.items(),
    )

    main_robot_navigation = TimerAction(
        period=navigation_start_delay,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(navigation_launch),
                launch_arguments={
                    'namespace': main_robot_namespace,
                    'use_sim_time': use_sim_time,
                    'use_rviz': use_rviz,
                    'map': map_yaml,
                    'params_file': params_file,
                }.items(),
            )
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('model', default_value=default_model_path, description='Robot model file path'),
        DeclareLaunchArgument('world', default_value=default_world_path, description='Gazebo world file path'),
        DeclareLaunchArgument('map', default_value=default_map_path, description='Map yaml file for Nav2'),
        DeclareLaunchArgument('params_file', default_value=default_nav2_params_path, description='Nav2 parameters file'),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation clock'),
        DeclareLaunchArgument('use_rviz', default_value='true', description='Whether to launch RViz for the main robot navigation stack'),
        DeclareLaunchArgument('launch_gazebo', default_value='true', description='Whether to launch Gazebo'),
        DeclareLaunchArgument('navigation_start_delay', default_value='6.0', description='Delay before launching the main robot Nav2 stack'),
        DeclareLaunchArgument('main_robot_namespace', default_value='main_robot', description='Main robot namespace'),
        DeclareLaunchArgument('main_x_pose', default_value='0.0', description='Main robot initial X position'),
        DeclareLaunchArgument('main_y_pose', default_value='0.0', description='Main robot initial Y position'),
        DeclareLaunchArgument('main_z_pose', default_value='0.03', description='Main robot initial Z position'),
        DeclareLaunchArgument('main_yaw', default_value='0.0', description='Main robot initial yaw'),
        DeclareLaunchArgument('follower_robot_namespace', default_value='follower_robot', description='Follower robot namespace'),
        DeclareLaunchArgument('follower_x_pose', default_value='-1.0', description='Follower robot initial X position'),
        DeclareLaunchArgument('follower_y_pose', default_value='0.0', description='Follower robot initial Y position'),
        DeclareLaunchArgument('follower_z_pose', default_value='0.03', description='Follower robot initial Z position'),
        DeclareLaunchArgument('follower_yaw', default_value='0.0', description='Follower robot initial yaw'),
        DeclareLaunchArgument('follower_start_delay', default_value='5.0', description='Delay before launching follower robot bringup'),
        DeclareLaunchArgument('use_follower_controller', default_value='true', description='Whether to run the follower controller'),
        DeclareLaunchArgument('follower_controller_start_delay', default_value='8.0', description='Delay before starting follower control'),
        DeclareLaunchArgument('follow_distance', default_value='1.0', description='Desired distance behind the main robot'),
        DeclareLaunchArgument('follower_control_rate', default_value='10.0', description='Follower control frequency in Hz'),
        DeclareLaunchArgument('follower_linear_gain', default_value='0.8', description='Follower linear control gain'),
        DeclareLaunchArgument('follower_angular_gain', default_value='2.0', description='Follower angular control gain'),
        DeclareLaunchArgument('follower_max_linear_speed', default_value='0.5', description='Follower maximum forward speed'),
        DeclareLaunchArgument('follower_max_reverse_speed', default_value='0.3', description='Follower maximum reverse speed'),
        DeclareLaunchArgument('follower_max_angular_speed', default_value='1.8', description='Follower maximum angular speed'),
        DeclareLaunchArgument('leader_pose_publish_rate', default_value='15.0', description='Leader pose publish rate in Hz'),
        DeclareLaunchArgument('gazebo_state_service', default_value='/gazebo_state/get_entity_state', description='Gazebo entity state service used by the follower pipeline'),
        DeclareLaunchArgument('state_reference_frame', default_value='world', description='Reference frame used for Gazebo entity state queries'),
        dual_robot_bringup,
        main_robot_navigation,
    ])
