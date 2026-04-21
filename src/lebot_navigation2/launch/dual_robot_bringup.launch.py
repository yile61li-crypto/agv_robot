import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    lebot_description_dir = get_package_share_directory('lebot_description')
    gazebo_launch_path = os.path.join(lebot_description_dir, 'launch', 'gazebo_sim.launch.py')
    default_model_path = os.path.join(lebot_description_dir, 'urdf', 'lebot', 'lebot.urdf.xacro')
    default_world_path = os.path.join(lebot_description_dir, 'world', 'custom_room.world')

    model = LaunchConfiguration('model')
    world = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_gazebo = LaunchConfiguration('launch_gazebo')

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

    leader_pose_topic = PythonExpression(["'/", main_robot_namespace, "/follow_target_pose'"])
    follower_cmd_vel_topic = PythonExpression(["'/", follower_robot_namespace, "/cmd_vel'"])

    main_robot_launch = GroupAction(
        scoped=True,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gazebo_launch_path),
                launch_arguments={
                    'model': model,
                    'world': world,
                    'use_sim_time': use_sim_time,
                    'launch_gazebo': launch_gazebo,
                    'robot_namespace': main_robot_namespace,
                    'x_pose': main_x_pose,
                    'y_pose': main_y_pose,
                    'z_pose': main_z_pose,
                    'yaw': main_yaw,
                }.items(),
            )
        ],
    )

    follower_support_nodes = TimerAction(
        period=follower_controller_start_delay,
        actions=[
            Node(
                package='lebot_follower',
                executable='entity_pose_publisher',
                name='leader_pose_publisher',
                output='screen',
                condition=IfCondition(use_follower_controller),
                parameters=[{
                    'entity_name': main_robot_namespace,
                    'entity_state_service': gazebo_state_service,
                    'reference_frame': state_reference_frame,
                    'pose_topic': leader_pose_topic,
                    'publish_rate': leader_pose_publish_rate,
                }],
            ),
            Node(
                package='lebot_follower',
                executable='follower_controller',
                name='follower_controller',
                output='screen',
                condition=IfCondition(use_follower_controller),
                parameters=[{
                    'target_pose_topic': leader_pose_topic,
                    'follower_entity_name': follower_robot_namespace,
                    'entity_state_service': gazebo_state_service,
                    'reference_frame': state_reference_frame,
                    'cmd_vel_topic': follower_cmd_vel_topic,
                    'follow_distance': follow_distance,
                    'control_rate': follower_control_rate,
                    'linear_gain': follower_linear_gain,
                    'angular_gain': follower_angular_gain,
                    'max_linear_speed': follower_max_linear_speed,
                    'max_reverse_speed': follower_max_reverse_speed,
                    'max_angular_speed': follower_max_angular_speed,
                }],
            )
        ],
    )

    follower_robot_launch = TimerAction(
        period=follower_start_delay,
        actions=[
            GroupAction(
                scoped=True,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(gazebo_launch_path),
                        launch_arguments={
                            'model': model,
                            'world': world,
                            'use_sim_time': use_sim_time,
                            'launch_gazebo': 'false',
                            'robot_namespace': follower_robot_namespace,
                            'x_pose': follower_x_pose,
                            'y_pose': follower_y_pose,
                            'z_pose': follower_z_pose,
                            'yaw': follower_yaw,
                        }.items(),
                    )
                ],
            )
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('model', default_value=default_model_path, description='Robot model file path'),
        DeclareLaunchArgument('world', default_value=default_world_path, description='Gazebo world file path'),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation clock'),
        DeclareLaunchArgument('launch_gazebo', default_value='true', description='Whether to launch Gazebo for the first robot instance'),
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
        DeclareLaunchArgument('follower_start_delay', default_value='5.0', description='Delay before launching the follower robot instance'),
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
        DeclareLaunchArgument('gazebo_state_service', default_value='/gazebo_state/get_entity_state', description='Gazebo entity state service used by the follow pipeline'),
        DeclareLaunchArgument('state_reference_frame', default_value='world', description='Reference frame used for Gazebo entity state queries'),
        main_robot_launch,
        follower_robot_launch,
        follower_support_nodes,
    ])
