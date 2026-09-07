from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
import os
import xacro
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = FindPackageShare('my_robot_description').find('my_robot_description')
    nav2_bringup_share = FindPackageShare('nav2_bringup').find('nav2_bringup')

    map_file = os.path.join(pkg_share, 'maps', 'my_map.yaml')
    params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )    
    share_dir = get_package_share_directory('my_robot_description')

    rviz_config_file = os.path.join(share_dir, 'config', 'display.rviz')

    xacro_file = os.path.join(share_dir, 'urdf', 'my_robot.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_urdf, 'use_sim_time': use_sim_time}
        ]
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    # controller_manager = Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     parameters=[{'robot_description': robot_urdf},
    #                 controller_params_file]
    # )

    world_file = os.path.join(share_dir, 'worlds', 'my_world.world')

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzserver.launch.py'
            ])
        ]),
        launch_arguments={
            'pause': 'false',
            'world': world_file
        }.items()
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzclient.launch.py'
            ])
        ])
    )

    urdf_spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'my_robot',
            '-topic', 'robot_description'
        ],
        output='screen'
    )

    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
    )

    node_rviz = Node(
        package = 'rviz2',
        executable = 'rviz2',
        name = 'rviz2',
        output = 'screen',
        arguments = ['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}]
        
    )

    slam_toolbox_node = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        PathJoinSubstitution([
            FindPackageShare('slam_toolbox'),
            'launch',
            'online_async_launch.py'
        ])
    ]),
    launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_file,
            'use_sim_time': 'true', # Đang chạy Gazebo nên bắt buộc là true
            'params_file': params_file
        }.items()
    )

    delayed_joint_broadcaster = TimerAction(
        period=4.0,
        actions=[joint_state_broadcaster_spawner]
    )

    delayed_diff_drive = TimerAction(
        period=5.0,
        actions=[diff_drive_controller_spawner]
    )
    
    delayed_nav2 = TimerAction(
        period=7.0,
        actions=[nav2_launch]
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        gazebo_server,
        gazebo_client,
        urdf_spawn_node,
        node_rviz,          
        diff_drive_controller_spawner,         
        # slam_toolbox_node,
        delayed_nav2,
        # delayed_joint_broadcaster,  
        # delayed_diff_drive,
    ])
