#!/usr/bin/env python3
"""
hall_odom_mapping.launch.py

Ban day du de chay SLAM thuc te tren robot:
  - Robot State Publisher + Joint State Publisher (TF cho cac link/frame)
  - arduino_odom_node (hall_reader_cpp) => nguon ODOM DUY NHAT (encoder banh xe)
  - 2x RPLidar C1 -> /scan_1 va /scan_2
  - laserscan_multi_merger (ira_laser_tools) -> /scan
  - SLAM Toolbox (online async) -> map
  - RViz2

Luu y: KHONG dung rf2o_laser_odometry trong file nay.
Chi co DUY NHAT arduino_odom_node duoc phep publish TF 'odom' -> 'base_footprint'.
Neu bat ca rf2o va arduino_odom_node cung luc, TF se bi gianh nhau
=> gay loi "Message Filter dropping message" o /scan_1 va /scan_2.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro


def generate_launch_description():
    # ---------------------------------------------------------------- #
    # 0. Duong dan & tham so chung
    # ---------------------------------------------------------------- #
    use_sim_time = LaunchConfiguration('use_sim_time')

    share_dir = get_package_share_directory('my_robot_description')
    xacro_file = os.path.join(share_dir, 'urdf', 'my_robot.xacro')
    rviz_config_file = os.path.join(share_dir, 'config', 'display.rviz')
    slam_params_file = os.path.join(share_dir, 'config', 'mapper_params_online_async.yaml')

    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    # ---------------------------------------------------------------- #
    # 1. Robot model (TF cac frame co dinh: base_footprint/base_link/laser_*_frame)
    # ---------------------------------------------------------------- #
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_urdf, 'use_sim_time': use_sim_time}
        ]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ---------------------------------------------------------------- #
    # 2. NGUON ODOM DUY NHAT: encoder hall qua Arduino Mega
    #    Topic: /odom ; TF: odom -> base_footprint
    # ---------------------------------------------------------------- #
    hall_reader_node = Node(
        package='hall_reader_cpp',
        executable='arduino_odom_node',
        name='arduino_odom_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ---------------------------------------------------------------- #
    # 3. Hai LiDAR RPLidar C1
    # ---------------------------------------------------------------- #
    rplidar_1_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_1',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': '/dev/ttyUSB0',  # LiDAR truoc
            'serial_baudrate': 460800,
            'frame_id': 'laser_1_frame',
            'inverted': False,
            'angle_compensate': True,
            'use_sim_time': use_sim_time,
        }],
        remappings=[('scan', 'scan_1')],
        output='screen'
    )

    rplidar_2_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_2',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': '/dev/ttyUSB1',  # LiDAR sau
            'serial_baudrate': 460800,
            'frame_id': 'laser_2_frame',
            'inverted': False,
            'angle_compensate': True,
            'use_sim_time': use_sim_time,
        }],
        remappings=[('scan', 'scan_2')],
        output='screen'
    )

    # ---------------------------------------------------------------- #
    # 4. Gop 2 LaserScan thanh 1 (/scan_1 + /scan_2 -> /scan)
    # ---------------------------------------------------------------- #
    scan_merger_node = Node(
        package='ira_laser_tools',
        executable='laserscan_multi_merger',
        name='laserscan_multi_merger',
        parameters=[{
            'use_sim_time': use_sim_time,
            'destination_frame': 'base_footprint',
            'cloud_destination_topic': '/merged_cloud',
            'scan_destination_topic': '/scan',
            'laserscan_topics': '/scan_1 /scan_2',
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.0058,
            'scan_time': 0.2,
            'range_min': 0.15,
            'range_max': 20.0
        }],
        output='screen'
    )

    # ---------------------------------------------------------------- #
    # 5. SLAM Toolbox (online async) -> dung /scan + TF odom tu encoder
    # ---------------------------------------------------------------- #
    slam_toolbox_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('slam_toolbox'),
                'launch',
                'online_async_launch.py'
            ])
        ]),
        launch_arguments=[
            ('use_sim_time', use_sim_time),
            ('slam_params_file', slam_params_file)
        ]
    )

    # ---------------------------------------------------------------- #
    # 6. RViz2
    # ---------------------------------------------------------------- #
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # ---------------------------------------------------------------- #
    return LaunchDescription([
        declare_use_sim_time_cmd,
        robot_state_publisher_node,
        joint_state_publisher_node,
        hall_reader_node,      # odom duy nhat (encoder banh xe)
        rplidar_1_node,
        rplidar_2_node,
        scan_merger_node,
        slam_toolbox_node,
        rviz_node,
    ])
