import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    share_dir = get_package_share_directory('my_robot_description')
    
    # 1. Khai báo URDF và Đường dẫn Cấu hình
    xacro_file = os.path.join(share_dir, 'urdf', 'my_robot.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()
    
    rviz_config_file = os.path.join(share_dir, 'config', 'display.rviz')
    slam_params_file = os.path.join(share_dir, 'config', 'mapper_params_online_async.yaml')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )    

    # 2. Node đọc Hall Encoder và phát TF (odom -> base_footprint)
    hall_reader_node = Node(
        package='hall_reader_cpp',
        executable='arduino_odom_node',
        name='arduino_odom_node',
        output='screen'
    )

    # 3. Robot State Publisher & Joint State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_urdf,
            'use_sim_time': use_sim_time
        }]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    # 4. Trình điều khiển RPLidar C1 và Laserscan Merger
    rplidar_1_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_1',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 460800,
            'frame_id': 'laser_1_frame',
            'inverted': False,
            'angle_compensate': True,
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
            'serial_port': '/dev/ttyUSB1',
            'serial_baudrate': 460800,
            'frame_id': 'laser_2_frame',
            'inverted': False,
            'angle_compensate': True,
        }],
        remappings=[('scan', 'scan_2')],
        output='screen'
    )

    scan_merger_node = Node(
        package='ira_laser_tools',
        executable='laserscan_multi_merger',
        name='laserscan_multi_merger',
        parameters=[{
            'use_sim_time': False,
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

    # 5. Khởi động SLAM Toolbox với file YAML tuỳ chỉnh
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

    # 6. Giao diện RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        robot_state_publisher_node,
        joint_state_publisher_node,
        hall_reader_node,
        rplidar_1_node,
        rplidar_2_node,
        scan_merger_node,
        slam_toolbox_node, 
        # rviz_node,
    ])