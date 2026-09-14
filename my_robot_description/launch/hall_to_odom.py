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
    cpp_dir = get_package_share_directory('hall_reader_cpp')
    share_dir = get_package_share_directory('my_robot_description')
    xacro_file = os.path.join(share_dir, 'urdf', 'my_robot.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()
    laser_filter_params = os.path.join(share_dir, 'config', 'laser_filter.yaml')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )    
    hall_reader_node = Node(
        package='hall_reader_cpp',
        executable='arduino_odom_node',  # Sửa lại chỗ này
        name='arduino_odom_node',
        output='screen'
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_urdf, 'use_sim_time': use_sim_time}
        ]
    )

    # joint_state_broadcaster_spawner = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    # )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    # 1. Đường dẫn file cấu hình RViz
    share_dir = get_package_share_directory('my_robot_description')
    rviz_config_file = os.path.join(share_dir, 'config', 'display.rviz')

    # 2. Khai báo Static TF (Vị trí vật lý của LiDAR so với tâm xe)
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_laser',
        arguments=['-0.46', '0.0', '0.15', '0', '0', '0', 'base_link', 'laser_frame']
    )

    laser_filter_node = Node(
    package='laser_filters',
    executable='scan_to_scan_filter_chain',
    name='scan_to_scan_filter_chain',
    parameters=[laser_filter_params],
    output='screen'
    )   

    # 3. Trình điều khiển RPLidar C1
    rplidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 460800, # Baudrate chuẩn của LiDAR C1
            'frame_id': 'laser_frame',
            'inverted': False,
            'angle_compensate': True
        }],
        output='screen'
    )

    # 4. Thuật toán giả lập Odometry từ tia Laser (thay thế cho encoder bánh xe)
    # rf2o_node = Node(
    #     package='rf2o_laser_odometry',
    #     executable='rf2o_laser_odometry_node',
    #     name='rf2o_laser_odometry',
    #     output='screen',
    #     parameters=[{
    #         'laser_scan_topic' : '/scan',
    #         'odom_topic' : '/odom',
    #         'publish_tf' : True,
    #         'base_frame_id' : 'base_footprint',
    #         'odom_frame_id' : 'odom',
    #         'init_pose_from_topic' : '',
    #         'freq' : 20.0
    #     }]
    # )

    # 5. Khởi động SLAM Toolbox (Tắt use_sim_time vì đang chạy thực tế)
    slam_toolbox_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('slam_toolbox'),
                'launch',
                'online_async_launch.py'
            ])
        ]),
        launch_arguments=[('use_sim_time', 'false')]
    )

    # 6. Khởi động giao diện RViz2
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
        static_tf,
        rplidar_node,
        hall_reader_node,
        slam_toolbox_node,
        rviz_node,
    ])