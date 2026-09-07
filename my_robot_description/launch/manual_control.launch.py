from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Khởi động Driver đọc tay cầm
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen'
    )

    # 2. Khởi động Node gửi lệnh xuống Arduino
    joy_to_serial_node = Node(
        package='my_robot_description',
        executable='cmd_vel_to_serial',
        name='cmd_vel_to_serial_node',
        output='screen'
    )

    return LaunchDescription([
        joy_node,
        joy_to_serial_node
    ])