from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    joy_hardware_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node'
    )

   
    custom_teleop_node = Node(
        package='my_robot_description', 
        executable='custom_joy_teleop.py',
        name='custom_joy_teleop',
        output='screen'
    )

    return LaunchDescription([
        joy_hardware_node,
        custom_teleop_node
    ])