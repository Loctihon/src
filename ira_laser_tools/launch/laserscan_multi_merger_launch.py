from launch import LaunchDescription
from launch_ros.actions import Node
import math
import yaml
import os

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the directory of your package
    package_dir = get_package_share_directory('ira_laser_tools')

    # Path to the YAML file
    yaml_file = os.path.join(package_dir, 'config', 'laserscan_multi_merger.yaml')

    # Load the YAML file
    with open(yaml_file, 'r') as file:
        params = yaml.safe_load(file)['laserscan_multi_merger_node']['ros__parameters']

    # Create the Node with parameters loaded from the YAML file
    laserscan_multimerger_cmd = Node(
        package='ira_laser_tools',
        executable='laserscan_multi_merger',
        name='laserscan_multi_merger_node',
        parameters=[params],
    )

    return LaunchDescription([
        laserscan_multimerger_cmd,
    ])
