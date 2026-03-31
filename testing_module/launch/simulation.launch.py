from launch import LaunchDescription
from launch_ros.actions import Node
import os
import yaml

def generate_launch_description():
    config_path = os.path.join(os.getcwd(), 'config', 'config.yaml')

    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)

    common_params = config.get('common_params', {})

    return LaunchDescription([
        Node(
            package='testing_module',
            executable='rover_simulation',
            namespace=common_params.get('rover_name', 'Dora'),
            output='screen'
        ),
        Node(
            package='testing_module',
            executable='lidar_simulation',
            namespace=common_params.get('rover_name', 'Dora'),
            output='screen'
        ),
    ])