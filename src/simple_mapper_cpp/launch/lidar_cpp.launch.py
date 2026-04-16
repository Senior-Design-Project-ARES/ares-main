from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import yaml
from launch_ros.actions import PushRosNamespace

import os

def generate_launch_description():
    package_dir = get_package_share_directory('simple_mapper_cpp')
    config_path = os.path.join(package_dir, '..', '..', '..', '..', 'config', 'config.yaml')

    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)

    common_params = config.get('common_params', {})
    # lidar_params = config.get('lidar', {})

    rplidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rplidar_ros'),
                'launch',
                'rplidar_a2m12_launch.py'
            )
        )
    )

    mapper = Node(
        package='simple_mapper_cpp',
        executable='occupancy_mapper',
        # namespace=common_params.get('rover_name', 'Dora'),
        name='occupancy_mapper',
        output='screen',
        parameters=[common_params]

    )

    return LaunchDescription([
        GroupAction([
            PushRosNamespace(common_params.get('rover_name', 'Dora')),
            rplidar,
            mapper
        ])
    ])