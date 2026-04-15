from launch import LaunchDescription
from launch_ros.actions import Node
import os 
import yaml


def generate_launch_description():
    #note: changed this path to be relative to launch file -Ollie
    config_path = os.path.join(os.getcwd(), 'config', 'config.yaml')

    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)

    common_params = config.get('common_params', {})
    path_tracking_params = config.get('path_tracking', {})
    
    return LaunchDescription([
        Node(
            package='coordinator',
            executable='coordinator_node',
            namespace=common_params.get('rover_name', 'Dora'),
            output='screen',
            parameters=[common_params]
        ),
        Node(
            package='coordinator',
            executable='position_republish',
            namespace = common_params.get('rover_name', 'Dora'),
            output='screen',
            parameters=[common_params]
        ),
        Node(
            package='searching_and_planning',
            executable='path_planning',
            namespace=common_params.get('rover_name', 'Dora'),
            output='screen',
            parameters=[common_params]
        ),
        #Node(
        #    package='path_tracking',
        #    executable='pathTracking',
        #    namespace=common_params.get('rover_name', 'Dora'),
        #    output='screen',
        #    parameters=[common_params,path_tracking_params]
        #),
        Node(
            package='path_tracking',
            executable='pathTrackingVLA',
            namespace=common_params.get('rover_name', 'Dora'),
            output='screen',
            parameters=[common_params,path_tracking_params]
        ),
        Node(
            package='path_tracking',
            executable='pathInterp',
            namespace=common_params.get('rover_name', 'Dora'),
            output='screen',
            parameters=[common_params]
        ),
    ])