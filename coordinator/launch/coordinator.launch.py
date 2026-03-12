from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='coordinator',
            executable='coordinator_node',
            name='coordinator_node',
            output='screen',
            parameters=[
                {'rover_id': 1},
                {'rover_name': 'Marlin'},
            ]
        ),
        Node(
            package='coordinator',
            executable='position_republish',
            name='position_republish',
            output='screen',
            parameters=[
                {'rover_id': 1},
                {'rover_name': 'Marlin'},
            ]
        ),
    ])