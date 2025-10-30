from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os

def generate_launch_description():
    world_path = os.path.join(
        os.getenv('COLCON_PREFIX_PATH').split(':')[0],
        'ares-main/share/ares-main/sim_worlds/world.sdf'
    )

    # Launch Gazebo Fortress (gz sim)
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', world_path],
        output='screen'
    )

    # Bridge the /lidar topic from Gazebo to ROS 2
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'
        ],
        output='screen'
    )

    return LaunchDescription([
        gz_sim,
        bridge
    ])
