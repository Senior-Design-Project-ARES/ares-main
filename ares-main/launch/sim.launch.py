from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node
import os

def generate_launch_description():
    workspace_dir = os.path.expanduser('~/ares-main/ares-main')

    # 1️⃣ Set Gazebo resource path so it can find your models
    ign_resource_path = os.path.join(workspace_dir, 'models') + ":" + \
                        os.path.join(workspace_dir, 'sim_worlds')
    
    set_ign_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=ign_resource_path
    )

    # 2️⃣ Launch Ignition Gazebo with your world
    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', os.path.join(workspace_dir, 'sim_worlds', 'world.sdf')],
        output='screen'
    )

    # 3️⃣ Launch ROS 2 bridge for LiDAR
    lidar_bridge = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            '/lidar/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked'
        ],
        output='screen'
    )

    # 4️⃣ Launch your main ROS 2 node
    main_node = Node(
        package='ares-main',
        executable='main_node',
        output='screen'
    )

    return LaunchDescription([
        set_ign_path,
        gazebo,
        lidar_bridge,
        main_node
    ])