from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node
import os

def generate_launch_description():
    workspace_dir = os.path.expanduser('~/ros2_ws/src/ares-main/ares-main')

    # Set Gazebo resource path so it can find your models
    ign_resource_path = os.path.join(workspace_dir, 'models') + ":" + \
                        os.path.join(workspace_dir, 'sim_worlds')
    
    set_ign_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=ign_resource_path
    )

    sim_time_param = {'use_sim_time': True}

    # Launch Ignition Gazebo with your world
    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', os.path.join(workspace_dir, 'sim_worlds', 'world.sdf')],
        output='screen'
    )

    # Launch ROS 2 bridge for LiDAR
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[sim_time_param, {
            'config_file': os.path.join(workspace_dir, 'launch', 'bridge_config.yaml')
        }],
        output='screen'
    )

    # Launch your main ROS 2 node
    main_node = Node(
        package='ares-main',
        executable='main_node',
        output='screen'
    )

    return LaunchDescription([
        set_ign_path,
        gazebo,
        bridge,
        main_node
    ])