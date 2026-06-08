# ARES 2D LIDAR Mapping Node

## Requirements

* ROS 2 Humble
* `rplidar_ros` package by Slamtec
* RPLIDAR A2M12 or compatible 2D LIDAR
* External localization system

  * In our case, VRPN motion tracking is used

## Introduction

This package provides a ROS 2 node for 2D LIDAR-based mapping using the RPLIDAR A2M12. The node uses LIDAR scan data together with an external high-accuracy localization system to generate an occupancy grid map.

This package does **not** perform localization by itself. It requires the robot pose to be provided by another system, such as VRPN motion tracking. The pose topic subscribed to by the node is `[namespace]/pose`, the generated map is published as a `nav_msgs/OccupancyGrid` message on `/map`.

## How to Use

After building the package, launch the mapping node using:

```bash
ros2 launch simple_mapper_cpp lidar_cpp.launch.py
```

After launching, the node will:

1. Subscribe to the LIDAR scan data.
2. Subscribe to the robot pose data from the localization system.
3. Use the pose and scan data to build a 2D occupancy grid map.
4. Publish the generated map on the `/map` topic.

## Subscribed Topics

| Topic               | Message Type                          | Description                           |
| ------------------- | ------------------------------------- | ------------------------------------- |
| `/scan`             | `sensor_msgs/LaserScan`               | 2D LIDAR scan data                    |
| `/[namespace]/pose` | Pose message from localization system | Robot pose from external localization |

## Published Topics

| Topic  | Message Type             | Description                     |
| ------ | ------------------------ | ------------------------------- |
| `/map` | `nav_msgs/OccupancyGrid` | Generated 2D occupancy grid map |

## Notes

* The mapping accuracy depends heavily on the accuracy of the external localization system.
* The node assumes that the LIDAR data and robot pose data are available while the node is running.
* This package was developed for the ARES rover system using VRPN tracking for localization.
