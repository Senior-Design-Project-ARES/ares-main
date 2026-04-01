import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path as PathMsg

import csv
import os

class ExportDataNode(Node):
    def __init__(self):
        super().__init__('export_data_node')
        self.get_logger().info("Export Data Node has been started.")

        self.pose_subscription = self.create_subscription(
            PoseStamped, '/current_pose', self._pose_callback, 10)

        self.map_subscription = self.create_subscription(
            OccupancyGrid, '/map', self._map_callback, 10)

        self.cspace_map_subscription = self.create_subscription(
            OccupancyGrid, '/cspace_map', self._cspace_map_callback, 10)
        
        self.path_subscription = self.create_subscription(
            PathMsg,
            '/planned_paths',
            self._path_callback,
            10
        )

        self.output_dir = os.path.join(os.getcwd(), "output")
        os.makedirs(self.output_dir, exist_ok=True)

    def _pose_callback(self, msg):
        with open(os.path.join(self.output_dir, 'current_pose.csv'), 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([
                msg.header.stamp.sec,
                msg.header.stamp.nanosec,
                msg.pose.position.z, # using z for theta, orientation x was used for other purpose
                msg.pose.position.x,
                msg.pose.position.y,
            ])

    def _map_callback(self, msg):
        with open(os.path.join(self.output_dir, 'map_data.csv'), 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            # OccupancyGrid.data is already flattened row-major; cast to list for CSV concatenation.
            writer.writerow([msg.header.stamp.sec, msg.header.stamp.nanosec] + list(msg.data)) 

    def _cspace_map_callback(self, msg):
        with open(os.path.join(self.output_dir, 'cspace_map_data.csv'), 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            # Keep the map flattened in a single CSV row.
            writer.writerow([msg.header.stamp.sec, msg.header.stamp.nanosec] + list(msg.data))

    def _path_callback(self, msg):
        with open(os.path.join(self.output_dir, 'planned_path.csv'), 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            all_data = [msg.header.stamp.sec, msg.header.stamp.nanosec]
            for pose in msg.poses:
                all_data.extend([pose.pose.position.x, pose.pose.position.y])
            writer.writerow(all_data)

def main(args=None):
    rclpy.init(args=args)
    export_data_node = ExportDataNode()
    rclpy.spin(export_data_node)
    export_data_node.destroy_node()
    rclpy.shutdown()