import csv
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path as PathMsg

class PublishFakeDataNode(Node):
    def __init__(self):
        super().__init__('publish_fake_data_node')
        self.pose_publisher_ = self.create_publisher(PoseStamped, 'current_pose', 10)

        self.path_publisher_ = self.create_publisher(PathMsg, 'planned_paths', 10)

        self.OccupancyGrid_publisher_ = self.create_publisher(OccupancyGrid, 'map', 10)

        self.map_width = 100
        self.map_height = 100
        self.map_resolution = 1.0
        self.map_origin = (0.0, 0.0, 0.0)
        self.map_data = [0 for _ in range(self.map_width * self.map_height)]

        self._load_test_map()

        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def _load_test_map(self):
        pkg_share = Path(get_package_share_directory("testing_module"))
        map_path = pkg_share / "test_map" / "rover_0_iter_3_diskmap.csv"

        if not map_path.exists():
            self.get_logger().warning(f"Test map not found: {map_path}")
            return

        try:
            rows = []
            with map_path.open(newline='') as csvfile:
                reader = csv.reader(csvfile)
                for row in reader:
                    if not row:
                        continue
                    rows.append([int(value) for value in row])

            if not rows:
                self.get_logger().warning(f"Test map is empty: {map_path}")
                return

            width = len(rows[0])
            if any(len(row) != width for row in rows):
                self.get_logger().warning(f"Test map rows are not uniform width: {map_path}")
                return

            self.map_width = width
            self.map_height = len(rows)
            self.map_data = [value for row in rows for value in row]
            self.get_logger().info(
                f"Loaded test map {map_path} ({self.map_width}x{self.map_height})"
            )
        except Exception as exc:
            self.get_logger().warning(f"Failed to load test map {map_path}: {exc}")

    def timer_callback(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = 13.05
        msg.pose.position.y = 4.6
        msg.pose.position.z = 0.0
        msg.pose.orientation.x = -1.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0
        self.pose_publisher_.publish(msg)
        self.get_logger().info('Publishing PoseStamped message')

        # msg = PathMsg()
        # msg.header.stamp = self.get_clock().now().to_msg()
        # msg.header.frame_id = "map"
        # # Add fake poses to the path
        # for i in range(5):
        #     pose = PoseStamped()
        #     pose.header = msg.header
        #     pose.pose.position.x = float(i)
        #     pose.pose.position.y = float(i * 2)
        #     pose.pose.position.z = 0.0
        #     pose.pose.orientation.x = 1.0
        #     pose.pose.orientation.y = 0.0
        #     pose.pose.orientation.z = 0.0
        #     pose.pose.orientation.w = 1.0
        #     msg.poses.append(pose)
        # self.path_publisher_.publish(msg)
        # self.get_logger().info('Publishing Path message')

        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.info.resolution = self.map_resolution
        msg.info.width = self.map_width
        msg.info.height = self.map_height
        msg.info.origin.position.x = self.map_origin[0]
        msg.info.origin.position.y = self.map_origin[1]
        msg.info.origin.position.z = self.map_origin[2]
        msg.info.origin.orientation.x = 0.0
        msg.info.origin.orientation.y = 0.0
        msg.info.origin.orientation.z = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = self.map_data
        self.OccupancyGrid_publisher_.publish(msg)
        self.get_logger().info('Publishing OccupancyGrid message')

def main(args=None):
    rclpy.init(args=args)

    publish_fake_data_node = PublishFakeDataNode()
    rclpy.spin(publish_fake_data_node)
    publish_fake_data_node.destroy_node()
    rclpy.shutdown()