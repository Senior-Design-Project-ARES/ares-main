import math
import csv
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

# import matplotlib
# matplotlib.use("Agg")
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap, BoundaryNorm

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid

import random

LIDAR_RADIUS = 3.0      # metres  (matches C++ LIDARRADIUS)
ANGLE_RESOLUTION = 0.008  # radians (matches C++ ANGLERESOLUSION)

class LidarSimulatorNode(Node):
    def __init__(self):
        super().__init__('lidar_simulator_node')
        self.get_logger().info("Lidar Simulator Node has been started.")

        self.map_width = 100
        self.map_height = 100
        self.map_resolution = 0.02  # metres per cell (matches C++ MAPRESOLUTION)
        self.map_origin = (0.0, 0.0, 0.0)
        self.map_data = [0 for _ in range(self.map_width * self.map_height)]

        self._load_test_map()

        self.lidar_map = [-1 for _ in range(self.map_width * self.map_height)]
        self.robot_position = (0.0, 0.0)

        self.pose_subscription = self.create_subscription(
            PoseStamped, '/current_pose', self._pose_callback, 10)

        self.lidar_map_publisher = self.create_publisher(OccupancyGrid, '/map', 10)

        self.update_timer = self.create_timer(1/10, self.publish_map)
        # self._check_update_lidar_map()

    def _check_update_lidar_map(self):
        # check update_lidar_map is working by plotting the map using matplotlib
        self.update_lidar_map()
        self.get_logger().info("Lidar Map Updated:")
        self.plot_grid_from_vector(self.lidar_map, self.map_width, self.map_height, title="Lidar Map")

    
    def plot_grid_from_vector(self, data_vec, width=250, height=250, title="Grid Map"):
        # 1) Validate size
        expected = width * height
        if len(data_vec) != expected:
            raise ValueError(f"Expected {expected} values, got {len(data_vec)}")

        # 2) Convert 1D vector -> 2D grid
        grid = np.array(data_vec, dtype=np.int16).reshape((height, width))

        # Optional: if map appears upside-down, use np.flipud(grid)

        # 3) Plot with custom colors: -1 gray, 0 white, 1 black
        cmap = ListedColormap(["gray", "white", "black"])
        norm = BoundaryNorm([-1.5, -0.5, 0.5, 1.5], cmap.N)


        plt.figure(figsize=(7, 7))
        plt.imshow(grid, cmap=cmap, norm=norm, origin="lower")  # origin lower is common for maps
        plt.title(title)
        plt.colorbar(label="Occupancy")
        plt.tight_layout()
        plt.show()


    def publish_map(self):
        self.update_lidar_map()

        # Create and publish OccupancyGrid message
        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        grid_msg.header.frame_id = 'map'
        grid_msg.info.resolution = self.map_resolution
        grid_msg.info.width = self.map_width
        grid_msg.info.height = self.map_height
        grid_msg.info.origin.position.x = self.map_origin[0]
        grid_msg.info.origin.position.y = self.map_origin[1]
        grid_msg.info.origin.position.z = self.map_origin[2]
        grid_msg.data = self.lidar_map

        # self.get_logger().info("Publishing Lidar Map")
        # self.get_logger().info(f"Map data sample: {grid_msg.data[:10]}")

        self.lidar_map_publisher.publish(grid_msg)
        # self._check_update_lidar_map()

    def _pose_callback(self, msg: PoseStamped):
        self.robot_position = (msg.pose.position.x, msg.pose.position.y)

    def update_lidar_map(self):
        """Ray-cast lidar using self.map_data as ground truth.

        Mirrors MyLidarEmulateConstructor::lidarMimicConstruct4point:
          - unknown cells (-1) are discovered each scan
          - already-known cells are skipped
          - obstacles are collected then stamped 100 after all rays
        """
        rx, ry = self.robot_position
        origin_x = self.map_origin[0]
        origin_y = self.map_origin[1]
        res = self.map_resolution

        x_min = origin_x
        x_max = origin_x + self.map_width * res
        y_min = origin_y
        y_max = origin_y + self.map_height * res

        cell_step = res / 2.0   # distance increment along ray (cell_height / 2)

        obstacle_locations = []

        angle = random.uniform(-0.02, 0)
        while angle < 2.0 * math.pi:
            # Rotate unit arm (0, 1) by angle: dx = -sin(a), dy = cos(a)
            dx = -math.sin(angle)
            dy =  math.cos(angle)

            distance = 0.0
            while distance < LIDAR_RADIUS:
                lx = rx + dx * distance
                ly = ry + dy * distance

                if lx <= x_min or lx >= x_max or ly <= y_min or ly >= y_max:
                    distance += cell_step
                    continue

                cx = max(0, min(self.map_width  - 1, int((lx - origin_x) / res)))
                cy = max(0, min(self.map_height - 1, int((ly - origin_y) / res)))
                idx = cy * self.map_width + cx

                # # Skip cells already mapped (equivalent to getState != -1)
                # if self.lidar_map[idx] != -1:
                #     distance += cell_step
                #     continue

                # Truth-map collision check (replaces checkCellCollision)
                if self.map_data[idx] > 0:
                    obstacle_locations.append(((lx, ly), distance))
                    break

                self._update_around_point(lx, ly, 0, distance)
                distance += cell_step

            angle += ANGLE_RESOLUTION

        for (ox, oy), dist in obstacle_locations:
            self._update_around_point(ox, oy, 1, dist)

    def _update_around_point(self, wx: float, wy: float, value: int, radius: float):
        """Set a cell (and a small neighbourhood scaled by radius) to value.

        Never overwrites a cell already marked as 100 (occupied).
        Mirrors MyLidarEmulateConstructor::updateAroundPoint.
        """
        origin_x = self.map_origin[0]
        origin_y = self.map_origin[1]
        res = self.map_resolution

        if (wx <= origin_x or wx >= origin_x + self.map_width  * res or
                wy <= origin_y or wy >= origin_y + self.map_height * res):
            return

        cx = max(0, min(self.map_width  - 1, int((wx - origin_x) / res)))
        cy = max(0, min(self.map_height - 1, int((wy - origin_y) / res)))

        def set_cell(x, y):
            if 0 <= x < self.map_width and 0 <= y < self.map_height:
                i = y * self.map_width + x
                if self.lidar_map[i] != 100:
                    self.lidar_map[i] = value

        set_cell(cx, cy)

        if radius == 0:
            return

        # num_point_to_change mirrors: std::ceil(radius * ANGLERESOLUSION)
        num_point_to_change = math.ceil(radius * ANGLE_RESOLUTION)
        spread = num_point_to_change // 2 + 1
        for i in range(spread):
            for j in range(spread):
                set_cell(cx + i, cy + j)
                set_cell(cx + i, cy - j)
                set_cell(cx - i, cy + j)
                set_cell(cx - i, cy - j)

    def _load_test_map(self):
        pkg_share = Path(get_package_share_directory("testing_module"))
        map_path = pkg_share / "test_map" / "occupancy_250x250_flip.csv"

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

        

def main(args=None):
    rclpy.init(args=args)

    lidar_simulator_node = LidarSimulatorNode()
    rclpy.spin(lidar_simulator_node)
    lidar_simulator_node.destroy_node()
    rclpy.shutdown()