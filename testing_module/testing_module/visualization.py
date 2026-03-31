import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path as PathMsg

import csv
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap, BoundaryNorm

class VisualizationNode(Node):
    def __init__(self):
        super().__init__('visualization_node')
        self.get_logger().info("Visualization Node has been started.")

        self.pose2d = [0, 0, 0]  # [theta, x, y]
        
        self.map_width = 100
        self.map_height = 100
        self.map_resolution = 0.02
        self.map_origin = (0.0, 0.0, 0.0)
        self.map_data = [0 for _ in range(self.map_width * self.map_height)]

        self._load_test_map()

        self.lidar_map_data = [-1 for _ in range(self.map_width * self.map_height)]
        self.lidar_map_width = self.map_width
        self.lidar_map_height = self.map_height
        self.lidar_map_resolution = self.map_resolution
        self.lidar_map_origin = self.map_origin

        self.cspace_map_data = [-1 for _ in range(self.map_width * self.map_height)]

        self.cmap = ListedColormap(["gray", "white", "black"])
        self.norm = BoundaryNorm([-1.5, -0.5, 0.5, 1.5], self.cmap.N)

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
        
        plt.ion()
        self.truth_map_fig, self.truth_map_ax, self.truth_map_im = self.create_map_plot(
            self.map_data,
            self.map_width,
            self.map_height,
            self.map_origin,
            self.map_resolution,
            "Truth Map",
        )
        self.lidar_map_fig, self.lidar_map_ax, self.lidar_map_im = self.create_map_plot(
            self.lidar_map_data,
            self.lidar_map_width,
            self.lidar_map_height,
            self.lidar_map_origin,
            self.lidar_map_resolution,
            "Lidar Map",
        )
        self.cspace_map_fig, self.cspace_map_ax, self.cspace_map_im = self.create_map_plot(
            self.cspace_map_data,
            self.map_width,
            self.map_height,
            self.map_origin,
            self.map_resolution,
            "C-Space Map",
        )

        # Demo overlay points: one marker on each map that updates with current pose.
        self.truth_pose_marker, = self.truth_map_ax.plot([0], [0], "ro", markersize=7, label="Pose")
        self.lidar_pose_marker, = self.lidar_map_ax.plot([0], [0], "ro", markersize=7, label="Pose")
        self.truth_map_ax.legend()
        self.lidar_map_ax.legend()

        # show path marker on truth map
        self.truth_path_marker, = self.truth_map_ax.plot([], [], "-o", linewidth=2, label="Planned Path")
        self.lidar_path_marker, = self.lidar_map_ax.plot([], [], "-o", linewidth=2, label="Planned Path")
        self.truth_map_ax.legend()
        self.lidar_map_ax.legend()

        plt.show(block=False)
        self.truth_map_fig.canvas.draw()
        self.truth_map_fig.canvas.flush_events()
        self.lidar_map_fig.canvas.draw()
        self.lidar_map_fig.canvas.flush_events()
        self.cspace_map_fig.canvas.draw()
        self.cspace_map_fig.canvas.flush_events()

        # self.overlay_timer = self.create_timer(0.2, self.update_pose_overlays)

    def _pose_callback(self, msg):
        # self.get_logger().info(f"Received Pose: {msg.pose.position.x}, {msg.pose.position.y}, {msg.pose.position.z}")
        self.pose2d[1] = msg.pose.position.x
        self.pose2d[2] = msg.pose.position.y
        self.pose2d[0] = msg.pose.position.z

        self.update_pose_overlays()

    def _map_callback(self, msg):
        # self.get_logger().info(f"Received Map: width={msg.info.width}, height={msg.info.height}, resolution={msg.info.resolution}")
        self.lidar_map_width = msg.info.width
        self.lidar_map_height = msg.info.height
        self.lidar_map_resolution = msg.info.resolution
        self.lidar_map_origin = (
            msg.info.origin.position.x,
            msg.info.origin.position.y,
            msg.info.origin.position.z,
        )
        self.lidar_map_data = list(msg.data)

        self.update_lidar_map_plot()

    def _cspace_map_callback(self, msg):
        grid = np.array(msg.data, dtype=np.int16).reshape((msg.info.height, msg.info.width))
        self.cspace_map_im.set_data(grid)
        self.cspace_map_fig.canvas.draw_idle()
        self.cspace_map_fig.canvas.flush_events()

    def _path_callback(self, msg):
        self.get_logger().info(f"Received planned path with {len(msg.poses)} poses.")
        
        xs = [pose.pose.position.x for pose in msg.poses]
        ys = [pose.pose.position.y for pose in msg.poses]

        self.truth_path_marker.set_data(xs, ys)
        self.lidar_path_marker.set_data(xs, ys)

        self.truth_map_fig.canvas.draw_idle()
        self.truth_map_fig.canvas.flush_events()
        self.lidar_map_fig.canvas.draw_idle()
        self.lidar_map_fig.canvas.flush_events()



    def create_map_plot(self, data_vec, width, height, origin, resolution, title="Map"):
        expected = width * height
        if len(data_vec) != expected:
            raise ValueError(f"Expected {expected} values, got {len(data_vec)}")
        
        grid = np.array(data_vec, dtype=np.int16).reshape((height, width))
        x_min = origin[0]
        x_max = origin[0] + width * resolution
        y_min = origin[1]
        y_max = origin[1] + height * resolution

        fig, ax = plt.subplots(figsize=(15, 15))
        im = ax.imshow(
            grid,
            cmap=self.cmap,
            norm=self.norm,
            origin='lower',
            extent=[x_min, x_max, y_min, y_max],
        )
        ax.set_title(title)
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        ax.set_aspect("equal", adjustable="box")

        return fig, ax, im
    
    def update_lidar_map_plot(self):
        expected = self.lidar_map_width * self.lidar_map_height
        if len(self.lidar_map_data) != expected:
            self.get_logger().warning(f"Expected {expected} values for lidar map, got {len(self.lidar_map_data)}")
            return
        
        grid = np.array(self.lidar_map_data, dtype=np.int16).reshape((self.lidar_map_height, self.lidar_map_width))
        self.lidar_map_im.set_data(grid)
        x_min = self.lidar_map_origin[0]
        x_max = self.lidar_map_origin[0] + self.lidar_map_width * self.lidar_map_resolution
        y_min = self.lidar_map_origin[1]
        y_max = self.lidar_map_origin[1] + self.lidar_map_height * self.lidar_map_resolution
        self.lidar_map_im.set_extent([x_min, x_max, y_min, y_max])

        self.lidar_map_fig.canvas.draw_idle()
        self.lidar_map_fig.canvas.flush_events()

    def update_pose_overlays(self):
        x_world = self.pose2d[1]
        y_world = self.pose2d[2]

        self.truth_pose_marker.set_data([x_world], [y_world])
        self.lidar_pose_marker.set_data([x_world], [y_world])

        self.truth_map_fig.canvas.draw_idle()
        self.truth_map_fig.canvas.flush_events()
        self.lidar_map_fig.canvas.draw_idle()
        self.lidar_map_fig.canvas.flush_events()

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

    visualization_node = VisualizationNode()
    rclpy.spin(visualization_node)
    visualization_node.destroy_node()
    plt.close('all')
    rclpy.shutdown()
