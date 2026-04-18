import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path as PathMsg

class PlotPlannedPath(Node):
    def __init__(self):
        super().__init__('plot_planned_path')

        self.path_subscription = self.create_subscription(
            PathMsg,
            'planned_paths',
            self.path_callback,
            10
        )

    def path_callback(self, msg):
        self.plot_trajectory(msg)

    def plot_trajectory(self, response):
        if not hasattr(response, "poses") or len(response.poses) == 0:
            self.get_logger().warning("No trajectory points in response.")
            return

        xs = [pose.pose.position.x for pose in response.poses]
        ys = [pose.pose.position.y for pose in response.poses]

        plt.figure(figsize=(6, 6))
        plt.plot(xs, ys, marker="o", linewidth=1.5)
        plt.title("Planned Trajectory")
        plt.xlabel("x")
        plt.ylabel("y")
        plt.axis("equal")
        plt.grid(True, alpha=0.3)

        output_dir = os.path.join(os.getcwd(), "output")
        os.makedirs(output_dir, exist_ok=True)
        output_path = os.path.join(output_dir, "trajectory.png")
        plt.savefig(output_path, dpi=150, bbox_inches="tight")
        plt.close()

        self.get_logger().info(f"Trajectory plot saved to {output_path}")

def main(args=None):
    rclpy.init(args=args)
    plot_planned_path = PlotPlannedPath()
    rclpy.spin(plot_planned_path)
    plot_planned_path.destroy_node()
    rclpy.shutdown()