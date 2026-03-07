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

        fig, ax = plt.subplots(figsize=(6, 6))
        ax.plot(xs, ys, marker="o", linewidth=1.5)
        ax.set_title("Planned Trajectory")
        ax.set_xlabel("x")
        ax.set_ylabel("y")
        ax.set_xlim(0.0, 4.6)
        ax.set_ylim(0.0, 4.6)
        ax.set_aspect("equal", adjustable="box")
        ax.grid(True, alpha=0.3)

        output_dir = os.path.join(os.getcwd(), "output")
        os.makedirs(output_dir, exist_ok=True)
        output_path = os.path.join(output_dir, "trajectory.png")
        fig.savefig(output_path, dpi=150, bbox_inches="tight")
        plt.close(fig)

        self.get_logger().info(f"Trajectory plot saved to {output_path}")

def main(args=None):
    rclpy.init(args=args)
    plot_planned_path = PlotPlannedPath()
    rclpy.spin(plot_planned_path)
    plot_planned_path.destroy_node()
    rclpy.shutdown()