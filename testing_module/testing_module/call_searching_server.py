import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from cartographer_ros_msgs.srv import TrajectoryQuery

class TestSearchingClient(Node):
    def __init__(self):
        super().__init__('test_searching_client')
        self.client = self.create_client(TrajectoryQuery, 'get_path')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.request = TrajectoryQuery.Request()
        self.request.trajectory_id = 0  # Example trajectory ID

    def send_request(self):
        self.future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        if self.future.result() is not None:
            self.get_logger().info('Service call succeeded')
            # Process the response here (save it as csv)
            response = self.future.result()
            self.plot_trajectory(response)
        else:
            self.get_logger().error('Service call failed %r' % (self.future.exception(),))

    def plot_trajectory(self, response):
        if not hasattr(response, "trajectory") or len(response.trajectory) == 0:
            self.get_logger().warning("No trajectory points in response.")
            return

        xs = [pose.pose.position.x for pose in response.trajectory]
        ys = [pose.pose.position.y for pose in response.trajectory]

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
    test_searching_client = TestSearchingClient()
    test_searching_client.send_request()
    test_searching_client.destroy_node()
    rclpy.shutdown()