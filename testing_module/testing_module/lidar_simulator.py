import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid

class LidarSimulatorNode(Node):
    def __init__(self):
        super().__init__('lidar_simulator_node')
        self.get_logger().info("Lidar Simulator Node has been started.")

        

def main(args=None):
    rclpy.init(args=args)

    lidar_simulator_node = LidarSimulatorNode()
    rclpy.spin(lidar_simulator_node)
    lidar_simulator_node.destroy_node()
    rclpy.shutdown()