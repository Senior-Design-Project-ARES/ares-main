import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path as PathMsg
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

class RoverSimulatorNode(Node):
    def __init__(self):
        super().__init__('rover_simulator_node')
        self.get_logger().info("Rover Simulator Node has been started.")

        

def main(args=None):
    rclpy.init(args=args)

    rover_simulator_node = RoverSimulatorNode()
    rclpy.spin(rover_simulator_node)
    rover_simulator_node.destroy_node()
    rclpy.shutdown()