import rclpy
from rclpy.node import Node
import tf2_py as tf2

from geometry_msgs.msg import PoseStamped

class PositionRepublish(Node):
    def __init__(self):
        super().__init__('position_republish')
        self.get_logger().info('Starting position republish node...')

        # Parameters
        self.declare_parameter('rover_id', -1)
        self.rover_id = self.get_parameter('rover_id').get_parameter_value().integer_value

        self.position_publisher = self.create_publisher(PoseStamped, '/current_pose', 10)
        self.position_subscription = self.create_subscription(PoseStamped, 'pose', self.position_callback, 10)

    def position_callback(self, msg):
        new_msg = PoseStamped()
        new_msg.header = msg.header
        
        theta = tf2.transformations.euler_from_quaternion([
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ])[2]  # Yaw

        new_msg.pose.position = msg.pose.position
        new_msg.pose.position.z = theta  # Store yaw in z position
        new_msg.pose.orientation.x = self.rover_id  # Store rover ID in orientation w

        self.position_publisher.publish(new_msg)


        

def main(args=None):
    rclpy.init(args=args)
    position_republish = PositionRepublish()
    rclpy.spin(position_republish)
    position_republish.destroy_node()
    rclpy.shutdown()