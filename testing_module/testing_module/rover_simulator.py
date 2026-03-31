import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist

import numpy as np

class RoverSimulatorNode(Node):
    def __init__(self):
        super().__init__('rover_simulator')
        self.get_logger().info("Rover Simulator Node has been started.")
        self.cmd_vel = (0, 0)
        self.pose2d = [0, 0.3, 4.5]  # [theta, x, y]

        self.velocity_subscriber = self.create_subscription(
            Twist,
            'cmd_vel',
            self.velocity_callback,
            10
        )

        self.pose_publisher = self.create_publisher(PoseStamped,'/current_pose',10)

        self.timer = self.create_timer(1/240, self.publish_pose)

    def velocity_callback(self, msg):
        self.cmd_vel = (msg.linear.x, msg.angular.z)

    def publish_pose(self):
        # Update location based on velocity

        self.pose2d[0] += self.cmd_vel[1] * 1/240 #Ang pose
        self.pose2d[1] += self.cmd_vel[0] * 1/240 * np.cos(self.pose2d[0]) #X pose
        self.pose2d[2] += self.cmd_vel[0] * 1/240 * np.sin(self.pose2d[0]) #Y pose

        # Create and publish PoseStamped message
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = self.pose2d[1]
        pose_msg.pose.position.y = self.pose2d[2]
        pose_msg.pose.position.z = self.pose2d[0]  # Using z for theta
        pose_msg.pose.orientation.x = 1.0  # rover ID
        pose_msg.pose.orientation.w = 1.0  # No rotation

        self.pose_publisher.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)

    rover_simulator_node = RoverSimulatorNode()
    rclpy.spin(rover_simulator_node)
    rover_simulator_node.destroy_node()
    rclpy.shutdown()