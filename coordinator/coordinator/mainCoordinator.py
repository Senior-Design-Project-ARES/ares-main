import rclpy
from rclpy.node import Node

from cartographer_ros_msgs.srv import TrajectoryQuery
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as PathMsg
from std_msgs.msg import Bool

from enum import Enum

class State(Enum):
    IDLE = 0
    PLANNING_FOR_TARGET = 1
    NORMAL_PLANNING = 2
    STOPPED = 3

class MainCoordinator(Node):
    def __init__(self):
        super().__init__('main_coordinator')

        # Parameters
        self.declare_parameter('rover_id', -1)
        self.rover_id = self.get_parameter('rover_id').get_parameter_value().integer_value

        self.declare_parameter('replanning_threshold', 0.3)
        self.replanning_threshold = self.get_parameter('replanning_threshold').get_parameter_value().double_value

        # State variables
        self.should_exit = False
        self.path2target_planned = False
        self.state = State.IDLE

        # Flags to control when to request new paths
        self.need_path_to_target = False
        self.need_new_path = False
        self.target_found = False
        self.target_path_planned = False

        # Store the end of current path
        self.path_end = [None] * 2

        # Store target location
        self.target_location = [None] * 2

        # Create publisher and subscribers
        self.path_publisher = self.create_publisher(PathMsg, 'planned_paths', 10)
        self.stop_publisher = self.create_publisher(Bool, 'stop_rover', 10)

        self.target_subscription = self.create_subscription(
            PoseStamped,
            'target_location',
            self.target_callback,
            10
        )

        self.pose_subscription = self.create_subscription(
            PoseStamped,
            'current_pose',
            self.pose_callback,
            10
        )

        self.other_rovers_path_subscription = self.create_subscription(
            PathMsg,
            'planned_paths',
            self.other_rovers_path_callback,
            10
        )


        # Create service client for trajectory planning
        self.planning_client = self.create_client(TrajectoryQuery, 'get_path')
        while not self.planning_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for path planning service...')
        self.pending_future = None

        # create timers for periodic planning for target
        self.target_plan_timer = self.create_timer(5.0, self.timer_target_callback)

        # create main palnning timer
        self.timer = self.create_timer(0.1, self.main_loop)

    def main_loop(self):
        if self.state != State.IDLE:
            return

        if self.need_new_path:
            self.request_trajectory(trajectory_id=0)
            self.need_new_path = False
            self.need_path_to_target = False
            self.state = State.NORMAL_PLANNING
            return
        
        if self.need_path_to_target:
            self.request_trajectory(trajectory_id=1)
            self.need_path_to_target = False
            self.state = State.PLANNING_FOR_TARGET
            return
        
    def target_callback(self, msg):
        self.target_found = True
        self.target_location[0] = msg.pose.position.x
        self.target_location[1] = msg.pose.position.y

    def timer_target_callback(self):
        if self.target_path_planned:
            return

        if self.target_found:
            self.need_path_to_target = True
            return
        
    def other_rovers_path_callback(self, msg):
        if (not self.target_found):
            return

        if (msg.poses[0].pose.orientation.x == self.rover_id):
            return
        
        # if (msg.poses[-1].pose.position.x == self.path_end[0] and msg.poses[-1].pose.position.y == self.path_end[1]):
            
        end_location = [msg.poses[-1].pose.position.x, msg.poses[-1].pose.position.y]
        if ((end_location[0]-self.target_location[0])**2 + (end_location[1]-self.target_location[1])**2) < 0.05**2:
            self.state = State.STOPPED
            stop_msg = Bool()
            stop_msg.data = True
            self.stop_publisher.publish(stop_msg)
        
    def pose_callback(self, msg):
        if (msg.pose.orientation.x != self.rover_id):
            return
        
        if (self.path_end[0] is None or self.path_end[1] is None):
            return
        
        pose = [msg.pose.position.x, msg.pose.position.y]
        if (pose[0]-self.path_end[0])**2 + (pose[1]-self.path_end[1])**2 < self.replanning_threshold**2:
            self.need_new_path = True


    def planning_response_callback(self, future):
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f'TrajectoryQuery call failed: {exc}')
            return

        if response is None:
            self.get_logger().error('TrajectoryQuery returned no response.')
            return

        path_msg = PathMsg()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "world"

        for node in response.trajectory:
            pose_stamped = PoseStamped()
            pose_stamped.header = path_msg.header
            pose_stamped.pose = node.pose
            path_msg.poses.append(pose_stamped)

        self.path_publisher.publish(path_msg)
        self.path_end[0] = path_msg.poses[-1].pose.position.x
        self.path_end[1] = path_msg.poses[-1].pose.position.y

        self.state = State.IDLE

    def request_trajectory(self, trajectory_id):
        if self.pending_future and not self.pending_future.done():
            return
        
        request = TrajectoryQuery.Request()
        request.trajectory_id = trajectory_id
        self.pending_future = self.planning_client.call_async(request)
        self.pending_future.add_done_callback(self.planning_response_callback)

def main(args=None):
    rclpy.init(args=args)
    main_coordinator = MainCoordinator()
    try:
        while rclpy.ok() and not main_coordinator.should_exit:
            rclpy.spin_once(main_coordinator, timeout_sec=0.1)
    finally:
        main_coordinator.destroy_node()
        rclpy.shutdown()