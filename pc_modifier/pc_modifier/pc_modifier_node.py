import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
import struct
import numpy as np

def add_time_field_to_pointcloud2(cloud_msg: PointCloud2, logger, scan_duration=0.1) -> PointCloud2:
    """
    Adds a 'time' field to a PointCloud2 message by directly manipulating the binary data.
    This approach is more robust than using ros2_numpy conversions.
    
    Args:
        cloud_msg: Input PointCloud2 message
        logger: ROS 2 logger
        scan_duration: Duration of one scan in seconds (for time field generation)
    
    Returns:
        New PointCloud2 message with 'time' field added
    """
    try:
        # Check if 'time' field already exists
        field_names = [field.name for field in cloud_msg.fields]
        if 'time' in field_names:
            logger.info("Point cloud already has 'time' field")
            return cloud_msg
        
        logger.info(f"Input fields: {field_names}")
        logger.info(f"Point cloud: {cloud_msg.width} points, frame: {cloud_msg.header.frame_id}")
        
        # Parse existing fields
        num_points = cloud_msg.width * cloud_msg.height
        point_step = cloud_msg.point_step
        
        # Create field mapping for existing fields
        field_map = {}
        for field in cloud_msg.fields:
            field_map[field.name] = {
                'offset': field.offset,
                'datatype': field.datatype,
                'count': field.count
            }
        
        # Define new point_step (add 4 bytes for float32 'time' field)
        new_point_step = point_step + 4
        
        # Create new fields list
        new_fields = list(cloud_msg.fields)
        time_field = PointField()
        time_field.name = 'time'
        time_field.offset = point_step  # Append at the end
        time_field.datatype = PointField.FLOAT32
        time_field.count = 1
        new_fields.append(time_field)
        
        # Generate time values (linear distribution)
        time_values = np.linspace(0, scan_duration, num_points, dtype=np.float32)
        
        # Create new data array
        new_data = bytearray()
        
        # Process each point
        for i in range(num_points):
            point_start = i * point_step
            point_end = point_start + point_step
            
            # Copy existing point data
            new_data.extend(cloud_msg.data[point_start:point_end])
            
            # Append time value as float32 (4 bytes, little-endian)
            new_data.extend(struct.pack('<f', time_values[i]))
        
        # Create new PointCloud2 message
        new_msg = PointCloud2()
        new_msg.header = cloud_msg.header
        new_msg.height = cloud_msg.height
        new_msg.width = cloud_msg.width
        new_msg.fields = new_fields
        new_msg.is_bigendian = cloud_msg.is_bigendian
        new_msg.point_step = new_point_step
        new_msg.row_step = new_point_step * cloud_msg.width
        new_msg.data = bytes(new_data)
        new_msg.is_dense = cloud_msg.is_dense
        
        logger.info(f"Successfully added 'time' field (range: 0.0 to {scan_duration}s)")
        logger.info(f"New point_step: {point_step} -> {new_point_step} bytes")
        
        return new_msg
        
    except Exception as e:
        logger.error(f"Failed to add time field: {e}")
        import traceback
        logger.error(traceback.format_exc())
        return cloud_msg


class PointCloudModifierNode(Node):
    def __init__(self):
        super().__init__('point_cloud_modifier_node')
        self.get_logger().info('PointCloudModifierNode started (binary method)')

        # Parameters
        self.declare_parameter('input_topic', '/lidar/points')
        self.declare_parameter('output_topic', '/lidar/points_with_time')
        self.declare_parameter('scan_duration', 0.1)  # 10Hz LiDAR
        
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.scan_duration = self.get_parameter('scan_duration').value

        # Define QoS profile to match Fast-LIO's expectations
        # Fast-LIO uses BEST_EFFORT reliability for point clouds
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscription and Publisher with matching QoS
        self.subscription = self.create_subscription(
            PointCloud2,
            input_topic,
            self.point_cloud_callback,
            qos_profile  # Use QoS profile instead of just depth
        )
        
        self.publisher = self.create_publisher(
            PointCloud2, 
            output_topic, 
            qos_profile  # Use matching QoS profile
        )
        
        self.msg_count = 0
        self.success_count = 0
        
        self.get_logger().info(f'Input topic: {input_topic}')
        self.get_logger().info(f'Output topic: {output_topic}')
        self.get_logger().info(f'Scan duration: {self.scan_duration}s')
        
        # Status timer
        self.create_timer(10.0, self.status_callback)

    def status_callback(self):
        """Periodic status update"""
        if self.msg_count == 0:
            self.get_logger().warn(
                f'No messages received. Check topic: {self.get_parameter("input_topic").value}'
            )
        else:
            self.get_logger().info(
                f'Processed: {self.success_count}/{self.msg_count} messages'
            )

    def point_cloud_callback(self, msg: PointCloud2):
        self.msg_count += 1
        
        # Log details for first few messages
        if self.msg_count <= 3:
            self.get_logger().info(f'\n{"="*50}\nMessage #{self.msg_count}\n{"="*50}')
        
        # Add time field
        modified_msg = add_time_field_to_pointcloud2(
            msg, 
            self.get_logger(), 
            self.scan_duration
        )
        
        if modified_msg is not None:
            self.publisher.publish(modified_msg)
            self.success_count += 1
            
            if self.msg_count <= 3:
                self.get_logger().info(f'Published modified message #{self.msg_count}')


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudModifierNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(
            f'Shutdown. Success rate: {node.success_count}/{node.msg_count}'
        )
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()