#!/usr/bin/env python3
"""
Occupancy grid mapper for the A2M12 RPLidar — stationary test version.

Uses a hardcoded fixed pose instead of Vicon so you can test
the map and laser scan display without the Vicon system.
Robot is placed near the bottom-left corner of the grid.
"""

import math
from typing import Tuple

import numpy as np

import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def world_to_grid(
    x: float, y: float,
    origin_x: float, origin_y: float,
    res: float,
) -> Tuple[int, int]:
    gx = int(math.floor((x - origin_x) / res))
    gy = int(math.floor((y - origin_y) / res))
    return gx, gy


def in_bounds(gx: int, gy: int, width: int, height: int) -> bool:
    return 0 <= gx < width and 0 <= gy < height


def bresenham(x0: int, y0: int, x1: int, y1: int):
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    x, y = x0, y0
    while True:
        yield x, y
        if x == x1 and y == y1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x += sx
        if e2 < dx:
            err += dx
            y += sy


def yaw_from_quat(qx: float, qy: float, qz: float, qw: float) -> float:
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


# ---------------------------------------------------------------------------
# Mapper node
# ---------------------------------------------------------------------------

class OccupancyMapper(Node):

    L_OCC:  float = 2.0
    L_FREE: float = 0.10

    HARD_RANGE_MAX: float = 4.0

    def __init__(self):
        super().__init__('occupancy_mapper')

        # ---- parameters -------------------------------------------------------
        self.declare_parameter('map_frame',        'map')
        self.declare_parameter('resolution',        0.02)
        self.declare_parameter('width',             250)
        self.declare_parameter('height',            250)
        self.declare_parameter('max_log',           50.0)
        self.declare_parameter('min_log',          -10.0)
        self.declare_parameter('range_min',         0.15)
        self.declare_parameter('publish_rate',      2.0)
        self.declare_parameter('occ_radius_cells',  1)

        # Fixed robot position in world coordinates.
        # The map origin will be set so this position lands
        # near the bottom-left corner of the grid.
        # Change these to wherever your robot actually is sitting.
        self.declare_parameter('robot_x',   0.0)
        self.declare_parameter('robot_y',   0.0)
        self.declare_parameter('robot_yaw', 0.0)  # degrees

        # Laser mounting offset from robot centre (metres)
        self.declare_parameter('laser_offset_x', 0.0)
        self.declare_parameter('laser_offset_y', 0.0)

        self.map_frame      = self.get_parameter('map_frame').value
        self.res            = float(self.get_parameter('resolution').value)
        self.width          = int(self.get_parameter('width').value)
        self.height         = int(self.get_parameter('height').value)
        self.max_log        = float(self.get_parameter('max_log').value)
        self.min_log        = float(self.get_parameter('min_log').value)
        self.rmin           = float(self.get_parameter('range_min').value)
        pub_rate            = float(self.get_parameter('publish_rate').value)
        self.occ_radius     = int(self.get_parameter('occ_radius_cells').value)
        self.laser_offset_x = float(self.get_parameter('laser_offset_x').value)
        self.laser_offset_y = float(self.get_parameter('laser_offset_y').value)

        # Fixed robot pose — stationary for this test version
        robot_x   = float(self.get_parameter('robot_x').value)
        robot_y   = float(self.get_parameter('robot_y').value)
        robot_yaw = math.radians(float(self.get_parameter('robot_yaw').value))

        # Place robot near bottom-left corner with a 10-cell padding
        padding = 10
        self.origin_x = robot_x - padding * self.res
        self.origin_y = robot_y - padding * self.res

        # Fixed laser position in world space
        self.tx = robot_x + math.cos(robot_yaw) * self.laser_offset_x \
                           - math.sin(robot_yaw) * self.laser_offset_y
        self.ty = robot_y + math.sin(robot_yaw) * self.laser_offset_x \
                           + math.cos(robot_yaw) * self.laser_offset_y
        self.laser_yaw = robot_yaw

        # ---- internal state ---------------------------------------------------
        self.log = np.zeros((self.height, self.width), dtype=np.float32)

        # ---- TF broadcasters --------------------------------------------------
        self.static_br  = StaticTransformBroadcaster(self)
        self.dynamic_br = TransformBroadcaster(self)
        self._publish_static_frames(robot_x, robot_y, robot_yaw)

        # ---- ROS interfaces ---------------------------------------------------
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 1)

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.on_scan, 10
        )

        self.timer = self.create_timer(1.0 / pub_rate, self.publish_map)

        self.get_logger().info(
            f"OccupancyMapper (stationary test)  |  "
            f"grid {self.width}×{self.height} @ {self.res} m/cell  |  "
            f"robot fixed at ({robot_x:.2f}, {robot_y:.2f}) "
            f"yaw={math.degrees(robot_yaw):.1f}°  |  "
            f"origin ({self.origin_x:.3f}, {self.origin_y:.3f})"
        )

    # -----------------------------------------------------------------------
    # Static TF frames
    # -----------------------------------------------------------------------

    def _publish_static_frames(self, robot_x, robot_y, robot_yaw):
        """
        Publish all TF frames so RViz is happy.
        world → map  (identity, static)
        map → base_link  (fixed robot pose, static since robot isn't moving)
        base_link → laser  (identity, static)
        """
        now = self.get_clock().now().to_msg()
        transforms = []

        # world → map
        t = TransformStamped()
        t.header.stamp    = now
        t.header.frame_id = 'world'
        t.child_frame_id  = 'map'
        t.transform.rotation.w = 1.0
        transforms.append(t)

        # map → base_link — fixed robot position
        qz = math.sin(robot_yaw / 2.0)
        qw = math.cos(robot_yaw / 2.0)

        t2 = TransformStamped()
        t2.header.stamp       = now
        t2.header.frame_id    = 'map'
        t2.child_frame_id     = 'base_link'
        t2.transform.translation.x = robot_x
        t2.transform.translation.y = robot_y
        t2.transform.translation.z = 0.0
        t2.transform.rotation.z    = qz
        t2.transform.rotation.w    = qw
        transforms.append(t2)

        # base_link → laser
        t3 = TransformStamped()
        t3.header.stamp    = now
        t3.header.frame_id = 'base_link'
        t3.child_frame_id  = 'laser'
        t3.transform.rotation.w = 1.0
        transforms.append(t3)

        self.static_br.sendTransform(transforms)

    # -----------------------------------------------------------------------
    # Grid helpers
    # -----------------------------------------------------------------------

    def _clamp_grid(self, gx: int, gy: int) -> Tuple[int, int]:
        return (
            max(0, min(self.width  - 1, gx)),
            max(0, min(self.height - 1, gy)),
        )

    def _mark_free(self, cx: int, cy: int):
        if not in_bounds(cx, cy, self.width, self.height):
            return
        self.log[cy, cx] = max(self.log[cy, cx] - self.L_FREE, self.min_log)

    def _mark_occupied_thick(self, gx: int, gy: int):
        r  = self.occ_radius
        x0 = max(0, gx - r);  x1 = min(self.width,  gx + r + 1)
        y0 = max(0, gy - r);  y1 = min(self.height, gy + r + 1)
        self.log[y0:y1, x0:x1] = np.clip(
            self.log[y0:y1, x0:x1] + self.L_OCC,
            self.min_log, self.max_log,
        )

    # -----------------------------------------------------------------------
    # Scan callback
    # -----------------------------------------------------------------------

    def on_scan(self, scan: LaserScan):

        ogx, ogy = world_to_grid(
            self.tx, self.ty, self.origin_x, self.origin_y, self.res
        )

        if not in_bounds(ogx, ogy, self.width, self.height):
            self.get_logger().warn(
                'Robot is outside the occupancy grid.',
                throttle_duration_sec=2.0,
            )
            return

        angle = scan.angle_min

        for r in scan.ranges:
            a = self.laser_yaw + angle
            angle += scan.angle_increment

            hit_obstacle = False

            if math.isfinite(r):
                if r < self.rmin:
                    continue
                if r > self.HARD_RANGE_MAX:
                    ray_range = self.HARD_RANGE_MAX
                else:
                    ray_range    = r
                    hit_obstacle = True
            else:
                ray_range = self.HARD_RANGE_MAX

            ex = self.tx + ray_range * math.cos(a)
            ey = self.ty + ray_range * math.sin(a)

            egx, egy = world_to_grid(
                ex, ey, self.origin_x, self.origin_y, self.res
            )
            egx, egy = self._clamp_grid(egx, egy)

            cells = list(bresenham(ogx, ogy, egx, egy))
            if len(cells) < 2:
                continue

            if hit_obstacle:
                for (cx, cy) in cells[:-1]:
                    self._mark_free(cx, cy)
                endx, endy = cells[-1]
                self._mark_occupied_thick(endx, endy)
            else:
                for (cx, cy) in cells:
                    self._mark_free(cx, cy)

    # -----------------------------------------------------------------------
    # Map publisher
    # -----------------------------------------------------------------------

    def publish_map(self):
        msg                           = OccupancyGrid()
        msg.header.stamp              = self.get_clock().now().to_msg()
        msg.header.frame_id           = self.map_frame
        msg.info.resolution           = self.res
        msg.info.width                = self.width
        msg.info.height               = self.height
        msg.info.origin.position.x    = self.origin_x
        msg.info.origin.position.y    = self.origin_y
        msg.info.origin.position.z    = 0.0
        msg.info.origin.orientation.w = 1.0

        log_range = self.max_log - self.min_log
        scaled    = ((self.log - self.min_log) / log_range * 100.0).astype(np.int8)

        unknown_mask = (self.log == 0.0)
        scaled[unknown_mask] = -1

        np.clip(scaled, -1, 100, out=scaled)

        msg.data = scaled.flatten().tolist()
        self.map_pub.publish(msg)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    rclpy.init()
    node = OccupancyMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
