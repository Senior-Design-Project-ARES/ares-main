#!/usr/bin/env python3
"""
Occupancy grid mapper for the A2M12 RPLidar — TF-free version.

Key design decisions:
  - No TF lookup. Pose comes directly from /Wand/pose (PoseStamped).
  - Publishes static TF frames internally so RViz is happy:
      world → map → base_link → laser
  - Log-odds probabilistic mapping
  - Obstacle endpoints thickened to fill ray gaps.
  - Range hard-capped at 4 m.
  - Scan and pose data buffered separately, matched by timestamp,
    and processed together in a dedicated publish loop.
"""

import math
from collections import deque
from typing import Tuple, Optional

import numpy as np

import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
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


def stamp_to_sec(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


# ---------------------------------------------------------------------------
# Mapper node
# ---------------------------------------------------------------------------

class OccupancyMapper(Node):

    L_OCC:  float = 1.0
    L_FREE: float = 1.0

    HARD_RANGE_MAX: float = 4.0

    # Maximum allowed time difference between a scan and its matched pose
    MAX_SYNC_DELTA_SEC: float = 0.2   # 20 ms — tight for 60 Hz Vicon

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
        self.declare_parameter('publish_rate',      12.0)   # match lidar rate
        self.declare_parameter('occ_radius_cells',  1)
        self.declare_parameter('start_padding_cells', 5)
        self.declare_parameter('laser_offset_x',    0.0)
        self.declare_parameter('laser_offset_y',    0.0)

        self.map_frame      = self.get_parameter('map_frame').value
        self.res            = float(self.get_parameter('resolution').value)
        self.width          = int(self.get_parameter('width').value)
        self.height         = int(self.get_parameter('height').value)
        self.max_log        = float(self.get_parameter('max_log').value)
        self.min_log        = float(self.get_parameter('min_log').value)
        self.rmin           = float(self.get_parameter('range_min').value)
        pub_rate            = float(self.get_parameter('publish_rate').value)
        self.occ_radius     = int(self.get_parameter('occ_radius_cells').value)
        self.start_pad      = int(self.get_parameter('start_padding_cells').value)
        self.laser_offset_x = float(self.get_parameter('laser_offset_x').value)
        self.laser_offset_y = float(self.get_parameter('laser_offset_y').value)

        # ---- internal state ---------------------------------------------------
        self.log = np.zeros((self.height, self.width), dtype=np.float32)

        self.origin_x = 0.0
        self.origin_y = 0.0

        # Separate buffers for scans and poses
        # Scans buffer: holds unprocessed scans waiting for a matched pose
        # Vicon buffer: holds recent poses for interpolation
        # At 12 Hz lidar, 20 unprocessed scans = ~1.6 seconds backlog before dropping
        self.scan_buffer: deque  = deque(maxlen=2)
        # At 60 Hz vicon, 120 poses = 2 seconds of history
        self.pose_buffer: deque  = deque(maxlen=120)

        # ---- TF broadcasters --------------------------------------------------
        self.static_br  = StaticTransformBroadcaster(self)
        self.dynamic_br = TransformBroadcaster(self)

        self._publish_static_frames()

        # ---- ROS interfaces ---------------------------------------------------
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 1)

        self.pose_sub = self.create_subscription(
            PoseStamped,
            'pose',
            self.on_pose,
            10,
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.on_scan,
            10,
        )

        # Processing loop runs at publish_rate — drains scan buffer,
        # matches poses, updates grid, and publishes map
        self.timer = self.create_timer(1.0 / pub_rate, self.process_and_publish)

        self.get_logger().info(
            f"OccupancyMapper ready  |  "
            f"grid {self.width}×{self.height} @ {self.res} m/cell  |  "
            f"range cap {self.HARD_RANGE_MAX} m  |  "
            f"L_OCC={self.L_OCC}  L_FREE={self.L_FREE}  |  "
            f"sync tolerance {self.MAX_SYNC_DELTA_SEC*1000:.0f} ms  |  "
            f"processing at {pub_rate} Hz"
        )

    # -----------------------------------------------------------------------
    # Static TF frames
    # -----------------------------------------------------------------------

    def _publish_static_frames(self):
        now = self.get_clock().now().to_msg()
        transforms = []

        # world → map
        t = TransformStamped()
        t.header.stamp    = now
        t.header.frame_id = 'world'
        t.child_frame_id  = 'map'
        t.transform.rotation.w = 1.0
        transforms.append(t)

        # base_link → laser
        t2 = TransformStamped()
        t2.header.stamp    = now
        t2.header.frame_id = 'base_link'
        t2.child_frame_id  = 'laser'
        t2.transform.rotation.w = 1.0
        transforms.append(t2)

        self.static_br.sendTransform(transforms)

    # -----------------------------------------------------------------------
    # Callbacks — only buffer incoming data, no processing here
    # -----------------------------------------------------------------------

    def on_pose(self, msg: PoseStamped):
        """Buffer pose. Also broadcast TF for RViz."""
        self.pose_buffer.append(msg)

        t = TransformStamped()
        t.header.stamp    = msg.header.stamp
        t.header.frame_id = 'map'
        t.child_frame_id  = 'base_link'
        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = 0.0
        t.transform.rotation      = msg.pose.orientation
        self.dynamic_br.sendTransform(t)

    def on_scan(self, scan: LaserScan):
        """Buffer scan for later processing."""
        self.get_logger().info(f'Received scan with timestamp {stamp_to_sec(scan.header.stamp):.3f}')  
        self.scan_buffer.append(scan)

    # -----------------------------------------------------------------------
    # Pose interpolation
    # -----------------------------------------------------------------------

    def _get_pose_at_time_sec(self, query_t: float) -> Optional[PoseStamped]:
        """
        Interpolate the Vicon pose buffer to find the pose at query_t.
        Returns None if no bracketing poses exist within MAX_SYNC_DELTA_SEC.
        """
        if len(self.pose_buffer) < 2:
            return None

        pose_list = list(self.pose_buffer)
        times = np.asarray(
            [stamp_to_sec(p.header.stamp) for p in pose_list],
            dtype=np.float64,
        )

        # self.get_logger().info(f'--- SCAN BUFFER ({len(scan_times)} msgs) ---')
        # for t in scan_times:
        #     self.get_logger().info(f'  scan_t: {t:.6f}')

        # self.get_logger().info(f'--- POSE BUFFER ({len(times)} msgs {len(pose_list)}) ---')
        # for t in times:
        #     self.get_logger().info(f'  pose_t: {t:.6f}')

        before   = None
        after    = None
        before_t = -float('inf')
        after_t  =  float('inf')
        before_i = None
        after_i  = None

        # Find bracket indices in O(log n) using NumPy's binary search.
        after_pos = int(np.searchsorted(times, query_t, side='right'))
        before_pos = after_pos - 1

        if before_pos >= 0:
            before_i = before_pos
            before_t = float(times[before_pos])
            before = pose_list[before_pos]

        if after_pos < len(pose_list):
            after_i = after_pos
            after_t = float(times[after_pos])
            after = pose_list[after_pos]

        # Fall back to nearest single pose
        if before is None and after is None:
            self.get_logger().warn('fuck, no before and after')
            return None
        # if before is None:
        #     return after if abs(after_t - query_t) < self.MAX_SYNC_DELTA_SEC else None
        # if after is None:
        #     return before if abs(query_t - before_t) < self.MAX_SYNC_DELTA_SEC else None

        if abs(after_t - query_t) <= abs(query_t - before_t):
            # self.get_logger().info(f'a: {after_i}')
            # self.get_logger().info(f'{abs(after_t - query_t)}')
            return after
        elif abs(after_t - query_t) > abs(query_t - before_t):
            # self.get_logger().info(f'b: {before_i}')
            # self.get_logger().info(f'{abs(query_t - before_t)}')
            return before
        else:
            self.get_logger().warn('fuck, fuck')
            return None

        # Bracket too wide — Vicon gap, use nearest
        if (after_t - before_t) > self.MAX_SYNC_DELTA_SEC:
            self.get_logger().warn(
                f'Vicon bracket {(after_t - before_t)*1000:.1f} ms wide',
                throttle_duration_sec=2.0,
            )
            nearest = before if (query_t - before_t) < (after_t - query_t) else after
            return nearest

        # Interpolation factor
        alpha = (query_t - before_t) / (after_t - before_t)

        interp = PoseStamped()
        interp.header.frame_id = before.header.frame_id

        # Linear position interpolation
        bp = before.pose.position
        ap = after.pose.position
        interp.pose.position.x = bp.x + alpha * (ap.x - bp.x)
        interp.pose.position.y = bp.y + alpha * (ap.y - bp.y)
        interp.pose.position.z = bp.z + alpha * (ap.z - bp.z)

        # SLERP orientation interpolation
        q1 = [before.pose.orientation.x, before.pose.orientation.y,
              before.pose.orientation.z, before.pose.orientation.w]
        q2 = [after.pose.orientation.x,  after.pose.orientation.y,
              after.pose.orientation.z,  after.pose.orientation.w]

        dot = sum(a * b for a, b in zip(q1, q2))
        if dot < 0.0:
            q2  = [-v for v in q2]
            dot = -dot
        dot = min(1.0, dot)

        if dot > 0.9995:
            qr = [q1[i] + alpha * (q2[i] - q1[i]) for i in range(4)]
        else:
            theta_0 = math.acos(dot)
            theta   = theta_0 * alpha
            sin_t0  = math.sin(theta_0)
            qr = [
                (math.sin(theta_0 - theta) * q1[i] +
                 math.sin(theta)           * q2[i]) / sin_t0
                for i in range(4)
            ]

        mag = math.sqrt(sum(v * v for v in qr))
        qr  = [v / mag for v in qr]

        interp.pose.orientation.x = qr[0]
        interp.pose.orientation.y = qr[1]
        interp.pose.orientation.z = qr[2]
        interp.pose.orientation.w = qr[3]

        return interp

    # -----------------------------------------------------------------------
    # Scan-to-pose matching
    # -----------------------------------------------------------------------

    def _match_scan_to_pose(self, scan: LaserScan) -> Optional[PoseStamped]:
        """
        Find the best pose for the centre timestamp of this scan.
        Using the centre of the scan (half scan_time) gives a better
        average pose than using the start stamp directly.
        """
        scan_centre_t = (stamp_to_sec(scan.header.stamp)
                         + scan.scan_time * 0.5)
        return self._get_pose_at_time_sec(scan_centre_t)

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
    # Scan integration — per-ray time-matched pose
    # -----------------------------------------------------------------------

    def _integrate_scan(self, scan: LaserScan):
        """
        Process a single scan. Each ray uses its own interpolated pose
        to account for rover movement during the scan rotation.
        """
        angle = scan.angle_min

        for i, r in enumerate(scan.ranges):
            if r > scan.range_max or r < scan.range_min:
                angle += scan.angle_increment
                continue

            # Exact capture time for this ray
            ray_t = (stamp_to_sec(scan.header.stamp)
                     + i * scan.time_increment)

            pose = self._get_pose_at_time_sec(ray_t)
            if pose is None:
                angle += scan.angle_increment
                continue

            bx  = pose.pose.position.x
            by  = pose.pose.position.y
            yaw = yaw_from_quat(
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w,
            )

            tx = bx + math.cos(yaw) * self.laser_offset_x \
                    - math.sin(yaw) * self.laser_offset_y
            ty = by + math.sin(yaw) * self.laser_offset_x \
                    + math.cos(yaw) * self.laser_offset_y
            laser_yaw = yaw

            ogx, ogy = world_to_grid(tx, ty, self.origin_x, self.origin_y, self.res)
            if not in_bounds(ogx, ogy, self.width, self.height):
                self.get_logger().warn(
                    'Robot outside occupancy grid.',
                    throttle_duration_sec=2.0,
                )
                angle += scan.angle_increment
                continue

            a = laser_yaw + angle
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

            ex = tx + ray_range * math.cos(a)
            ey = ty + ray_range * math.sin(a)

            egx, egy = world_to_grid(ex, ey, self.origin_x, self.origin_y, self.res)
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
    # Main processing loop — called by timer
    # -----------------------------------------------------------------------

    def process_and_publish(self):
        """
        Drain the scan buffer. For each scan, check that a matching pose
        exists before integrating. Publish the map after processing.
        Scans with no matchable pose are dropped with a warning.
        """
        if not self.pose_buffer:
            self.get_logger().warn(
                'No Vicon poses received yet.',
                throttle_duration_sec=2.0,
            )
            return

        processed = 0
        dropped   = 0

        # Work through every buffered scan
        while self.scan_buffer:
            self.get_logger().info(f'Scan buffer size: {len(self.scan_buffer)}')
            scan = self.scan_buffer.popleft()

            # Quick check: does a usable pose exist for this scan?
            matched_pose = self._match_scan_to_pose(scan)
            if matched_pose is None:
                dropped += 1
                self.get_logger().warn(
                    f'Dropped scan — no pose within '
                    f'{self.MAX_SYNC_DELTA_SEC*1000:.0f} ms  '
                    f'(scan_t={stamp_to_sec(scan.header.stamp):.3f})',
                    throttle_duration_sec=2.0,
                )
                continue

            self._integrate_scan(scan)
            processed += 1

        if processed == 0 and dropped == 0:
            return  # Nothing to do, skip publish

        if dropped > 0:
            self.get_logger().warn(
                f'This cycle: {processed} scans processed, {dropped} dropped.',
                throttle_duration_sec=2.0,
            )

        self._publish_map()

    # -----------------------------------------------------------------------
    # Map publisher
    # -----------------------------------------------------------------------

    def _publish_map(self):
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