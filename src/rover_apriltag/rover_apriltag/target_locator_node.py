import math
import os
from itertools import product
from typing import List, Optional, Sequence, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformListener

from rover_apriltag.config_utils import (
    camera_frame_id,
    camera_names,
    load_config,
    require_float,
    require_positive_float,
    require_str,
    require_vector,
    rover_frame_id,
    rover_name,
    rover_pose_topic,
    tag_frame_id,
    world_frame_id,
)


Point2 = Tuple[float, float]
RoverPose2 = Tuple[float, float, float]
SharedTarget = Tuple[float, float, float]


def rpy_to_quat(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def quat_z_axis_in_parent(qx: float, qy: float, qz: float, qw: float) -> Point2:
    return (
        2.0 * (qx * qz + qy * qw),
        2.0 * (qy * qz - qx * qw),
    )


def quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> Optional[float]:
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 1e-12:
        return None
    qx /= norm
    qy /= norm
    qz /= norm
    qw /= norm
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def planar_distance(a: Point2, b: Point2) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


class TargetLocator(Node):
    def __init__(self) -> None:
        super().__init__("target_locator")

        self.declare_parameter("config_path", "")
        config_path = self.get_parameter("config_path").get_parameter_value().string_value
        config_path = os.path.expanduser(config_path)
        if not config_path or not os.path.exists(config_path):
            raise RuntimeError(f"config_path is invalid: '{config_path}'")

        self.cfg = load_config(config_path)

        self._rover_name = rover_name(self.cfg)
        self._world_frame_id = world_frame_id(self.cfg)
        self._rover_frame_id = rover_frame_id(self.cfg)
        self._rover_pose_topic = rover_pose_topic(self.cfg)
        self._camera_names = camera_names(self.cfg)
        self._tag_ids = [int(tag_id) for tag_id in self.cfg["target"]["tag_ids"]]
        self._tag_center_offset_m = 0.5 * require_positive_float(self.cfg, ["target", "cube_size_m"], "target.cube_size_m")
        self._max_jump_m = require_positive_float(self.cfg, ["target", "max_jump_m"], "target.max_jump_m")
        self._publish_rate_hz = require_positive_float(self.cfg, ["pipeline", "publish_rate_hz"], "pipeline.publish_rate_hz")
        self._max_tag_age_sec = require_float(self.cfg, ["pipeline", "max_tag_age_sec"], "pipeline.max_tag_age_sec")

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=True)
        self._static_tf_broadcaster = StaticTransformBroadcaster(self)

        self._rover_pose: Optional[RoverPose2] = None
        self._shared_target: Optional[SharedTarget] = None
        self._local_target_reference: Optional[Point2] = None
        self._warned_invalid_rover_orientation = False

        target_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.create_subscription(PoseStamped, self._rover_pose_topic, self._on_rover_pose, 10)
        self.create_subscription(PoseStamped, "/target_location", self._on_shared_target, target_qos)
        self._target_pub = self.create_publisher(PoseStamped, "/target_location", target_qos)

        self._publish_camera_static_transforms()
        self.create_timer(1.0 / self._publish_rate_hz, self._on_timer)

        self.get_logger().info(
            f"target_locator running for rover '{self._rover_name}' on topic '{self._rover_pose_topic}'"
        )

    def _publish_camera_static_transforms(self) -> None:
        stamp = self.get_clock().now().to_msg()
        transforms: List[TransformStamped] = []

        for camera_name in self._camera_names:
            xyz = require_vector(
                self.cfg,
                ["cameras", camera_name, "pose", "xyz_m"],
                f"cameras.{camera_name}.pose.xyz_m",
                length=3,
            )
            rpy = require_vector(
                self.cfg,
                ["cameras", camera_name, "pose", "rpy_rad"],
                f"cameras.{camera_name}.pose.rpy_rad",
                length=3,
            )
            qx, qy, qz, qw = rpy_to_quat(rpy[0], rpy[1], rpy[2])

            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = self._rover_frame_id
            transform.child_frame_id = camera_frame_id(self.cfg, camera_name)
            transform.transform.translation.x = xyz[0]
            transform.transform.translation.y = xyz[1]
            transform.transform.translation.z = xyz[2]
            transform.transform.rotation.x = qx
            transform.transform.rotation.y = qy
            transform.transform.rotation.z = qz
            transform.transform.rotation.w = qw
            transforms.append(transform)

        self._static_tf_broadcaster.sendTransform(transforms)

    def _on_rover_pose(self, msg: PoseStamped) -> None:
        yaw = quat_to_yaw(
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        )
        if yaw is None:
            if not self._warned_invalid_rover_orientation:
                self.get_logger().warning(
                    f"Ignoring rover pose from '{self._rover_pose_topic}' because the quaternion is invalid."
                )
                self._warned_invalid_rover_orientation = True
            return

        self._warned_invalid_rover_orientation = False
        self._rover_pose = (msg.pose.position.x, msg.pose.position.y, yaw)

    def _on_shared_target(self, msg: PoseStamped) -> None:
        distance = float(msg.pose.orientation.x)
        candidate = (float(msg.pose.position.x), float(msg.pose.position.y), distance)
        if self._shared_target is None or distance < self._shared_target[2]:
            self._shared_target = candidate

    def _lookup_tag_transform(self, tag_frame: str) -> Optional[TransformStamped]:
        try:
            transform = self._tf_buffer.lookup_transform(self._rover_frame_id, tag_frame, Time())
        except Exception:
            return None

        age = (self.get_clock().now() - Time.from_msg(transform.header.stamp)).nanoseconds * 1e-9
        if age > self._max_tag_age_sec:
            return None
        return transform

    def _best_transform_for_tag(self, tag_id: int) -> Optional[TransformStamped]:
        best: Optional[TransformStamped] = None
        best_age = math.inf

        for camera_name in self._camera_names:
            transform = self._lookup_tag_transform(tag_frame_id(self.cfg, camera_name, tag_id))
            if transform is None:
                continue
            age = (self.get_clock().now() - Time.from_msg(transform.header.stamp)).nanoseconds * 1e-9
            if age < best_age:
                best = transform
                best_age = age

        return best

    def _reference_target(self) -> Optional[Point2]:
        if self._shared_target is not None:
            return (self._shared_target[0], self._shared_target[1])
        return self._local_target_reference

    def _local_to_world(self, point: Point2) -> Point2:
        if self._rover_pose is None:
            raise RuntimeError("rover pose is not available")
        rover_x, rover_y, rover_yaw = self._rover_pose
        cos_yaw = math.cos(rover_yaw)
        sin_yaw = math.sin(rover_yaw)
        return (
            rover_x + cos_yaw * point[0] - sin_yaw * point[1],
            rover_y + sin_yaw * point[0] + cos_yaw * point[1],
        )

    def _candidate_pair_for_transform(self, transform: TransformStamped) -> Tuple[Point2, Point2]:
        tx = float(transform.transform.translation.x)
        ty = float(transform.transform.translation.y)
        q = transform.transform.rotation
        nx, ny = quat_z_axis_in_parent(q.x, q.y, q.z, q.w)

        candidate_a_local = (tx - nx * self._tag_center_offset_m, ty - ny * self._tag_center_offset_m)
        candidate_b_local = (tx + nx * self._tag_center_offset_m, ty + ny * self._tag_center_offset_m)
        return (self._local_to_world(candidate_a_local), self._local_to_world(candidate_b_local))

    def _select_points_without_reference(self, pairs: Sequence[Tuple[Point2, Point2]]) -> List[Point2]:
        if len(pairs) == 1:
            return [pairs[0][0]]

        best_error = math.inf
        best_selection: List[Point2] = []

        for choice in product((0, 1), repeat=len(pairs)):
            selection = [pairs[index][bit] for index, bit in enumerate(choice)]
            center_x = sum(point[0] for point in selection) / len(selection)
            center_y = sum(point[1] for point in selection) / len(selection)
            error = sum((point[0] - center_x) ** 2 + (point[1] - center_y) ** 2 for point in selection)
            if error < best_error:
                best_error = error
                best_selection = selection

        return best_selection

    def _estimate_target(self) -> Optional[Point2]:
        if self._rover_pose is None:
            return None

        pairs: List[Tuple[Point2, Point2]] = []
        for tag_id in self._tag_ids:
            transform = self._best_transform_for_tag(tag_id)
            if transform is None:
                continue
            pairs.append(self._candidate_pair_for_transform(transform))

        if not pairs:
            return None

        reference = self._reference_target()
        if reference is None:
            selected = self._select_points_without_reference(pairs)
        else:
            selected = [
                pair[0] if planar_distance(pair[0], reference) <= planar_distance(pair[1], reference) else pair[1]
                for pair in pairs
            ]

        center_x = sum(point[0] for point in selected) / len(selected)
        center_y = sum(point[1] for point in selected) / len(selected)
        estimate = (center_x, center_y)

        if reference is not None and planar_distance(estimate, reference) > self._max_jump_m:
            return None

        return estimate

    def _on_timer(self) -> None:
        estimate = self._estimate_target()
        if estimate is None or self._rover_pose is None:
            return

        rover_xy = (self._rover_pose[0], self._rover_pose[1])
        distance = planar_distance(estimate, rover_xy)
        if self._shared_target is not None and distance >= self._shared_target[2]:
            return

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._world_frame_id
        msg.pose.position.x = estimate[0]
        msg.pose.position.y = estimate[1]
        msg.pose.position.z = 0.0
        msg.pose.orientation.x = distance
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 0.0
        self._target_pub.publish(msg)

        self._shared_target = (estimate[0], estimate[1], distance)
        self._local_target_reference = estimate


def main() -> None:
    rclpy.init()
    node = TargetLocator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
