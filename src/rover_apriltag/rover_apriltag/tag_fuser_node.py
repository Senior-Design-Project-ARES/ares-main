import math
import os
from typing import Dict, List, Optional, Tuple

import yaml

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import Pose, PoseArray, PoseStamped, TransformStamped
from std_msgs.msg import Int32MultiArray

from tf2_ros import Buffer, TransformBroadcaster, TransformListener, StaticTransformBroadcaster

def rpy_to_quat(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw

def quat_z_axis_in_parent(qx: float, qy: float, qz: float, qw: float) -> Tuple[float, float, float]:
    zx = 2.0 * (qx * qz + qy * qw)
    zy = 2.0 * (qy * qz - qx * qw)
    zz = 1.0 - 2.0 * (qx * qx + qy * qy)
    return zx, zy, zz

class TagFuser(Node):
    def __init__(self) -> None:
        super().__init__("tag_fuser")

        self.declare_parameter("config_path", "")
        config_path = self.get_parameter("config_path").get_parameter_value().string_value
        config_path = os.path.expanduser(config_path)

        if not config_path or not os.path.exists(config_path):
            raise RuntimeError(f"[tag_fuser] config_path is invalid: '{config_path}'")

        with open(config_path, "r", encoding="utf-8") as f:
            self.cfg = yaml.safe_load(f)

        self.rover_frame_id = str(self.cfg.get("rover", {}).get("frame_id", "rover_base"))

        self.cameras: Dict[str, dict] = dict(self.cfg.get("cameras", {}))
        if not self.cameras:
            raise RuntimeError("[tag_fuser] config error: cameras section is empty")

        apr = dict(self.cfg.get("apriltag", {}))
        self.tag_ids: List[int] = [int(x) for x in apr.get("ids", [])]
        if not self.tag_ids:
            raise RuntimeError("[tag_fuser] config error: apriltag.ids is empty")

        pipe = dict(self.cfg.get("pipeline", {}))
        self.publish_rate_hz = float(pipe.get("publish_rate_hz", 30.0))
        self.max_tf_age_sec = float(pipe.get("max_tf_age_sec", 0.25))
        self.camera_priority = [str(x) for x in pipe.get("camera_priority", list(self.cameras.keys()))]

        self.publish_fused_tag_tfs = bool(pipe.get("publish_fused_tag_tfs", True))
        self.fused_tag_frame_prefix = str(pipe.get("fused_tag_frame_prefix", "tag_"))

        self.publish_tag_pose_array = bool(pipe.get("publish_tag_pose_array", True))
        self.tag_pose_array_topic = str(pipe.get("tag_pose_array_topic", "tag_poses"))
        self.tag_ids_topic = str(pipe.get("tag_ids_topic", "tag_ids"))

        tgt = dict(self.cfg.get("target", {}))
        self.target_enabled = bool(tgt.get("enabled", True))
        self.target_frame_id = str(tgt.get("frame_id", "target"))
        self.target_side_length_m = float(tgt.get("side_length_m", 0.15))
        self.target_center_offset_m = float(tgt.get("center_offset_m", -0.5 * self.target_side_length_m))
        self.target_max_jump_m = float(tgt.get("max_jump_m", 0.50))
        self.target_smoothing_alpha = float(tgt.get("smoothing_alpha", 1.0))
        self.target_publish_tf = bool(tgt.get("publish_tf", True))
        self.target_publish_topic = bool(tgt.get("publish_topic", True))
        self.target_topic = str(tgt.get("topic", "target_pose"))
        self.target_use_tag_ids = [int(x) for x in tgt.get("use_tag_ids", self.tag_ids)]

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        if self.publish_tag_pose_array:
            self.tag_pose_pub = self.create_publisher(PoseArray, self.tag_pose_array_topic, 10)
            self.tag_ids_pub = self.create_publisher(Int32MultiArray, self.tag_ids_topic, 10)

        if self.target_enabled and self.target_publish_topic:
            self.target_pose_pub = self.create_publisher(PoseStamped, self.target_topic, 10)

        self._publish_camera_static_tfs()

        self._target_pos: Optional[Tuple[float, float, float]] = None

        period = 1.0 / max(1.0, self.publish_rate_hz)
        self.timer = self.create_timer(period, self._on_timer)

        self.get_logger().info(
            f"tag_fuser running. rover_frame='{self.rover_frame_id}', cameras={list(self.cameras.keys())}, tag_ids={self.tag_ids}"
        )

    def _camera_tag_frame(self, camera_name: str, tag_id: int) -> str:
        cam = self.cameras[camera_name]
        prefix = str(cam.get("tag_frame_prefix", f"{camera_name}_tag_"))
        return f"{prefix}{tag_id}"

    def _fused_tag_frame(self, tag_id: int) -> str:
        return f"{self.fused_tag_frame_prefix}{tag_id}"

    def _publish_camera_static_tfs(self) -> None:
        now = self.get_clock().now().to_msg()
        tfs: List[TransformStamped] = []

        for cam_name, cam_cfg in self.cameras.items():
            pose = dict(cam_cfg.get("pose", {}))
            xyz = pose.get("xyz_m", [0.0, 0.0, 0.0])
            rpy = pose.get("rpy_rad", [0.0, 0.0, 0.0])
            if len(xyz) != 3 or len(rpy) != 3:
                raise RuntimeError(f"[tag_fuser] config error: cameras.{cam_name}.pose.xyz_m and rpy_rad must be length 3")

            qx, qy, qz, qw = rpy_to_quat(float(rpy[0]), float(rpy[1]), float(rpy[2]))

            tf_msg = TransformStamped()
            tf_msg.header.stamp = now
            tf_msg.header.frame_id = self.rover_frame_id
            tf_msg.child_frame_id = str(cam_cfg.get("camera_frame_id", f"{cam_name}_camera_optical_frame"))
            tf_msg.transform.translation.x = float(xyz[0])
            tf_msg.transform.translation.y = float(xyz[1])
            tf_msg.transform.translation.z = float(xyz[2])
            tf_msg.transform.rotation.x = qx
            tf_msg.transform.rotation.y = qy
            tf_msg.transform.rotation.z = qz
            tf_msg.transform.rotation.w = qw
            tfs.append(tf_msg)

        self.static_tf_broadcaster.sendTransform(tfs)

    def _lookup_tag_in_rover(self, tag_frame_id: str) -> Optional[TransformStamped]:
        try:
            tf_msg = self.tf_buffer.lookup_transform(self.rover_frame_id, tag_frame_id, Time())
        except Exception:
            return None

        now = self.get_clock().now()
        stamp = Time.from_msg(tf_msg.header.stamp)
        age = (now - stamp).nanoseconds * 1e-9
        if age > self.max_tf_age_sec:
            return None
        return tf_msg

    def _choose_best_camera_tf(self, tag_id: int) -> Optional[TransformStamped]:
        best: Optional[TransformStamped] = None
        best_age = 1e9

        for cam_name in self.camera_priority:
            if cam_name not in self.cameras:
                continue
            tag_frame = self._camera_tag_frame(cam_name, tag_id)
            tf_msg = self._lookup_tag_in_rover(tag_frame)
            if tf_msg is None:
                continue

            now = self.get_clock().now()
            stamp = Time.from_msg(tf_msg.header.stamp)
            age = (now - stamp).nanoseconds * 1e-9

            if age < best_age:
                best = tf_msg
                best_age = age

        return best

    def _on_timer(self) -> None:
        now_msg = self.get_clock().now().to_msg()

        poses: List[Pose] = []
        ids: List[int] = []

        target_candidates: List[Tuple[float, float, float]] = []

        for tag_id in self.tag_ids:
            best_tf = self._choose_best_camera_tf(tag_id)
            if best_tf is None:
                continue

            if self.publish_fused_tag_tfs:
                fused = TransformStamped()
                fused.header.stamp = now_msg
                fused.header.frame_id = self.rover_frame_id
                fused.child_frame_id = self._fused_tag_frame(tag_id)
                fused.transform = best_tf.transform
                self.tf_broadcaster.sendTransform(fused)

            if self.publish_tag_pose_array:
                p = Pose()
                p.position.x = best_tf.transform.translation.x
                p.position.y = best_tf.transform.translation.y
                p.position.z = best_tf.transform.translation.z
                p.orientation = best_tf.transform.rotation
                poses.append(p)
                ids.append(tag_id)

            if self.target_enabled and (tag_id in self.target_use_tag_ids):
                tx = best_tf.transform.translation.x
                ty = best_tf.transform.translation.y
                tz = best_tf.transform.translation.z
                q = best_tf.transform.rotation
                zx, zy, zz = quat_z_axis_in_parent(q.x, q.y, q.z, q.w)

                vax = zx * self.target_center_offset_m
                vay = zy * self.target_center_offset_m
                vaz = zz * self.target_center_offset_m

                cand_a = (tx + vax, ty + vay, tz + vaz)
                cand_b = (tx - vax, ty - vay, tz - vaz)

                if self._target_pos is None:
                    chosen = cand_a
                else:
                    pa = (cand_a[0] - self._target_pos[0], cand_a[1] - self._target_pos[1], cand_a[2] - self._target_pos[2])
                    pb = (cand_b[0] - self._target_pos[0], cand_b[1] - self._target_pos[1], cand_b[2] - self._target_pos[2])
                    da = pa[0] * pa[0] + pa[1] * pa[1] + pa[2] * pa[2]
                    db = pb[0] * pb[0] + pb[1] * pb[1] + pb[2] * pb[2]
                    chosen = cand_a if da <= db else cand_b

                target_candidates.append(chosen)

        if self.publish_tag_pose_array:
            msg = PoseArray()
            msg.header.stamp = now_msg
            msg.header.frame_id = self.rover_frame_id
            msg.poses = poses
            self.tag_pose_pub.publish(msg)

            id_msg = Int32MultiArray()
            id_msg.data = ids
            self.tag_ids_pub.publish(id_msg)

        if self.target_enabled and target_candidates:
            cx = sum(p[0] for p in target_candidates) / len(target_candidates)
            cy = sum(p[1] for p in target_candidates) / len(target_candidates)
            cz = sum(p[2] for p in target_candidates) / len(target_candidates)

            if self._target_pos is not None:
                dx = cx - self._target_pos[0]
                dy = cy - self._target_pos[1]
                dz = cz - self._target_pos[2]
                dist = math.sqrt(dx * dx + dy * dy + dz * dz)
                if dist > self.target_max_jump_m:
                    return

                a = max(0.0, min(1.0, self.target_smoothing_alpha))
                cx = a * cx + (1.0 - a) * self._target_pos[0]
                cy = a * cy + (1.0 - a) * self._target_pos[1]
                cz = a * cz + (1.0 - a) * self._target_pos[2]

            self._target_pos = (cx, cy, cz)

            if self.target_publish_topic:
                p = PoseStamped()
                p.header.stamp = now_msg
                p.header.frame_id = self.rover_frame_id
                p.pose.position.x = cx
                p.pose.position.y = cy
                p.pose.position.z = cz
                p.pose.orientation.w = 1.0
                self.target_pose_pub.publish(p)

            if self.target_publish_tf:
                tf_msg = TransformStamped()
                tf_msg.header.stamp = now_msg
                tf_msg.header.frame_id = self.rover_frame_id
                tf_msg.child_frame_id = self.target_frame_id
                tf_msg.transform.translation.x = cx
                tf_msg.transform.translation.y = cy
                tf_msg.transform.translation.z = cz
                tf_msg.transform.rotation.w = 1.0
                self.tf_broadcaster.sendTransform(tf_msg)

def main() -> None:
    rclpy.init()
    node = TagFuser()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
