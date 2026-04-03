import os
from typing import Any, Dict, List

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def _load_config(path: str) -> Dict[str, Any]:
    path = os.path.expanduser(path)
    with open(path, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    if not isinstance(cfg, dict):
        raise RuntimeError(f"Config file did not parse as a dictionary: {path}")
    return cfg

def _require(cfg: Dict[str, Any], keys: List[str], path: str) -> Any:
    cur: Any = cfg
    for k in keys:
        if not isinstance(cur, dict) or k not in cur:
            raise RuntimeError(f"Missing required config key: {path}")
        cur = cur[k]
    return cur

def _make_nodes(context, *args, **kwargs):
    config_path = LaunchConfiguration("config").perform(context)
    cfg = _load_config(config_path)

    cameras = _require(cfg, ["cameras"], "cameras")
    apr = _require(cfg, ["apriltag"], "apriltag")
    tag_ids = _require(cfg, ["apriltag", "ids"], "apriltag.ids")

    camera_order = list(cameras.keys())
    camera_priority = cfg.get("pipeline", {}).get("camera_priority", None)
    if isinstance(camera_priority, list) and camera_priority:
        camera_order = [c for c in camera_priority if c in cameras] + [c for c in cameras.keys() if c not in camera_priority]

    nodes: List[Node] = []

    for cam_name in camera_order:
        cam = cameras[cam_name]
        if not isinstance(cam, dict):
            raise RuntimeError(f"cameras.{cam_name} must be a dict")

        v4l2_params = {
            "video_device": str(cam.get("video_device", "/dev/video0")),
            "camera_info_url": str(cam.get("camera_info_url", "file:///home/ares/.ros/camera_info/center.yaml")),
            "camera_frame_id": str(cam.get("camera_frame_id", f"{cam_name}_camera_optical_frame")),
            "image_size": cam.get("image_size", [1920, 1080]),
            "pixel_format": str(cam.get("pixel_format", "YUYV")),
            "output_encoding": str(cam.get("output_encoding", "yuv422_yuy2")),
        }

        nodes.append(
            Node(
                package="v4l2_camera",
                executable="v4l2_camera_node",
                namespace=cam_name,
                name="camera",
                parameters=[v4l2_params],
                output="screen",
            )
        )

        nodes.append(
            Node(
                package="image_proc",
                executable="rectify_node",
                namespace=cam_name,
                name="rectify",
                remappings=[
                    ("image", "image_raw"),
                    ("camera_info", "camera_info"),
                ],
                output="screen",
            )
        )

        tag_prefix = str(cam.get("tag_frame_prefix", f"{cam_name}_tag_"))
        frames = [f"{tag_prefix}{int(i)}" for i in tag_ids]
        sizes = [float(apr.get("size_m", 0.15)) for _ in tag_ids]

        apriltag_params = {
            "image_transport": str(apr.get("image_transport", "raw")),
            "family": str(apr.get("family", "36h11")),
            "size": float(apr.get("size_m", 0.15)),
            "max_hamming": int(apr.get("max_hamming", 0)),
            "profile": bool(apr.get("profile", False)),
            "pose_estimation_method": str(apr.get("pose_estimation_method", "pnp")),
            "tag": {"ids": [int(i) for i in tag_ids], "frames": frames, "sizes": sizes},
        }

        nodes.append(
            Node(
                package="apriltag_ros",
                executable="apriltag_node",
                namespace=cam_name,
                name="apriltag",
                parameters=[apriltag_params],
                output="screen",
            )
        )

    nodes.append(
        Node(
            package="rover_apriltag",
            executable="tag_fuser",
            name="tag_fuser",
            parameters=[{"config_path": config_path}],
            output="screen",
        )
    )

    return nodes

def generate_launch_description() -> LaunchDescription:
    default_config = os.path.join(
        get_package_share_directory("rover_apriltag"),
        "config",
        "rover_apriltag.yaml",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value=default_config,
                description="Path to the central rover_apriltag YAML config file.",
            ),
            OpaqueFunction(function=_make_nodes),
        ]
    )
