import os
from typing import List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from rover_apriltag.config_utils import (
    camera_frame_id,
    camera_names,
    camera_namespace,
    load_config,
    require_bool,
    require_float,
    require_int,
    require_list,
    require_positive_float,
    require_str,
    resolve_camera_info_url,
    rover_name,
    tag_frame_id,
)


def _make_nodes(context, *args, **kwargs):
    config_path = os.path.expanduser(LaunchConfiguration("config").perform(context))
    cfg = load_config(config_path)

    rover = rover_name(cfg)
    tag_ids = [int(tag_id) for tag_id in cfg["target"]["tag_ids"]]
    tag_family = require_str(cfg, ["target", "tag_family"], "target.tag_family")
    tag_size_m = require_positive_float(cfg, ["target", "tag_size_m"], "target.tag_size_m")

    image_transport = require_str(cfg, ["apriltag", "image_transport"], "apriltag.image_transport")
    qos_profile = require_str(cfg, ["apriltag", "qos_profile"], "apriltag.qos_profile")
    max_hamming = require_int(cfg, ["apriltag", "max_hamming"], "apriltag.max_hamming")
    pose_estimation_method = require_str(
        cfg,
        ["apriltag", "pose_estimation_method"],
        "apriltag.pose_estimation_method",
    )
    detector_threads = require_int(cfg, ["apriltag", "detector", "threads"], "apriltag.detector.threads")
    detector_decimate = require_positive_float(
        cfg,
        ["apriltag", "detector", "decimate"],
        "apriltag.detector.decimate",
    )
    detector_blur = require_float(cfg, ["apriltag", "detector", "blur"], "apriltag.detector.blur")
    detector_refine = require_bool(cfg, ["apriltag", "detector", "refine"], "apriltag.detector.refine")
    detector_sharpening = require_float(
        cfg,
        ["apriltag", "detector", "sharpening"],
        "apriltag.detector.sharpening",
    )
    detector_debug = require_bool(cfg, ["apriltag", "detector", "debug"], "apriltag.detector.debug")

    nodes: List[Node] = []

    for camera_name in camera_names(cfg):
        namespace = camera_namespace(cfg, camera_name)
        video_device = require_str(cfg, ["cameras", camera_name, "device"], f"cameras.{camera_name}.device")
        calibration = require_str(cfg, ["cameras", camera_name, "calibration"], f"cameras.{camera_name}.calibration")
        image_size = require_list(cfg, ["cameras", camera_name, "image_size"], f"cameras.{camera_name}.image_size")
        pixel_format = require_str(cfg, ["cameras", camera_name, "pixel_format"], f"cameras.{camera_name}.pixel_format")
        output_encoding = require_str(
            cfg,
            ["cameras", camera_name, "output_encoding"],
            f"cameras.{camera_name}.output_encoding",
        )

        v4l2_params = {
            "video_device": video_device,
            "framerate": 1.0,
            "camera_info_url": resolve_camera_info_url(config_path, calibration),
            "camera_frame_id": camera_frame_id(cfg, camera_name),
            "image_size": image_size,
            "pixel_format": pixel_format,
            "output_encoding": output_encoding,
        }

        apriltag_params = {
            "image_transport": image_transport,
            "qos_profile": qos_profile,
            "family": tag_family,
            "size": tag_size_m,
            "max_hamming": max_hamming,
            "pose_estimation_method": pose_estimation_method,
            "detector": {
                "threads": detector_threads,
                "decimate": detector_decimate,
                "blur": detector_blur,
                "refine": detector_refine,
                "sharpening": detector_sharpening,
                "debug": detector_debug,
            },
            "tag": {
                "ids": tag_ids,
                "frames": [tag_frame_id(cfg, camera_name, tag_id) for tag_id in tag_ids],
                "sizes": [tag_size_m for _ in tag_ids],
            },
        }

        nodes.append(
            Node(
                package="v4l2_camera",
                executable="v4l2_camera_node",
                namespace=namespace,
                name="camera",
                parameters=[v4l2_params],
                output="screen",
            )
        )
        nodes.append(
            Node(
                package="image_proc",
                executable="rectify_node",
                namespace=namespace,
                name="rectify",
                remappings=[("image", "image_raw"), ("camera_info", "camera_info")],
                output="screen",
            )
        )
        nodes.append(
            Node(
                package="apriltag_ros",
                executable="apriltag_node",
                namespace=namespace,
                name="apriltag",
                parameters=[apriltag_params],
                output="screen",
            )
        )

    nodes.append(
        Node(
            package="rover_apriltag",
            executable="target_locator",
            namespace=rover,
            name="target_locator",
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
            DeclareLaunchArgument("config", default_value=default_config),
            OpaqueFunction(function=_make_nodes),
        ]
    )
