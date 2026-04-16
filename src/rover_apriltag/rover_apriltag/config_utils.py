import os
import re
from pathlib import Path
from typing import Any, Dict, List

import yaml

Config = Dict[str, Any]


_ROVER_NAME_RE = re.compile(r"^[A-Za-z0-9_]+$")
_ALLOWED_IMAGE_TRANSPORTS = {"raw", "compressed"}
_ALLOWED_QOS_PROFILES = {"default", "sensor_data", "system_default"}
_ALLOWED_POSE_ESTIMATION_METHODS = {"pnp", "homography"}


def load_config(config_path: str) -> Config:
    path = os.path.expanduser(config_path)
    with open(path, "r", encoding="utf-8") as stream:
        cfg = yaml.safe_load(stream)
    if not isinstance(cfg, dict):
        raise RuntimeError(f"Config file did not parse as a dictionary: {path}")
    validate_config(cfg)
    return cfg


def require(cfg: Config, keys: List[str], path: str) -> Any:
    cur: Any = cfg
    for key in keys:
        if not isinstance(cur, dict) or key not in cur:
            raise RuntimeError(f"Missing required config key: {path}")
        cur = cur[key]
    return cur


def require_dict(cfg: Config, keys: List[str], path: str) -> Config:
    value = require(cfg, keys, path)
    if not isinstance(value, dict):
        raise RuntimeError(f"Config key must be a dictionary: {path}")
    return value


def require_non_empty_dict(cfg: Config, keys: List[str], path: str) -> Config:
    value = require_dict(cfg, keys, path)
    if not value:
        raise RuntimeError(f"Config dictionary cannot be empty: {path}")
    return value


def require_str(cfg: Config, keys: List[str], path: str) -> str:
    value = require(cfg, keys, path)
    if not isinstance(value, str) or not value.strip():
        raise RuntimeError(f"Config key must be a non-empty string: {path}")
    return value.strip()


def require_rover_name(cfg: Config, keys: List[str], path: str) -> str:
    value = require_str(cfg, keys, path)
    if _ROVER_NAME_RE.fullmatch(value) is None:
        raise RuntimeError(
            f"Config key must match [A-Za-z0-9_]+ because it is used in ROS names and TF frame IDs: {path}"
        )
    return value


def require_bool(cfg: Config, keys: List[str], path: str) -> bool:
    value = require(cfg, keys, path)
    if not isinstance(value, bool):
        raise RuntimeError(f"Config key must be a boolean: {path}")
    return value


def require_int(cfg: Config, keys: List[str], path: str) -> int:
    value = require(cfg, keys, path)
    if isinstance(value, bool) or not isinstance(value, int):
        raise RuntimeError(f"Config key must be an integer: {path}")
    return value


def require_float(cfg: Config, keys: List[str], path: str) -> float:
    value = require(cfg, keys, path)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise RuntimeError(f"Config key must be a number: {path}")
    return float(value)


def require_positive_float(cfg: Config, keys: List[str], path: str) -> float:
    value = require_float(cfg, keys, path)
    if value <= 0.0:
        raise RuntimeError(f"Config key must be greater than zero: {path}")
    return value


def require_non_negative_float(cfg: Config, keys: List[str], path: str) -> float:
    value = require_float(cfg, keys, path)
    if value < 0.0:
        raise RuntimeError(f"Config key must be zero or greater: {path}")
    return value


def require_list(cfg: Config, keys: List[str], path: str) -> List[Any]:
    value = require(cfg, keys, path)
    if not isinstance(value, list):
        raise RuntimeError(f"Config key must be a list: {path}")
    return value


def require_int_list(cfg: Config, keys: List[str], path: str, *, non_empty: bool = False) -> List[int]:
    items = require_list(cfg, keys, path)
    result: List[int] = []
    for item in items:
        if isinstance(item, bool) or not isinstance(item, int):
            raise RuntimeError(f"Config key must contain only integers: {path}")
        result.append(int(item))
    if non_empty and not result:
        raise RuntimeError(f"Config list cannot be empty: {path}")
    if len(set(result)) != len(result):
        raise RuntimeError(f"Config list cannot contain duplicates: {path}")
    return result


def require_vector(cfg: Config, keys: List[str], path: str, *, length: int) -> List[float]:
    items = require_list(cfg, keys, path)
    if len(items) != length:
        raise RuntimeError(f"Config key must have length {length}: {path}")
    result: List[float] = []
    for item in items:
        if isinstance(item, bool) or not isinstance(item, (int, float)):
            raise RuntimeError(f"Config key must contain only numbers: {path}")
        result.append(float(item))
    return result


def rover_name(cfg: Config) -> str:
    return require_rover_name(cfg, ["rover", "name"], "rover.name")


def world_frame_id(cfg: Config) -> str:
    return require_str(cfg, ["rover", "world_frame_id"], "rover.world_frame_id")


def rover_frame_id(cfg: Config) -> str:
    return f"{rover_name(cfg)}_base"


def rover_pose_topic(cfg: Config) -> str:
    return f"/{rover_name(cfg)}/pose"


def camera_names(cfg: Config) -> List[str]:
    return list(require_non_empty_dict(cfg, ["cameras"], "cameras").keys())


def camera_namespace(cfg: Config, camera_name: str) -> str:
    return f"{rover_name(cfg)}/{camera_name}"


def camera_frame_id(cfg: Config, camera_name: str) -> str:
    return f"{rover_name(cfg)}_{camera_name}_camera_optical_frame"


def tag_frame_id(cfg: Config, camera_name: str, tag_id: int) -> str:
    return f"{rover_name(cfg)}_{camera_name}_tag_{int(tag_id)}"


def resolve_camera_info_url(config_path: str, calibration_value: str) -> str:
    if "://" in calibration_value:
        return calibration_value
    path = Path(os.path.expanduser(calibration_value))
    if not path.is_absolute():
        path = Path(os.path.expanduser(config_path)).resolve().parent / path
    return path.resolve().as_uri()


def validate_config(cfg: Config) -> None:
    require_rover_name(cfg, ["rover", "name"], "rover.name")
    require_str(cfg, ["rover", "world_frame_id"], "rover.world_frame_id")

    cameras = require_non_empty_dict(cfg, ["cameras"], "cameras")
    for camera_name, camera_cfg in cameras.items():
        if not isinstance(camera_cfg, dict):
            raise RuntimeError(f"Config key must be a dictionary: cameras.{camera_name}")
        require_str(cfg, ["cameras", camera_name, "device"], f"cameras.{camera_name}.device")
        require_str(cfg, ["cameras", camera_name, "calibration"], f"cameras.{camera_name}.calibration")
        image_size = require_list(cfg, ["cameras", camera_name, "image_size"], f"cameras.{camera_name}.image_size")
        if len(image_size) != 2:
            raise RuntimeError(f"Config key must have length 2: cameras.{camera_name}.image_size")
        for item in image_size:
            if isinstance(item, bool) or not isinstance(item, int) or item <= 0:
                raise RuntimeError(f"Config key must contain two positive integers: cameras.{camera_name}.image_size")
        require_str(cfg, ["cameras", camera_name, "pixel_format"], f"cameras.{camera_name}.pixel_format")
        require_str(cfg, ["cameras", camera_name, "output_encoding"], f"cameras.{camera_name}.output_encoding")
        require_vector(cfg, ["cameras", camera_name, "pose", "xyz_m"], f"cameras.{camera_name}.pose.xyz_m", length=3)
        require_vector(cfg, ["cameras", camera_name, "pose", "rpy_rad"], f"cameras.{camera_name}.pose.rpy_rad", length=3)

    target_tag_ids = require_int_list(cfg, ["target", "tag_ids"], "target.tag_ids", non_empty=True)
    if not target_tag_ids:
        raise RuntimeError("Config list cannot be empty: target.tag_ids")
    require_str(cfg, ["target", "tag_family"], "target.tag_family")
    tag_size_m = require_positive_float(cfg, ["target", "tag_size_m"], "target.tag_size_m")
    cube_size_m = require_positive_float(cfg, ["target", "cube_size_m"], "target.cube_size_m")
    if tag_size_m >= cube_size_m:
        raise RuntimeError("target.tag_size_m must be smaller than target.cube_size_m")
    max_jump_m = require_positive_float(cfg, ["target", "max_jump_m"], "target.max_jump_m")
    if max_jump_m >= cube_size_m:
        raise RuntimeError("target.max_jump_m must be smaller than target.cube_size_m")

    image_transport = require_str(cfg, ["apriltag", "image_transport"], "apriltag.image_transport")
    if image_transport not in _ALLOWED_IMAGE_TRANSPORTS:
        raise RuntimeError("apriltag.image_transport must be one of: raw, compressed")
    qos_profile = require_str(cfg, ["apriltag", "qos_profile"], "apriltag.qos_profile")
    if qos_profile not in _ALLOWED_QOS_PROFILES:
        raise RuntimeError("apriltag.qos_profile must be one of: default, sensor_data, system_default")
    if require_int(cfg, ["apriltag", "max_hamming"], "apriltag.max_hamming") < 0:
        raise RuntimeError("apriltag.max_hamming must be zero or greater")
    pose_estimation_method = require_str(
        cfg,
        ["apriltag", "pose_estimation_method"],
        "apriltag.pose_estimation_method",
    )
    if pose_estimation_method not in _ALLOWED_POSE_ESTIMATION_METHODS:
        raise RuntimeError("apriltag.pose_estimation_method must be one of: pnp, homography")
    if require_int(cfg, ["apriltag", "detector", "threads"], "apriltag.detector.threads") <= 0:
        raise RuntimeError("apriltag.detector.threads must be greater than zero")
    require_positive_float(cfg, ["apriltag", "detector", "decimate"], "apriltag.detector.decimate")
    require_non_negative_float(cfg, ["apriltag", "detector", "blur"], "apriltag.detector.blur")
    require_bool(cfg, ["apriltag", "detector", "refine"], "apriltag.detector.refine")
    require_float(cfg, ["apriltag", "detector", "sharpening"], "apriltag.detector.sharpening")
    require_bool(cfg, ["apriltag", "detector", "debug"], "apriltag.detector.debug")

    require_positive_float(cfg, ["pipeline", "publish_rate_hz"], "pipeline.publish_rate_hz")
    require_non_negative_float(cfg, ["pipeline", "max_tag_age_sec"], "pipeline.max_tag_age_sec")
