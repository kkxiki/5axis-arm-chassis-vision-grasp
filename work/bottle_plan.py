from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable

from .models import Pose3D


class BottlePlanError(ValueError):
    pass


def quaternion_rotation(qx: float, qy: float, qz: float, qw: float) -> list[list[float]]:
    norm = qx * qx + qy * qy + qz * qz + qw * qw
    if norm < 1e-12:
        raise BottlePlanError("四元数长度为零")
    s = 2.0 / norm
    return [
        [1.0 - s * (qy * qy + qz * qz), s * (qx * qy - qz * qw), s * (qx * qz + qy * qw)],
        [s * (qx * qy + qz * qw), 1.0 - s * (qx * qx + qz * qz), s * (qy * qz - qx * qw)],
        [s * (qx * qz - qy * qw), s * (qy * qz + qx * qw), 1.0 - s * (qx * qx + qy * qy)],
    ]


def _mat_vec(rotation: list[list[float]], point: Iterable[float]) -> tuple[float, float, float]:
    p = tuple(float(v) for v in point)
    if len(p) != 3:
        raise BottlePlanError("三维点必须包含 3 个数值")
    return tuple(sum(rotation[row][col] * p[col] for col in range(3)) for row in range(3))


def camera_point_to_arm_base(
    point_camera_m: Iterable[float],
    base_tcp_pose: Iterable[float],
    tcp_t_camera: list[list[float]],
) -> tuple[float, float, float]:
    """Apply base_T_tcp @ tcp_T_camera @ point_camera for the eye-in-hand camera."""
    tcp = tuple(float(v) for v in base_tcp_pose)
    if len(tcp) != 7:
        raise BottlePlanError("实时 TCP 位姿必须为 [x,y,z,qx,qy,qz,qw]")
    if len(tcp_t_camera) != 4 or any(len(row) != 4 for row in tcp_t_camera):
        raise BottlePlanError("tcp_T_camera 必须为 4x4 矩阵")
    p_camera = tuple(float(v) for v in point_camera_m)
    if len(p_camera) != 3 or any(not math.isfinite(v) for v in p_camera):
        raise BottlePlanError("相机三维点无效")
    p_tcp_rot = _mat_vec([row[:3] for row in tcp_t_camera[:3]], p_camera)
    p_tcp = tuple(p_tcp_rot[i] + float(tcp_t_camera[i][3]) for i in range(3))
    base_r_tcp = quaternion_rotation(*tcp[3:7])
    p_base_rot = _mat_vec(base_r_tcp, p_tcp)
    return tuple(p_base_rot[i] + tcp[i] for i in range(3))


def pixel_depth_to_camera(
    u: float,
    v: float,
    depth_m: float,
    camera_matrix: list[list[float]],
) -> tuple[float, float, float]:
    if not (math.isfinite(depth_m) and 0.05 <= depth_m <= 2.0):
        raise BottlePlanError(f"深度值 {depth_m!r}m 超出允许范围")
    fx, fy = float(camera_matrix[0][0]), float(camera_matrix[1][1])
    cx, cy = float(camera_matrix[0][2]), float(camera_matrix[1][2])
    if fx <= 0 or fy <= 0:
        raise BottlePlanError("相机内参焦距无效")
    return ((float(u) - cx) * depth_m / fx, (float(v) - cy) * depth_m / fy, depth_m)


def pixel_ray_to_arm_base_plane(
    u: float,
    v: float,
    target_z_base_m: float,
    camera_matrix: list[list[float]],
    base_tcp_pose: Iterable[float],
    tcp_t_camera: list[list[float]],
) -> tuple[float, float, float]:
    """Intersect an RGB pixel ray with a fixed Z plane in arm_base.

    The input pixel should be undistorted before this function when lens
    distortion is material. This is the depth-free fallback for a fixed table.
    """
    tcp = tuple(float(value) for value in base_tcp_pose)
    if len(tcp) != 7:
        raise BottlePlanError("实时 TCP 位姿必须为 7 个数值")
    fx, fy = float(camera_matrix[0][0]), float(camera_matrix[1][1])
    cx, cy = float(camera_matrix[0][2]), float(camera_matrix[1][2])
    ray_camera = ((float(u) - cx) / fx, (float(v) - cy) / fy, 1.0)
    base_r_tcp = quaternion_rotation(*tcp[3:7])
    tcp_r_camera = [row[:3] for row in tcp_t_camera[:3]]
    ray_tcp = _mat_vec(tcp_r_camera, ray_camera)
    ray_base = _mat_vec(base_r_tcp, ray_tcp)
    camera_origin_base = camera_point_to_arm_base((0.0, 0.0, 0.0), tcp, tcp_t_camera)
    if abs(ray_base[2]) < 1e-8:
        raise BottlePlanError("像素射线与固定高度平面平行")
    scale = (float(target_z_base_m) - camera_origin_base[2]) / ray_base[2]
    if scale <= 0:
        raise BottlePlanError("固定高度平面位于相机射线后方")
    return (
        camera_origin_base[0] + scale * ray_base[0],
        camera_origin_base[1] + scale * ray_base[1],
        float(target_z_base_m),
    )


@dataclass(frozen=True)
class BottlePickPlan:
    observe: Pose3D
    above: Pose3D
    grasp: Pose3D
    lift: Pose3D
    place_above: Pose3D
    place: Pose3D
    home: Pose3D
    detection_base_m: tuple[float, float, float]
    compensation_delta_m: tuple[float, float, float]
    place_count: int = 0
    place_offset_m: tuple[float, float, float] = (0.0, 0.0, 0.0)

    def to_dict(self) -> dict[str, Any]:
        return {
            "detection_base_m": list(self.detection_base_m),
            "compensation_delta_m": list(self.compensation_delta_m),
            "place_count": self.place_count,
            "place_offset_m": list(self.place_offset_m),
            "poses": {
                "observe": self.observe.to_list(),
                "above": self.above.to_list(),
                "grasp": self.grasp.to_list(),
                "lift": self.lift.to_list(),
                "place_above": self.place_above.to_list(),
                "place": self.place.to_list(),
                "home": self.home.to_list(),
            },
            "sequence": [
                "observe", "detect", "open", "above", "grasp", "close",
                "lift", "place_above", "place", "open", "place_above", "home",
            ],
        }


def _confirmed_pose(poses: dict[str, Any], name: str, robot_model: str) -> Pose3D:
    value = poses.get(name)
    if not isinstance(value, dict):
        raise BottlePlanError(f"缺少 V4 示教点: {name}")
    if value.get("confirmed") is not True:
        raise BottlePlanError(f"V4 示教点尚未确认: {name}")
    if value.get("robot_model") != robot_model:
        raise BottlePlanError(
            f"{name} 的 robot_model={value.get('robot_model')!r}，应为 {robot_model!r}"
        )
    raw = value.get("robottarget")
    if not isinstance(raw, list) or len(raw) != 7:
        raise BottlePlanError(f"{name}.robottarget 必须包含 7 个数值")
    return Pose3D.from_list(raw)


def place_offset_for_count(cfg: dict[str, Any], place_count: int) -> tuple[float, float, float]:
    if place_count < 0:
        raise BottlePlanError("放置区水瓶数量不能为负数")
    rule = cfg.get("place_offset_rule", {})
    if place_count == 0:
        dx = 0.0
    elif place_count == 1:
        dx = float(rule.get("one_bottle_dx_m", 0.125))
    else:
        dx = float(rule.get("two_or_more_bottles_dx_m", 0.25))
    return (dx, 0.0, 0.0)


def build_bottle_pick_plan(
    cfg: dict[str, Any],
    target_base_m: Iterable[float],
    place_count: int = 0,
) -> BottlePickPlan:
    target = tuple(float(v) for v in target_base_m)
    if len(target) != 3 or any(not math.isfinite(v) for v in target):
        raise BottlePlanError("水瓶 arm_base 坐标无效")
    robot_model = str(cfg["robot_model"])
    poses = cfg["poses"]
    required = ("observe", "above_ref", "grasp_ref", "place_above", "place", "home")
    loaded = {name: _confirmed_pose(poses, name, robot_model) for name in required}
    reference = poses["grasp_ref"].get("ref_detection_base_m")
    if not isinstance(reference, list) or len(reference) != 3:
        raise BottlePlanError("grasp_ref 缺少示教时的 ref_detection_base_m")
    delta = tuple(target[i] - float(reference[i]) for i in range(3))
    grasp = loaded["grasp_ref"].shifted(
        dx=delta[0], dy=delta[1], dz=delta[2] + float(cfg.get("grasp_z_extra_m", 0.0))
    )
    above = loaded["above_ref"].shifted(*delta)
    lift = grasp.shifted(dz=float(cfg["lift_height_m"]))
    place_offset = place_offset_for_count(cfg, int(place_count))
    place_above = loaded["place_above"].shifted(*place_offset)
    place = loaded["place"].shifted(*place_offset)
    return BottlePickPlan(
        loaded["observe"], above, grasp, lift, place_above,
        place, loaded["home"], target, delta, int(place_count), place_offset,
    )


def load_handeye(path: str | Path) -> dict[str, Any]:
    value = json.loads(Path(path).read_text(encoding="utf-8"))
    if value.get("mode") != "eye_in_hand" or value.get("image_size") != [1280, 720]:
        raise BottlePlanError("V4 手眼标定必须为 eye_in_hand 且图像尺寸为 1280x720")
    if not value.get("camera_to_tcp_matrix") or not value.get("camera_matrix"):
        raise BottlePlanError("V4 手眼标定缺少矩阵或相机内参")
    return value
