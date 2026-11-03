"""水瓶抓取计划的几何计算模块。

负责三件事：
1. 坐标变换：把视觉检测结果从「相机坐标系」换算到「机械臂基座坐标系」。
2. 生成抓取点位：在示教参考点位基础上做平移补偿，得到本次抓取的完整点位序列。
3. 加载并校验手眼标定文件。

坐标系约定（贯穿全文件）：
    camera  —— 相机坐标系，Z 轴朝前指向被拍摄物体
    tcp     —— 机械臂末端法兰（工具中心点）坐标系
    arm_base—— 机械臂底座坐标系，也是最终下发运动指令所用的坐标系
"""
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
    """把四元数 (qx,qy,qz,qw) 转成 3x3 旋转矩阵。

    这里的 s = 2/|q|²（而不是 2）表示公式内部已包含归一化，
    因此即使传入的四元数没有归一化，结果依然是正确的旋转矩阵。
    """
    norm = qx * qx + qy * qy + qz * qz + qw * qw
    if norm < 1e-12:
        raise BottlePlanError("四元数长度为零")
    s = 2.0 / norm
    # 每行代表旋转后坐标轴在各基底上的分量：
    # 第 0/1/2 行分别给出 base 系下 x、y、z 三个方向的旋转结果
    return [
        [1.0 - s * (qy * qy + qz * qz), s * (qx * qy - qz * qw), s * (qx * qz + qy * qw)],
        [s * (qx * qy + qz * qw), 1.0 - s * (qx * qx + qz * qz), s * (qy * qz - qx * qw)],
        [s * (qx * qz - qy * qw), s * (qy * qz + qx * qw), 1.0 - s * (qx * qx + qy * qy)],
    ]


def _mat_vec(rotation: list[list[float]], point: Iterable[float]) -> tuple[float, float, float]:
    """3x3 旋转矩阵左乘三维列向量，即 rotation @ point。

    只做旋转、不含平移；平移需要调用方自己再加（见下面两个函数的用法）。
    """
    p = tuple(float(v) for v in point)
    if len(p) != 3:
        raise BottlePlanError("三维点必须包含 3 个数值")
    # 第 row 行 = 矩阵第 row 行与向量点积
    return tuple(sum(rotation[row][col] * p[col] for col in range(3)) for row in range(3))


def camera_point_to_arm_base(
    point_camera_m: Iterable[float],
    base_tcp_pose: Iterable[float],
    tcp_t_camera: list[list[float]],
) -> tuple[float, float, float]:
    """把相机坐标系下的一个点换算到机械臂基座坐标系（眼在手/eye-in-hand）。

    整体就是在做一次齐次矩阵连乘：

        P_arm_base = base_T_tcp @ tcp_T_camera @ P_camera

    其中 base_T_tcp 由「机械臂实时返回的 TCP 位姿」现场拼出来，
    tcp_T_camera 是固定的手眼标定矩阵。分两步走是因为第一步只用到
    旋转+平移即可，不必真的构造 4x4 齐次矩阵。
    """
    # tcp 为 [x, y, z, qx, qy, qz, qw]：前 3 个是位置(m)，后 4 个是姿态四元数
    tcp = tuple(float(v) for v in base_tcp_pose)
    if len(tcp) != 7:
        raise BottlePlanError("实时 TCP 位姿必须为 [x,y,z,qx,qy,qz,qw]")
    if len(tcp_t_camera) != 4 or any(len(row) != 4 for row in tcp_t_camera):
        raise BottlePlanError("tcp_T_camera 必须为 4x4 矩阵")
    p_camera = tuple(float(v) for v in point_camera_m)
    if len(p_camera) != 3 or any(not math.isfinite(v) for v in p_camera):
        raise BottlePlanError("相机三维点无效")

    # 第一步：相机 -> TCP。用 4x4 矩阵的左上 3x3 做旋转，第 4 列做平移
    p_tcp_rot = _mat_vec([row[:3] for row in tcp_t_camera[:3]], p_camera)
    p_tcp = tuple(p_tcp_rot[i] + float(tcp_t_camera[i][3]) for i in range(3))

    # 第二步：TCP -> 基座。用 TCP 四元数构造旋转，位置直接取 tcp 的前 3 个分量
    base_r_tcp = quaternion_rotation(*tcp[3:7])
    p_base_rot = _mat_vec(base_r_tcp, p_tcp)
    return tuple(p_base_rot[i] + tcp[i] for i in range(3))


def pixel_depth_to_camera(
    u: float,
    v: float,
    depth_m: float,
    camera_matrix: list[list[float]],
) -> tuple[float, float, float]:
    """已知像素坐标 (u,v) 和该点深度，反投影得到相机坐标系下的三维点。

    这是标准针孔相机模型的逆运算：
        X = (u - cx) * depth / fx
        Y = (v - cy) * depth / fy
        Z = depth
    camera_matrix 取自标定结果，形如 [[fx,0,cx],[0,fy,cy],[0,0,1]]。
    """
    # 深度限制在 5cm ~ 2m：太小说明是噪声或贴到镜头，太大说明检测不可信
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
    """把 RGB 像素发出的射线与「基座系下某个固定 Z 高度平面」求交点。

    用途：没有可靠深度时的兜底方案（例如桌面高度已知且固定）。

    原理：像素 (u,v) 在相机系中唯一确定一条射线（原点在光心），
    把这条射线转到基座系后，求它与平面 z = target_z_base_m 的交点：

        P = O + scale * D,   scale = (target_z - O_z) / D_z

    其中 O 是相机光心在基座系的位置，D 是射线方向。
    传入的像素应先做去畸变（镜头畸变明显时），本函数不处理畸变。
    """
    tcp = tuple(float(value) for value in base_tcp_pose)
    if len(tcp) != 7:
        raise BottlePlanError("实时 TCP 位姿必须为 7 个数值")
    fx, fy = float(camera_matrix[0][0]), float(camera_matrix[1][1])
    cx, cy = float(camera_matrix[0][2]), float(camera_matrix[1][2])
    # 相机系下的射线方向：Z 归一化为 1，(u-cx)/fx 与 (v-cy)/fy 是归一化平面上的偏移
    ray_camera = ((float(u) - cx) / fx, (float(v) - cy) / fy, 1.0)
    base_r_tcp = quaternion_rotation(*tcp[3:7])
    tcp_r_camera = [row[:3] for row in tcp_t_camera[:3]]
    # 方向向量只做旋转（不平移）：相机 -> TCP -> 基座
    ray_tcp = _mat_vec(tcp_r_camera, ray_camera)
    ray_base = _mat_vec(base_r_tcp, ray_tcp)
    # 光心 = 相机系原点变换到基座系，即射线的起点
    camera_origin_base = camera_point_to_arm_base((0.0, 0.0, 0.0), tcp, tcp_t_camera)
    # 射线方向在 Z 上无分量 -> 与水平面平行，永远不相交
    if abs(ray_base[2]) < 1e-8:
        raise BottlePlanError("像素射线与固定高度平面平行")
    scale = (float(target_z_base_m) - camera_origin_base[2]) / ray_base[2]
    # scale <= 0 表示交点落在相机后方，属于几何异常
    if scale <= 0:
        raise BottlePlanError("固定高度平面位于相机射线后方")
    # 交点的 Z 直接取目标平面高度，避免浮点误差带来的偏差
    return (
        camera_origin_base[0] + scale * ray_base[0],
        camera_origin_base[1] + scale * ray_base[1],
        float(target_z_base_m),
    )


@dataclass(frozen=True)
class BottlePickPlan:
    """一次完整抓取所需的全部机械臂点位（均为 arm_base 系下的 6D 位姿）。

    点位按机械臂实际运动顺序排列：

        observe      观察位：抬起腕部相机俯视目标，用于二次精定位
        above        目标正上方略高的接近位（避免斜插碰撞）
        grasp        抓取位：夹爪正好抱住水瓶
        lift         抬升位：夹住后向上抬起，脱离桌面
        place_above  放置点正上方
        place        放置位
        home         收工回零位

    另外记录本次实际检测到的目标位置，以及相对示教点的补偿量，便于复现问题。
    """

    observe: Pose3D
    above: Pose3D
    grasp: Pose3D
    lift: Pose3D
    place_above: Pose3D
    place: Pose3D
    home: Pose3D
    detection_base_m: tuple[float, float, float]
    compensation_delta_m: tuple[float, float, float]

    def to_dict(self) -> dict[str, Any]:
        # sequence 是给上层执行器用的动作序列：
        # 注意 place 之后又回到 place_above，是为了在放置后让夹爪张开并让开
        return {
            "detection_base_m": list(self.detection_base_m),
            "compensation_delta_m": list(self.compensation_delta_m),
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
    """从配置里取出一个「已确认」的示教点位，并做三重校验。

    校验项分别是：存在、已人工确认（confirmed=True）、机型一致。
    机型校验是为了防止把 A 型号机械臂示教出来的点误用到 B 型号上。
    """
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


def build_bottle_pick_plan(cfg: dict[str, Any], target_base_m: Iterable[float]) -> BottlePickPlan:
    """根据本次检测到的目标位置，生成一次抓取的完整点位计划。

    核心思路是「示教 + 平移补偿」，而不是现场做完整的运动学求解：

    1. 读出肉眼示教好的参考点（尤其 grasp_ref，它记录了示教当时的目标位置
       ref_detection_base_m）；
    2. 计算本次目标与示教目标的偏移量 delta；
    3. 把参考点整体平移 delta，就得到本次的 above / grasp 等点位。
       抓取点额外加一个 grasp_z_extra_m 的高度余量，防止插得太深撞到桌面。

    这样姿态保持示教时的姿态不变，只调整位置，简单且可控。
    """
    target = tuple(float(v) for v in target_base_m)
    if len(target) != 3 or any(not math.isfinite(v) for v in target):
        raise BottlePlanError("水瓶 arm_base 坐标无效")
    robot_model = str(cfg["robot_model"])
    poses = cfg["poses"]
    required = ("observe", "above_ref", "grasp_ref", "place_above", "place", "home")
    loaded = {name: _confirmed_pose(poses, name, robot_model) for name in required}
    # 示教抓取点当时对应的目标位置，是计算偏移量的基准
    reference = poses["grasp_ref"].get("ref_detection_base_m")
    if not isinstance(reference, list) or len(reference) != 3:
        raise BottlePlanError("grasp_ref 缺少示教时的 ref_detection_base_m")
    # delta = 本次目标 - 示教目标，三个方向上的平移量
    delta = tuple(target[i] - float(reference[i]) for i in range(3))
    # 抓取点：水平方向按 delta 平移，Z 方向再抬高一个余量
    grasp = loaded["grasp_ref"].shifted(
        dx=delta[0], dy=delta[1], dz=delta[2] + float(cfg.get("grasp_z_extra_m", 0.0))
    )
    # 接近点：与抓取点同姿态，整体平移同样的 delta
    above = loaded["above_ref"].shifted(*delta)
    # 抬升点：在抓取点基础上垂直向上抬高 lift_height_m
    lift = grasp.shifted(dz=float(cfg["lift_height_m"]))
    return BottlePickPlan(
        loaded["observe"], above, grasp, lift, loaded["place_above"],
        loaded["place"], loaded["home"], target, delta,
    )


def load_handeye(path: str | Path) -> dict[str, Any]:
    """读取手眼标定文件，并校验它符合本 demo 的硬性前提。

    要求：模式必须是 eye_in_hand（相机装在末端），图像尺寸必须为 1280x720
    （与检测服务、像素坐标约定一致），且必须同时提供 tcp_T_camera 矩阵和相机内参。
    """
    value = json.loads(Path(path).read_text(encoding="utf-8"))
    if value.get("mode") != "eye_in_hand" or value.get("image_size") != [1280, 720]:
        raise BottlePlanError("V4 手眼标定必须为 eye_in_hand 且图像尺寸为 1280x720")
    if not value.get("camera_to_tcp_matrix") or not value.get("camera_matrix"):
        raise BottlePlanError("V4 手眼标定缺少矩阵或相机内参")
    return value
