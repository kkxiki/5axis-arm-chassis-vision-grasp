"""全局共用数据结构定义模块。

把整个 demo 通用的数据结构集中放在这里，供任务层、设备层和日志模块共享：
- Pose2D / Pose3D：底盘二维位姿与机械臂三维 6D 位姿；
- Detection：一次视觉检测的结果（前视粗定位与腕部精定位共用同一结构）；
- Stage：任务状态机枚举；MissionResult：一次任务的最终结果。

本模块只定义数据结构，不含任何硬件通信或算法逻辑。
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
from enum import Enum
from typing import Any


@dataclass(frozen=True)
class Pose2D:
    """底盘在 map 平面上的二维位姿（x, y, yaw）。

    使用 frozen=True，实例创建后不可修改；需要改动时用新实例替换。
    """

    x: float  # 地图坐标系下的 x 坐标，单位 m
    y: float  # 地图坐标系下的 y 坐标，单位 m
    yaw: float = 0.0  # 航向角，单位 rad；以地图 x 轴正方向为 0，逆时针为正

    def to_dict(self) -> dict[str, float]:
        """转成可直接 JSON 序列化的字典，键名与字段名一致。"""
        return asdict(self)


@dataclass(frozen=True)
class Pose3D:
    """机械臂的 6D 位姿：位置 + 姿态四元数。

    位置是机械臂基座（arm_base）坐标系下的 x/y/z（单位 m），
    姿态用四元数表示，顺序固定为 [qx, qy, qz, qw]（虚部在前、实部 w 在最后）。
    """

    x: float  # 位置 x，单位 m
    y: float  # 位置 y，单位 m
    z: float  # 位置 z，单位 m
    qx: float = 0.0  # 四元数虚部 x
    qy: float = 1.0  # 四元数虚部 y；默认值 (0,1,0,0) 表示绕 y 轴旋转 180°
    qz: float = 0.0  # 四元数虚部 z
    qw: float = 0.0  # 四元数实部 w

    @classmethod
    def from_list(cls, values: list[float]) -> "Pose3D":
        """由 [x,y,z,qx,qy,qz,qw] 这 7 个数值构造 Pose3D。

        元素个数不是 7 时抛 ValueError；所有元素都会被转成 float。
        """
        if len(values) != 7:
            raise ValueError("Pose3D requires [x,y,z,qx,qy,qz,qw]")
        return cls(*map(float, values))

    def to_list(self) -> list[float]:
        """按 [x,y,z,qx,qy,qz,qw] 的顺序输出列表，用于下发指令或写日志。"""
        return [self.x, self.y, self.z, self.qx, self.qy, self.qz, self.qw]

    def shifted(self, dx: float = 0.0, dy: float = 0.0, dz: float = 0.0) -> "Pose3D":
        """保持姿态不变、只做位置平移，返回新的 Pose3D 实例。

        参数 dx/dy/dz 为三个方向上的平移量（单位 m），dz 为正值表示向上抬升。
        """
        return Pose3D(self.x + dx, self.y + dy, self.z + dz,
                      self.qx, self.qy, self.qz, self.qw)


@dataclass(frozen=True)
class Detection:
    """一次视觉检测的结果（前视相机粗定位与腕部相机精定位共用）。

    bearing_rad 与 distance_m 是两个相机都会给的信息；position_base_m 只有
    腕部相机（已做手眼标定）才会给，因此默认为 None。
    """

    label: str  # 目标类别名，例如 "bottle"
    confidence: float  # 检测置信度，取值范围 0~1
    bearing_rad: float  # 目标相对相机光轴的方位角，单位 rad；正值表示目标在画面中心右侧（详见 front_geometry.bearing_from_pixel）
    distance_m: float  # 目标距离，单位 m；由深度图 ROI 取中位数得到，是沿相机光轴方向的深度值
    position_base_m: tuple[float, float, float] | None = None  # 目标在 arm_base 系下的 [x,y,z]（单位 m），仅腕部精定位会返回
    frame_id: str = ""  # 检测结果所在坐标系名称，如前视的 "camera_front"、腕部的 "arm_base"/"base_link"
    timestamp_s: float = 0.0  # 检测时刻，Unix 时间戳（秒），用于判断结果是否过期

    def to_dict(self) -> dict[str, Any]:
        """转成可 JSON 序列化的字典；会把 position_base_m 元组转成列表。"""
        value = asdict(self)
        if self.position_base_m is not None:
            value["position_base_m"] = list(self.position_base_m)
        return value


class Stage(str, Enum):
    """任务状态机：主流程按下面顺序推进，出错时进入 SAFE_ABORT。

    同时继承 str，因此可以直接当作字符串写入日志（如 stage_enter 事件）。
    """

    CREATED = "CREATED"  # 任务对象已创建，尚未开始
    PREFLIGHT = "PREFLIGHT"  # 预检：检查底盘/相机/机械臂/夹爪健康状态与底盘定位
    NAVIGATING = "NAVIGATING"  # 底盘导航到地图中的目标附近
    FRONT_SEARCH = "FRONT_SEARCH"  # 前视相机粗定位，找出水瓶类别、方位与距离
    BASE_ALIGN = "BASE_ALIGN"  # 底盘按方位角转向并前进，使水瓶处于腕部相机工作范围内
    WRIST_LOCALIZE = "WRIST_LOCALIZE"  # 腕部相机精定位，得到 arm_base 系下的三维坐标
    PICK = "PICK"  # 机械臂抓取水瓶
    PLACE = "PLACE"  # 把水瓶放到指定放置点
    COMPLETE = "COMPLETE"  # 抓取与放置均成功完成
    SAFE_ABORT = "SAFE_ABORT"  # 任意环节异常后的安全中止（停止运动并记录失败原因）


@dataclass
class MissionResult:
    """一次抓取任务的最终结果，用于日志记录和向上层汇报。

    注意这是可变的普通 dataclass（不是 frozen），由任务层在结束时构造一次。
    """

    mission_id: str  # 任务唯一标识，形如 v5-20260910-110558-edff0e
    success: bool  # 任务是否成功完成
    final_stage: Stage  # 任务结束时所处的状态（成功时为 COMPLETE，失败时为出错时的状态/SAFE_ABORT）
    message: str  # 结果说明：成功提示或失败原因
    log_file: str  # 本次任务的 jsonl 日志文件路径
    target_base_m: tuple[float, float, float] | None = None  # 最终使用的目标点（arm_base 系，单位 m）；未定位到目标时为 None

    def to_dict(self) -> dict[str, Any]:
        """转成可 JSON 序列化的字典；final_stage 取字符串值，target_base_m 转成列表。"""
        value = asdict(self)
        value["final_stage"] = self.final_stage.value
        if self.target_base_m is not None:
            value["target_base_m"] = list(self.target_base_m)
        return value

