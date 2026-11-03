from __future__ import annotations

from dataclasses import asdict, dataclass
from enum import Enum
from typing import Any


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float = 0.0

    def to_dict(self) -> dict[str, float]:
        return asdict(self)


@dataclass(frozen=True)
class Pose3D:
    x: float
    y: float
    z: float
    qx: float = 0.0
    qy: float = 1.0
    qz: float = 0.0
    qw: float = 0.0

    @classmethod
    def from_list(cls, values: list[float]) -> "Pose3D":
        if len(values) != 7:
            raise ValueError("Pose3D requires [x,y,z,qx,qy,qz,qw]")
        return cls(*map(float, values))

    def to_list(self) -> list[float]:
        return [self.x, self.y, self.z, self.qx, self.qy, self.qz, self.qw]

    def shifted(self, dx: float = 0.0, dy: float = 0.0, dz: float = 0.0) -> "Pose3D":
        return Pose3D(self.x + dx, self.y + dy, self.z + dz,
                      self.qx, self.qy, self.qz, self.qw)


@dataclass(frozen=True)
class Detection:
    label: str
    confidence: float
    bearing_rad: float
    distance_m: float
    position_base_m: tuple[float, float, float] | None = None
    frame_id: str = ""
    timestamp_s: float = 0.0

    def to_dict(self) -> dict[str, Any]:
        value = asdict(self)
        if self.position_base_m is not None:
            value["position_base_m"] = list(self.position_base_m)
        return value


class Stage(str, Enum):
    CREATED = "CREATED"
    PREFLIGHT = "PREFLIGHT"
    NAVIGATING = "NAVIGATING"
    FRONT_SEARCH = "FRONT_SEARCH"
    BASE_ALIGN = "BASE_ALIGN"
    WRIST_LOCALIZE = "WRIST_LOCALIZE"
    PICK = "PICK"
    PLACE = "PLACE"
    COMPLETE = "COMPLETE"
    SAFE_ABORT = "SAFE_ABORT"


@dataclass
class MissionResult:
    mission_id: str
    success: bool
    final_stage: Stage
    message: str
    log_file: str
    target_base_m: tuple[float, float, float] | None = None

    def to_dict(self) -> dict[str, Any]:
        value = asdict(self)
        value["final_stage"] = self.final_stage.value
        if self.target_base_m is not None:
            value["target_base_m"] = list(self.target_base_m)
        return value

