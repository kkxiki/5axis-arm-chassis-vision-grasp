from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Protocol
from urllib.request import Request, urlopen
from urllib.parse import urlencode

from .models import Detection, Pose2D, Pose3D
from .transport import JsonLineClient, TransportError
from .controller_api import ControllerEventClient


class DeviceError(RuntimeError):
    pass


class MobileBase(Protocol):
    def health(self) -> dict[str, Any]: ...
    def pose(self) -> Pose2D: ...
    def navigate(self, goal: Pose2D, timeout_s: float) -> None: ...
    def align(self, bearing_rad: float, forward_m: float, timeout_s: float) -> None: ...
    def stop(self) -> None: ...


class Arm(Protocol):
    def health(self) -> dict[str, Any]: ...
    def move(self, targets: list[Pose3D], speed: str, timeout_s: float) -> None: ...
    def stop(self) -> None: ...


class Gripper(Protocol):
    def health(self) -> dict[str, Any]: ...
    def command(self, action: str, force: int, speed: int) -> None: ...


class Vision(Protocol):
    def health(self) -> dict[str, Any]: ...
    def detect(self, label: str, timeout_s: float) -> Detection: ...


@dataclass
class SimWorld:
    object_map: Pose2D
    object_height_m: float
    base_pose: Pose2D
    arm_pose: Pose3D
    grasped: bool = False
    placed: bool = False
    fail_at: str | None = None

    def fail_if(self, name: str) -> None:
        if self.fail_at == name:
            raise DeviceError(f"注入的仿真故障: {name}")


class SimBase:
    def __init__(self, world: SimWorld, cfg: dict[str, Any]):
        self.world, self.cfg = world, cfg

    def health(self) -> dict[str, Any]:
        self.world.fail_if("base_health")
        return {"ok": True, "localized": True, "battery_pct": 87}

    def pose(self) -> Pose2D:
        return self.world.base_pose

    def navigate(self, goal: Pose2D, timeout_s: float) -> None:
        self.world.fail_if("navigate")
        distance = math.hypot(goal.x - self.world.base_pose.x, goal.y - self.world.base_pose.y)
        if distance > float(self.cfg["max_goal_distance_m"]):
            raise DeviceError(f"导航目标 {distance:.2f}m 超过单次上限")
        time.sleep(min(0.12, timeout_s / 100.0))
        self.world.base_pose = goal

    def align(self, bearing_rad: float, forward_m: float, timeout_s: float) -> None:
        self.world.fail_if("base_align")
        p = self.world.base_pose
        yaw = p.yaw + bearing_rad
        self.world.base_pose = Pose2D(p.x + forward_m * math.cos(yaw),
                                     p.y + forward_m * math.sin(yaw), yaw)
        time.sleep(min(0.08, timeout_s / 100.0))

    def stop(self) -> None:
        return


class SimArm:
    def __init__(self, world: SimWorld):
        self.world = world

    def health(self) -> dict[str, Any]:
        self.world.fail_if("arm_health")
        return {"ok": True, "enabled": True, "state": "READY"}

    def move(self, targets: list[Pose3D], speed: str, timeout_s: float) -> None:
        self.world.fail_if("arm_move")
        if not targets:
            raise DeviceError("机械臂目标为空")
        time.sleep(min(0.1, timeout_s / 100.0))
        self.world.arm_pose = targets[-1]

    def stop(self) -> None:
        return


class SimGripper:
    def __init__(self, world: SimWorld):
        self.world = world

    def health(self) -> dict[str, Any]:
        self.world.fail_if("gripper_health")
        return {"ok": True, "enabled": True, "position": "open"}

    def command(self, action: str, force: int, speed: int) -> None:
        self.world.fail_if("gripper")
        if action == "close":
            self.world.grasped = True
        elif action == "open" and self.world.grasped:
            self.world.grasped = False
            if self.world.arm_pose.y < 0.0:
                self.world.placed = True


class SimFrontVision:
    def __init__(self, world: SimWorld):
        self.world = world

    def health(self) -> dict[str, Any]:
        self.world.fail_if("front_camera")
        return {"ok": True, "stream": "sim://front", "fps": 15}

    def detect(self, label: str, timeout_s: float) -> Detection:
        self.world.fail_if("front_detect")
        b = self.world.base_pose
        dx, dy = self.world.object_map.x - b.x, self.world.object_map.y - b.y
        distance = math.hypot(dx, dy)
        bearing = _wrap(math.atan2(dy, dx) - b.yaw)
        if distance > 3.5 or abs(bearing) > math.radians(60):
            raise DeviceError("前视相机视野内未找到目标")
        return Detection(label, 0.91, bearing, distance, None, "front_camera", time.time())


class SimWristVision:
    def __init__(self, world: SimWorld, target_base: tuple[float, float, float]):
        self.world, self.target_base = world, target_base

    def health(self) -> dict[str, Any]:
        self.world.fail_if("wrist_camera")
        return {"ok": True, "stream": "sim://wrist", "fps": 30, "depth": True}

    def detect(self, label: str, timeout_s: float) -> Detection:
        self.world.fail_if("wrist_detect")
        # Deterministic millimetre-level residual emulates hand-eye refinement.
        p = (self.target_base[0] + 0.002, self.target_base[1] - 0.001,
             self.target_base[2])
        return Detection(label, 0.97, 0.0, 0.24, p, "arm_base", time.time())


def _wrap(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class JsonTcpBase:
    """Adapter for the proposed chassis JSON-lines protocol."""

    def __init__(self, host: str, port: int, timeout_s: float):
        self.client = JsonLineClient(host, port, timeout_s)

    def health(self) -> dict[str, Any]:
        return self._ok(self.client.request({"cmd": "health"}))

    def pose(self) -> Pose2D:
        value = self._ok(self.client.request({"cmd": "pose"}))["pose"]
        return Pose2D(float(value["x"]), float(value["y"]), float(value["yaw"]))

    def navigate(self, goal: Pose2D, timeout_s: float) -> None:
        self._ok(self.client.request({"cmd": "navigate", "frame": "map", "goal": goal.to_dict()}, timeout_s))

    def align(self, bearing_rad: float, forward_m: float, timeout_s: float) -> None:
        self._ok(self.client.request({"cmd": "align", "bearing_rad": bearing_rad,
                                      "forward_m": forward_m}, timeout_s))

    def stop(self) -> None:
        try:
            self.client.request({"cmd": "stop"}, 1.0)
        except Exception:
            pass

    @staticmethod
    def _ok(reply: dict[str, Any]) -> dict[str, Any]:
        if not reply.get("ok"):
            raise DeviceError(f"底盘拒绝命令: {reply}")
        return reply


class ArmGateway:
    """Compatible with the v3 ArmGateway send/state JSON API."""

    def __init__(self, host: str, port: int, owner: str = "AUTO_TASK"):
        self.client = JsonLineClient(host, port, 5.0)
        self.owner = owner

    def health(self) -> dict[str, Any]:
        state = self.client.request({"cmd": "state"})
        ok = bool(state.get("connected") and state.get("arm_enabled") and state.get("state") == "READY")
        return {"ok": ok, **state}

    def move(self, targets: list[Pose3D], speed: str, timeout_s: float) -> None:
        owner = self.client.request({"cmd": "set_owner", "owner": self.owner})
        if not owner.get("ok"):
            raise DeviceError(f"机械臂控制权获取失败: {owner}")
        commands = ["{MoveBlend --type=first_insert}"]
        for target in targets:
            values = ",".join(f"{v:.9f}" for v in target.to_list())
            commands.append("{MoveBlend --type=insert_line --robottarget_value={" + values +
                            f"}} --zone={{0.0,0.0}} --speed={speed}}}")
        commands.append("{MoveBlend --type=start}")
        for index, raw in enumerate(commands):
            reply = self.client.request({"cmd": "send", "owner": self.owner, "raw": raw},
                                        timeout_s if index == len(commands) - 1 else 5.0)
            if not reply.get("ok"):
                raise DeviceError(f"机械臂拒绝命令: {reply}")

    def stop(self) -> None:
        try:
            self.client.request({"cmd": "stop"}, 2.0)
        except Exception:
            pass
        try:
            self.client.request({"cmd": "release_owner", "owner": self.owner}, 1.0)
        except Exception:
            pass


class ControllerGripper:
    """Use the controller's existing HTTP event API; never add a device."""

    def __init__(self, host: str, cfg: dict[str, Any]):
        self.client = ControllerEventClient(host)
        self.cfg = cfg

    def health(self) -> dict[str, Any]:
        return self.client.health()

    def command(self, action: str, force: int, speed: int) -> None:
        del force, speed
        if action not in ("open", "close"):
            raise DeviceError(f"未知夹爪动作: {action}")
        key = "close_distance" if action == "close" else "open_distance"
        try:
            self.client.send("MoveTwoFingersGripper",
                             distance=int(self.cfg[key]),
                             interval=int(self.cfg["interval"]))
        except Exception as exc:
            raise DeviceError(f"控制器夹爪命令失败: {exc}") from exc


class HttpVision:
    """HTTP detector service adapter for either network camera pipeline."""

    def __init__(self, base_url: str, camera_id: str, token: str = ""):
        self.base_url = base_url.rstrip("/")
        self.camera_id = camera_id
        self.token = token

    def _get(self, path: str, timeout_s: float) -> dict[str, Any]:
        headers = {"Authorization": f"Bearer {self.token}"} if self.token else {}
        request = Request(self.base_url + path, headers=headers)
        try:
            with urlopen(request, timeout=timeout_s) as response:
                return json.loads(response.read().decode("utf-8"))
        except Exception as exc:
            raise DeviceError(f"视觉服务 {self.camera_id} 请求失败: {exc}") from exc

    def health(self) -> dict[str, Any]:
        value = self._get("/health?" + urlencode({"camera_id": self.camera_id}), 2.0)
        return {"ok": bool(value.get("ok")), **value}

    def detect(self, label: str, timeout_s: float) -> Detection:
        query = urlencode({"camera_id": self.camera_id, "label": label,
                           "timeout_s": min(float(timeout_s), 10.0)})
        value = self._get("/detect?" + query, timeout_s + 1.0)
        if not value.get("detected"):
            raise DeviceError(f"{self.camera_id} 未检测到 {label}")
        pos = value.get("position_base_m")
        return Detection(label, float(value["confidence"]), float(value.get("bearing_rad", 0.0)),
                         float(value.get("distance_m", 0.0)), tuple(map(float, pos)) if pos else None,
                         str(value.get("frame_id", self.camera_id)), float(value.get("timestamp_s", time.time())))


@dataclass
class DeviceSet:
    base: MobileBase
    arm: Arm
    gripper: Gripper
    front: Vision
    wrist: Vision
    world: SimWorld | None = None


def build_devices(cfg: dict[str, Any], mode: str, fail_at: str | None = None) -> DeviceSet:
    if mode == "sim":
        sim = cfg["simulation"]
        obj = Pose2D(**sim["object_map_pose"])
        base_pose = Pose2D(**sim["initial_base_pose"])
        home = Pose3D.from_list(cfg["arm"]["poses"]["home"])
        world = SimWorld(obj, float(sim["object_height_m"]), base_pose, home, fail_at=fail_at)
        target = tuple(float(v) for v in sim["object_base_after_align_m"])
        return DeviceSet(SimBase(world, cfg["navigation"]), SimArm(world), SimGripper(world),
                         SimFrontVision(world), SimWristVision(world, target), world)

    net = cfg["network"]
    base = JsonTcpBase(net["base_ip"], net["base_command_port"], 3.0)
    arm = ArmGateway(net["arm_gateway_host"], net["arm_gateway_port"])
    gripper = ControllerGripper(net["controller_ip"], cfg["gripper"])
    vision_url = net["jetson_vision_service_url"]
    return DeviceSet(base, arm, gripper,
                     HttpVision(vision_url, "front"), HttpVision(vision_url, "wrist"))
