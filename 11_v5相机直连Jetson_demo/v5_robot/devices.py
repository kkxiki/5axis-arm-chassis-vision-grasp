"""设备抽象层：把「移动底盘 / 机械臂 / 夹爪 / 前视相机 / 腕部相机」统一成 5 个接口，
并提供两套实现：仿真（Sim* 类）与真实硬件（JsonTcpBase / ArmGateway / ControllerGripper / HttpVision）。

上层逻辑只依赖 MobileBase / Arm / Gripper / Vision 这几个 Protocol，
具体装配哪套实现由 build_devices() 按 mode（"sim" 或 "real"）决定，因此业务流程代码不需要区分仿真与真机。

关键通信与单位约定：
- 移动底盘：TCP + JSON-Lines 协议（见 transport.JsonLineClient），位姿为 map 系平面位姿（x/y 单位米，yaw 单位弧度）。
- 机械臂：通过 controller 的 HTTP 事件接口下发 MoveBlend/BlendL 指令，位姿为 arm_base 系 6D 位姿（位置单位米 + 姿态四元数）。
- 夹爪：同样由 controller 控制（HTTP 事件接口），不新增独立设备。
- 相机：Jetson 直连，视觉能力由独立的 vision server 通过 HTTP 提供；腕部相机为眼在手上（eye-in-hand），
  其检测结果直接给出 arm_base 系坐标，用于精定位。
"""
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
from .motion_commands import move_blend


class DeviceError(RuntimeError):
    """设备层统一异常：设备通信失败、命令被拒绝、状态异常等都用它向外抛出。"""
    pass


class MobileBase(Protocol):
    """移动底盘能力接口（在 map 系做平面运动）。"""
    # 健康自检：返回字典至少含 ok 字段；真实底盘还会带 localized，表示定位是否有效
    def health(self) -> dict[str, Any]: ...
    # 读取当前底盘位姿：x/y 单位米，yaw 单位弧度，map 系
    def pose(self) -> Pose2D: ...
    # 导航到 map 系目标位姿；timeout_s 为该次导航的超时时间（秒）
    def navigate(self, goal: Pose2D, timeout_s: float) -> None: ...
    # 视觉对准：先相对当前朝向旋转 bearing_rad（弧度），再沿新朝向前进 forward_m（米）
    def align(self, bearing_rad: float, forward_m: float, timeout_s: float) -> None: ...
    # 立即停止底盘运动（异常/中止时兜底使用）
    def stop(self) -> None: ...


class Arm(Protocol):
    """机械臂能力接口（arm_base 系 6D 位姿，位置米 + 四元数）。"""
    # 健康自检：ok 为 True 表示控制器在线且机械臂处于可运动状态
    def health(self) -> dict[str, Any]: ...
    # 依次经过 targets 中的各点位（最后一个为终点）；speed 是控制器速度档位名，timeout_s 单位秒
    def move(self, targets: list[Pose3D], speed: str, timeout_s: float) -> None: ...
    # 立即停止机械臂运动
    def stop(self) -> None: ...


class Gripper(Protocol):
    """夹爪能力接口（由 controller 控制）。"""
    # 健康自检
    def health(self) -> dict[str, Any]: ...
    # 执行一次开/合动作：action 取 "open" 或 "close"，force 为力度百分比，speed 为速度百分比
    def command(self, action: str, force: int, speed: int) -> None: ...


class Vision(Protocol):
    """相机检测能力接口（前视相机与腕部相机共用同一套方法）。"""
    # 健康自检：返回 ok 及码流、帧率等信息
    def health(self) -> dict[str, Any]: ...
    # 在 timeout_s 秒内检测指定 label 的目标；前视相机用于底盘粗定位/对齐，腕部相机用于抓取前精定位
    def detect(self, label: str, timeout_s: float) -> Detection: ...


@dataclass
class SimWorld:
    """仿真世界状态：用纯 Python 记录「目标在哪、底盘在哪、机械臂在哪、抓住没、放下没」。

    所有 Sim* 设备共享同一个 SimWorld 实例，所以一个设备造成的状态变化对其它设备立即可见，
    这样可以不接任何硬件就跑通完整的抓取流程。
    """
    # 目标水瓶在 map 系下的平面位姿（x/y 单位米，yaw 单位弧度）
    object_map: Pose2D
    # 水瓶高度（米），仅用于仿真建模，不参与控制计算
    object_height_m: float
    # 底盘当前位姿（map 系，x/y 米、yaw 弧度）
    base_pose: Pose2D
    # 机械臂末端当前位姿（arm_base 系，位置米 + 四元数）
    arm_pose: Pose3D
    # 夹爪当前是否已夹住水瓶
    grasped: bool = False
    # 水瓶是否已被放到放置点
    placed: bool = False
    # 故障注入点名称：与某次 fail_if(name) 的 name 相同时该调用会抛异常，用于测试异常/安全中止分支；
    # 正常运行为 None，即不注入任何故障
    fail_at: str | None = None

    def fail_if(self, name: str) -> None:
        """故障注入检查：若当前注入点正好是 name，就抛 DeviceError；否则什么也不做。"""
        if self.fail_at == name:
            raise DeviceError(f"注入的仿真故障: {name}")


class SimBase:
    """仿真移动底盘：不做真实运动学，直接把位姿「设」到目标值，只校验导航距离上限并短暂 sleep 模拟耗时。"""
    def __init__(self, world: SimWorld, cfg: dict[str, Any]):
        # cfg 传入的是 navigation 段配置，这里会用到 max_goal_distance_m
        self.world, self.cfg = world, cfg

    def health(self) -> dict[str, Any]:
        """返回固定可用的健康信息：仿真底盘默认已定位（localized=True）、电量 87%。"""
        self.world.fail_if("base_health")
        return {"ok": True, "localized": True, "battery_pct": 87}

    def pose(self) -> Pose2D:
        """返回仿真世界中的底盘当前位姿（map 系）。"""
        return self.world.base_pose

    def navigate(self, goal: Pose2D, timeout_s: float) -> None:
        """模拟导航到 map 系目标点。

        与真机一致地做一次距离校验：直线距离超过 cfg["max_goal_distance_m"] 就拒绝
        （真机单次导航距离受限，因此上层需要把目标选在合理范围内）。
        """
        self.world.fail_if("navigate")
        distance = math.hypot(goal.x - self.world.base_pose.x, goal.y - self.world.base_pose.y)
        if distance > float(self.cfg["max_goal_distance_m"]):
            raise DeviceError(f"导航目标 {distance:.2f}m 超过单次上限")
        # 用与 timeout 成比例的极短时间模拟运动（上限 0.12s），避免拖慢测试
        time.sleep(min(0.12, timeout_s / 100.0))
        self.world.base_pose = goal

    def align(self, bearing_rad: float, forward_m: float, timeout_s: float) -> None:
        """模拟视觉对准：相对当前朝向先转 bearing_rad（弧度），再沿新朝向前进 forward_m（米）。"""
        self.world.fail_if("base_align")
        p = self.world.base_pose
        # 新朝向 = 原朝向 + 目标方位角
        yaw = p.yaw + bearing_rad
        # 沿新朝向直线前进 forward_m 后更新底盘平面位姿
        self.world.base_pose = Pose2D(p.x + forward_m * math.cos(yaw),
                                     p.y + forward_m * math.sin(yaw), yaw)
        # 上限 0.08s 的耗时模拟
        time.sleep(min(0.08, timeout_s / 100.0))

    def stop(self) -> None:
        """仿真底盘没有真实运动，停止是空操作（仅为满足接口而存在）。"""
        return


class SimArm:
    """仿真机械臂：忽略运动学，直接把末端位姿设为最后一个目标点，sleep 一小段时间模拟运动耗时。"""
    def __init__(self, world: SimWorld):
        self.world = world

    def health(self) -> dict[str, Any]:
        """返回固定可用的健康信息：机械臂已使能、状态 READY。"""
        self.world.fail_if("arm_health")
        return {"ok": True, "enabled": True, "state": "READY"}

    def move(self, targets: list[Pose3D], speed: str, timeout_s: float) -> None:
        """模拟运动：目标列表为空视为调用错误；否则直接跳到 targets[-1]（终点），不做插补。"""
        self.world.fail_if("arm_move")
        if not targets:
            raise DeviceError("机械臂目标为空")
        time.sleep(min(0.1, timeout_s / 100.0))
        self.world.arm_pose = targets[-1]

    def stop(self) -> None:
        """仿真机械臂无需停止动作，空操作。"""
        return


class SimGripper:
    """仿真夹爪：只维护 SimWorld 里的 grasped / placed 两个布尔状态。"""
    def __init__(self, world: SimWorld):
        self.world = world

    def health(self) -> dict[str, Any]:
        """返回固定可用的健康信息：夹爪已使能、当前处于张开位置。"""
        self.world.fail_if("gripper_health")
        return {"ok": True, "enabled": True, "position": "open"}

    def command(self, action: str, force: int, speed: int) -> None:
        """模拟夹爪开合：close 记为已夹住；open 且原本夹住时记为已松开，并按仿真约定判断是否已放置。"""
        self.world.fail_if("gripper")
        if action == "close":
            self.world.grasped = True
        elif action == "open" and self.world.grasped:
            self.world.grasped = False
            # 仿真约定：末端 y < 0 表示停在放置点上方（放置区在 arm_base 系 y 负方向），
            # 因此在这里张开夹爪即视为瓶子已放到目标位置
            if self.world.arm_pose.y < 0.0:
                self.world.placed = True


class SimFrontVision:
    """仿真前视相机：按几何关系直接算出目标相对底盘的方位角与距离，并用固定阈值模拟视场限制。"""
    def __init__(self, world: SimWorld):
        self.world = world

    def health(self) -> dict[str, Any]:
        """返回固定可用的健康信息：仿真码流 sim://front，帧率 15fps。"""
        self.world.fail_if("front_camera")
        return {"ok": True, "stream": "sim://front", "fps": 15}

    def detect(self, label: str, timeout_s: float) -> Detection:
        """模拟前视检测：由底盘位姿和目标位置算出方位角（弧度）与距离（米）。

        仿真里约定的可见范围是距离 ≤ 3.5m 且 |方位角| ≤ 60°，超出即视为看不到目标。
        返回的 Detection 只带 bearing_rad / distance_m（position_base_m 为 None），
        因为前视相机的作用是让底盘做粗定位与对齐，不提供 arm_base 系三维坐标。
        """
        self.world.fail_if("front_detect")
        b = self.world.base_pose
        # 底盘 -> 目标 的水平分量（map 系，单位米）
        dx, dy = self.world.object_map.x - b.x, self.world.object_map.y - b.y
        distance = math.hypot(dx, dy)
        # 方位角 = 目标方向 - 底盘当前朝向，再归一化到 (-π, π]，单位弧度
        bearing = _wrap(math.atan2(dy, dx) - b.yaw)
        if distance > 3.5 or abs(bearing) > math.radians(60):
            raise DeviceError("前视相机视野内未找到目标")
        # 置信度固定 0.91（仅模拟用）；frame_id 标为 front_camera，position_base_m 为 None
        return Detection(label, 0.91, bearing, distance, None, "front_camera", time.time())


class SimWristVision:
    """仿真腕部相机（眼在手上）：返回带固定毫米级残差的目标 arm_base 坐标，模拟手眼精定位。"""
    def __init__(self, world: SimWorld, target_base: tuple[float, float, float]):
        # target_base 是仿真中「底盘对齐后目标在 arm_base 系的位置」（单位米）
        self.world, self.target_base = world, target_base

    def health(self) -> dict[str, Any]:
        """返回固定可用的健康信息：仿真码流 sim://wrist，帧率 30fps，带深度。"""
        self.world.fail_if("wrist_camera")
        return {"ok": True, "stream": "sim://wrist", "fps": 30, "depth": True}

    def detect(self, label: str, timeout_s: float) -> Detection:
        """模拟腕部精定位：在真值上叠加确定性的毫米级残差（x +2mm、y -1mm），保证结果可复现。"""
        self.world.fail_if("wrist_detect")
        # 固定不变（而非随机）的毫米级残差，用来模拟手眼精定位后仍有微小误差
        p = (self.target_base[0] + 0.002, self.target_base[1] - 0.001,
             self.target_base[2])
        # 返回 arm_base 系三维坐标（frame_id="arm_base"），置信度 0.97；
        # distance_m 填 0.24 表示腕部相机到目标的典型工作距离（米）
        return Detection(label, 0.97, 0.0, 0.24, p, "arm_base", time.time())


def _wrap(angle: float) -> float:
    """把角度（弧度）归一化到 (-π, π]，用于比较方位角/朝向差时避免跨 ±π 判断错误。"""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class JsonTcpBase:
    """真实底盘适配器：按底盘约定的 JSON-Lines 协议，一行一个 JSON 请求/响应。"""

    def __init__(self, host: str, port: int, timeout_s: float):
        self.client = JsonLineClient(host, port, timeout_s)

    def health(self) -> dict[str, Any]:
        """向底盘查询健康状态（cmd=health），非 ok 响应会抛 DeviceError。"""
        return self._ok(self.client.request({"cmd": "health"}))

    def pose(self) -> Pose2D:
        """查询底盘当前位姿（cmd=pose）；响应中 pose.x/pose.y 单位米，pose.yaw 单位弧度。"""
        value = self._ok(self.client.request({"cmd": "pose"}))["pose"]
        return Pose2D(float(value["x"]), float(value["y"]), float(value["yaw"]))

    def navigate(self, goal: Pose2D, timeout_s: float) -> None:
        """下发导航指令：frame 固定为 map，goal 为 {x, y, yaw}；用调用方给出的超时时间。"""
        self._ok(self.client.request({"cmd": "navigate", "frame": "map", "goal": goal.to_dict()}, timeout_s))

    def align(self, bearing_rad: float, forward_m: float, timeout_s: float) -> None:
        """下发视觉对准指令：先转 bearing_rad（弧度），再前进 forward_m（米）。"""
        self._ok(self.client.request({"cmd": "align", "bearing_rad": bearing_rad,
                                      "forward_m": forward_m}, timeout_s))

    def stop(self) -> None:
        """下发停止指令；用 1s 短超时且吞掉异常——中止流程时不应再因通信失败抛错。"""
        try:
            self.client.request({"cmd": "stop"}, 1.0)
        except Exception:
            pass

    @staticmethod
    def _ok(reply: dict[str, Any]) -> dict[str, Any]:
        """校验底盘响应：非{"ok": true} 视为命令被拒绝，直接抛 DeviceError；否则原样返回响应。"""
        if not reply.get("ok"):
            raise DeviceError(f"底盘拒绝命令: {reply}")
        return reply


class ArmGateway:
    """真实机械臂适配器：兼容 v3 ArmGateway 的 send/state JSON 接口（同样是 JSON-Lines）。"""

    def __init__(self, host: str, port: int, owner: str = "AUTO_TASK"):
        # owner 是控制权标识：下达运动指令前必须先抢到该 owner，避免与人工示教/其它程序同时控制机械臂
        self.client = JsonLineClient(host, port, 5.0)
        self.owner = owner

    def health(self) -> dict[str, Any]:
        """查询控制器状态（cmd=state），要求已连接、机械臂已使能且状态为 READY 才算 ok。"""
        state = self.client.request({"cmd": "state"})
        ok = bool(state.get("connected") and state.get("arm_enabled") and state.get("state") == "READY")
        return {"ok": ok, **state}

    def move(self, targets: list[Pose3D], speed: str, timeout_s: float) -> None:
        """执行一次运动：内容是一个只含 BlendL 段的 MoveBlend 容器（由 move_blend 生成）。"""
        # 第一步：获取控制权，失败则直接放弃下发
        owner = self.client.request({"cmd": "set_owner", "owner": self.owner})
        if not owner.get("ok"):
            raise DeviceError(f"机械臂控制权获取失败: {owner}")
        # 第二步：把位姿序列编成 {MoveBlend first_insert / BlendL... / MoveBlend start} 指令串
        commands = move_blend((target.to_list() for target in targets), speed)
        for index, raw in enumerate(commands):
            # 只有最后一条（start，真正触发整段运动）用调用方给的超时；
            # 前面的插入类指令是瞬时返回的，用 5s 足够
            reply = self.client.request({"cmd": "send", "owner": self.owner, "raw": raw},
                                        timeout_s if index == len(commands) - 1 else 5.0)
            if not reply.get("ok"):
                raise DeviceError(f"机械臂拒绝命令: {reply}")

    def stop(self) -> None:
        """尽力停止并释放控制权：两步都吞掉异常，保证中止流程不会因通信问题二次失败。"""
        try:
            self.client.request({"cmd": "stop"}, 2.0)
        except Exception:
            pass
        try:
            self.client.request({"cmd": "release_owner", "owner": self.owner}, 1.0)
        except Exception:
            pass


class ControllerGripper:
    """真实夹爪适配器：复用 controller 已有的 HTTP 事件接口，不新增独立设备。"""

    def __init__(self, host: str, cfg: dict[str, Any]):
        self.client = ControllerEventClient(host)
        # cfg 是 gripper 段配置，用到 close_distance / open_distance / interval
        self.cfg = cfg

    def health(self) -> dict[str, Any]:
        """探测控制器 HTTP 服务是否可达（返回 ok 与 HTTP 状态码/错误信息）。"""
        return self.client.health()

    def command(self, action: str, force: int, speed: int) -> None:
        """通过控制器事件 MoveTwoFingersGripper 开合两指夹爪。

        控制器的该事件只接受 distance（目标开合距离）与 interval（动作间隔），
        没有力度/速度概念，所以 force 与 speed 在这里被显式忽略。
        """
        del force, speed
        if action not in ("open", "close"):
            raise DeviceError(f"未知夹爪动作: {action}")
        # 合上时用 close_distance，张开时用 open_distance；均为配置里的整数距离值
        key = "close_distance" if action == "close" else "open_distance"
        try:
            self.client.send("MoveTwoFingersGripper",
                             distance=int(self.cfg[key]),
                             interval=int(self.cfg["interval"]))
        except Exception as exc:
            raise DeviceError(f"控制器夹爪命令失败: {exc}") from exc


class HttpVision:
    """视觉服务 HTTP 适配器：前视相机与腕部相机共用，靠 camera_id 区分具体是哪一路。"""

    def __init__(self, base_url: str, camera_id: str, token: str = ""):
        # base_url 为 Jetson 上 vision server 的地址（去掉结尾斜杠，便于后面直接拼路径）；
        # camera_id（"front" 或 "wrist"）用于告诉服务端本次要操作哪一路相机
        self.base_url = base_url.rstrip("/")
        self.camera_id = camera_id
        self.token = token

    def _get(self, path: str, timeout_s: float) -> dict[str, Any]:
        """发一个 GET 请求并解析 JSON；带 token 时加 Bearer 认证头，任何失败都转成 DeviceError。"""
        headers = {"Authorization": f"Bearer {self.token}"} if self.token else {}
        request = Request(self.base_url + path, headers=headers)
        try:
            with urlopen(request, timeout=timeout_s) as response:
                return json.loads(response.read().decode("utf-8"))
        except Exception as exc:
            raise DeviceError(f"视觉服务 {self.camera_id} 请求失败: {exc}") from exc

    def health(self) -> dict[str, Any]:
        """查询该路相机的健康状态（固定 2s 超时），并保证返回字典里含有 ok 字段。"""
        value = self._get("/health?" + urlencode({"camera_id": self.camera_id}), 2.0)
        return {"ok": bool(value.get("ok")), **value}

    def detect(self, label: str, timeout_s: float) -> Detection:
        """请求服务检测目标：服务端等待上限取 min(timeout_s, 10s)，本地 HTTP 超时再放宽 1s。

        服务返回的字段：detected（是否检出）、confidence、position_base_m（可选，arm_base 系坐标，单位米）、
        bearing_rad、distance_m、frame_id、timestamp_s。未检出时抛 DeviceError。
        """
        query = urlencode({"camera_id": self.camera_id, "label": label,
                           "timeout_s": min(float(timeout_s), 10.0)})
        value = self._get("/detect?" + query, timeout_s + 1.0)
        if not value.get("detected"):
            raise DeviceError(f"{self.camera_id} 未检测到 {label}")
        pos = value.get("position_base_m")
        # 逐字段做类型转换并给出保守默认值：位置缺失则为 None，时间戳缺失则取本地当前时间
        return Detection(label, float(value["confidence"]), float(value.get("bearing_rad", 0.0)),
                         float(value.get("distance_m", 0.0)), tuple(map(float, pos)) if pos else None,
                         str(value.get("frame_id", self.camera_id)), float(value.get("timestamp_s", time.time())))


@dataclass
class DeviceSet:
    """一次任务用到的设备集合；world 只在仿真模式下非空（用于断言仿真状态）。"""
    base: MobileBase
    arm: Arm
    gripper: Gripper
    front: Vision
    wrist: Vision
    world: SimWorld | None = None


def build_devices(cfg: dict[str, Any], mode: str, fail_at: str | None = None) -> DeviceSet:
    """按模式装配一整套设备。

    mode="sim"  ：用配置里的 simulation 段构造 SimWorld，再挂上各 Sim* 设备；
                  其中腕部相机的目标位置取 sim["object_base_after_align_m"]，即「底盘对齐后目标在 arm_base 系的位置」假设值。
    mode="real" ：用配置里的 network 段连真机——底盘走 TCP、机械臂网关走 TCP、夹爪走控制器的 HTTP 事件接口，
                  两路相机都是同一个 Jetson vision server 的 HTTP 服务（camera_id 分别为 front / wrist）。
    fail_at 仅在仿真下有效，用于在指定调用点注入故障，测试安全中止流程。
    """
    if mode == "sim":
        sim = cfg["simulation"]
        # 由配置构造仿真世界：目标水瓶位姿、底盘初始位姿、机械臂初始位姿（取示教 home 点）
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
    # 前视与腕部两路相机共用同一个 vision server 地址，靠 camera_id 区分
    vision_url = net["jetson_vision_service_url"]
    return DeviceSet(base, arm, gripper,
                     HttpVision(vision_url, "front"), HttpVision(vision_url, "wrist"))
