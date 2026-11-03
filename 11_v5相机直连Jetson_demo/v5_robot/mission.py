"""五轴移动机械臂「抓水瓶」任务的编排器（Stage 状态机）。

职责：把一次完整抓取拆成若干阶段，按顺序驱动底盘、机械臂、夹爪和前后两台相机，
并把每个阶段的进展以 JSONL 事件（一行一个 JSON 对象）追加写入 runtime 目录下的
<mission_id>.jsonl 日志，便于事后回放与排错。

阶段推进顺序：
    CREATED → PREFLIGHT → NAVIGATING → FRONT_SEARCH → BASE_ALIGN
    → WRIST_LOCALIZE → PICK → PLACE → COMPLETE

任何一步抛出异常都会切到 SAFE_ABORT：停掉底盘与机械臂输出后返回失败结果。
注意此处刻意不主动张开夹爪——若异常发生在已经夹住水瓶之后，把瓶子扔掉比保持夹紧更糟。

坐标系与单位约定：
    底盘 —— Pose2D，map 系，位置 m、偏航角 rad
    机械臂 —— Pose3D，arm_base（机械臂基座）系，位置 m + 四元数姿态
    时间 —— 统一为秒（s）
"""
from __future__ import annotations

import json
import math
import time
import uuid
from pathlib import Path
from typing import Any

from .devices import DeviceError, DeviceSet
from .models import MissionResult, Pose2D, Pose3D, Stage


class MissionError(RuntimeError):
    """任务编排中可预期的失败（预检不过、视觉结果不可信、坐标越界等）。"""

    pass


class MissionRunner:
    """一次抓取任务的执行者：持有配置与设备集合，按状态机推进整个流程。"""

    def __init__(self, cfg: dict[str, Any], devices: DeviceSet, runtime_dir: str | Path):
        """准备一次任务运行：生成任务 ID，并确定 JSONL 日志的输出位置。

        参数：
            cfg         —— 已校验的配置字典（导航超时、视觉阈值、机械臂速度、安全范围等）
            devices     —— 设备集合；sim 模式为仿真实现，real 模式为真实硬件适配器
            runtime_dir —— 事件日志目录，不存在时自动创建

        mission_id 形如 v5-20260915-143000-ab12cd（本地时间 + 6 位随机十六进制串），
        日志文件为 runtime_dir/<mission_id>.jsonl。
        """
        self.cfg = cfg
        self.devices = devices
        self.mission_id = time.strftime("v5-%Y%m%d-%H%M%S-") + uuid.uuid4().hex[:6]
        runtime = Path(runtime_dir).resolve()
        runtime.mkdir(parents=True, exist_ok=True)
        self.log_path = runtime / f"{self.mission_id}.jsonl"
        self.stage = Stage.CREATED
        # 腕部视觉给出的目标点（arm_base 系，单位 m），抓取前会重新赋值
        self.target_base: tuple[float, float, float] | None = None

    def emit(self, event: str, **data: Any) -> None:
        """把一条任务事件同时写入 JSONL 日志并打印到控制台。

        每行固定包含 ts（Unix 时间戳，秒）、mission_id、stage（当前阶段名）和
        event（事件名），其余键值由调用方通过 **data 追加。
        写文件时用紧凑分隔符且不转义中文，保证「一行一条记录」，方便流式追加与逐行解析。
        """
        # 事件行的固定字段：时间戳 + 任务 ID + 当前阶段名 + 事件名，再合并附加数据
        row = {"ts": time.time(), "mission_id": self.mission_id,
               "stage": self.stage.value, "event": event, **data}
        # 以追加模式写入，天真的 json.dumps 默认会带空格且转义中文，这里都关掉
        with self.log_path.open("a", encoding="utf-8") as file:
            file.write(json.dumps(row, ensure_ascii=False, separators=(",", ":")) + "\n")
        # 同时在终端打印一行摘要，方便现场实时观察任务进度
        details = " " + json.dumps(data, ensure_ascii=False) if data else ""
        print(f"[{self.stage.value}] {event}{details}", flush=True)

    def set_stage(self, stage: Stage) -> None:
        """切换状态机的当前阶段，并立刻补发一条 stage_enter 事件。

        所有阶段跳转都经由此方法，保证日志里能完整还原状态机轨迹。
        """
        self.stage = stage
        self.emit("stage_enter")

    def run(self, map_goal: Pose2D, label: str) -> MissionResult:
        """执行一次完整的「导航 → 对准 → 精定位 → 抓取 → 放置」流程。

        参数：
            map_goal —— 底盘导航目标位姿（map 系，x/y 单位 m，yaw 单位 rad），
                        一般是配置里预先示教好的物体附近坐标
            label    —— 目标类别标签（如 "bottle"），透传给前后两台相机的检测服务

        返回：
            MissionResult —— 含任务 ID、是否成功、最终阶段、消息说明与日志路径。
            本方法不向上抛异常：中途任何异常都会转成 SAFE_ABORT 后的失败结果返回。
        """
        try:
            # 阶段 PREFLIGHT：设备自检，确认底盘、机械臂、夹爪和两台相机都在线可用
            self._preflight()

            # 阶段 NAVIGATING：底盘按 map 系目标点自主导航到水瓶附近
            self.set_stage(Stage.NAVIGATING)
            self.emit("navigation_goal", goal=map_goal.to_dict())
            self.devices.base.navigate(map_goal, float(self.cfg["navigation"]["timeout_s"]))
            self.emit("navigation_reached", pose=self.devices.base.pose().to_dict())

            # 阶段 FRONT_SEARCH：前视相机粗定位，只取方位角和距离，不要求三维坐标
            self.set_stage(Stage.FRONT_SEARCH)
            coarse = self.devices.front.detect(label, float(self.cfg["vision"]["front_timeout_s"]))
            self._validate_detection(coarse, "front")
            self.emit("front_detection", detection=coarse.to_dict())

            # 阶段 BASE_ALIGN：底盘转向目标并前进到合适的作业站位
            self.set_stage(Stage.BASE_ALIGN)
            # 期望的前视站位距离(m)：比它更远才需要前进，以免撞到水瓶
            desired = float(self.cfg["navigation"]["front_standoff_m"])
            forward = max(0.0, coarse.distance_m - desired)
            # 单次视觉引导的前进量上限(m)，防止检测异常时底盘冲得太远
            forward = min(forward, float(self.cfg["navigation"]["max_visual_approach_m"]))
            # bearing_rad 为相对车身朝向的目标方位角(rad)，forward 为前进距离(m)
            self.devices.base.align(coarse.bearing_rad, forward,
                                    float(self.cfg["navigation"]["align_timeout_s"]))
            self.emit("base_aligned", commanded_bearing_rad=coarse.bearing_rad,
                      commanded_forward_m=forward, pose=self.devices.base.pose().to_dict())

            # 对准后必须复核一次：只有方位角与距离都落入容差，才允许伸出机械臂
            confirmed = self.devices.front.detect(label, float(self.cfg["vision"]["front_timeout_s"]))
            self._validate_detection(confirmed, "front")
            max_bearing = float(self.cfg["navigation"]["alignment_tolerance_rad"])
            max_distance = float(self.cfg["navigation"]["arm_activation_distance_m"])
            if abs(confirmed.bearing_rad) > max_bearing or confirmed.distance_m > max_distance:
                raise MissionError("底盘视觉对准复核失败，不允许伸出机械臂: "
                                   f"bearing={confirmed.bearing_rad:.3f}, distance={confirmed.distance_m:.3f}")
            self.emit("alignment_confirmed", detection=confirmed.to_dict())

            # 阶段 WRIST_LOCALIZE（前置动作）：先把机械臂摆到观察位，让腕部相机俯视目标
            observe = self._pose("observe")
            self.devices.arm.move([observe], self.cfg["arm"]["speed"],
                                  float(self.cfg["arm"]["motion_timeout_s"]))
            self.set_stage(Stage.WRIST_LOCALIZE)
            # 腕部相机（eye-in-hand，装在机械臂末端）做精定位，输出 arm_base 系三维坐标
            fine = self.devices.wrist.detect(label, float(self.cfg["vision"]["wrist_timeout_s"]))
            self._validate_detection(fine, "wrist")
            # 没有三维坐标就无法换算抓取点位，直接判失败
            if fine.position_base_m is None:
                raise MissionError("腕部视觉没有返回 arm_base 坐标")
            # 坐标系必须是机械臂基座系，否则点位含义完全不同
            if fine.frame_id not in ("arm_base", "base_link"):
                raise MissionError(f"腕部坐标系必须是 arm_base/base_link，实际为 {fine.frame_id}")
            self.target_base = fine.position_base_m
            self._validate_target(self.target_base)
            self.emit("wrist_detection", detection=fine.to_dict())

            # 阶段 PICK / PLACE：抓取与放置
            self._pick()
            self._place()
            self.set_stage(Stage.COMPLETE)
            # 仿真模式下以 world.placed 作为客观落位依据；real 模式没有 world，按成功处理
            self.emit("mission_complete", placed=bool(self.devices.world and self.devices.world.placed)
                      if self.devices.world else True)
            return MissionResult(self.mission_id, True, self.stage, "抓取放置完成",
                                 str(self.log_path), self.target_base)
        except Exception as exc:
            # 任意异常统一走安全中止：停输出、记错误，最后以失败结果返回而不向外抛
            self._safe_abort(exc)
            return MissionResult(self.mission_id, False, self.stage, str(exc),
                                 str(self.log_path), self.target_base)

    def _preflight(self) -> None:
        """阶段 PREFLIGHT：开机自检，确认所有设备可用且底盘已完成定位。

        依次询问底盘、前视相机、机械臂控制器、夹爪控制器、腕部相机的健康状态，
        结果以 health 事件写入日志。只要有一台设备返回 ok 为假，或底盘
        localized 明确为 False，就抛 MissionError 阻止后续运动。
        """
        self.set_stage(Stage.PREFLIGHT)
        checks = {
            "base": self.devices.base.health(),
            "front_camera": self.devices.front.health(),
            "controller_arm": self.devices.arm.health(),
            "controller_gripper": self.devices.gripper.health(),
            "wrist_camera": self.devices.wrist.health(),
        }
        self.emit("health", devices=checks)
        # 汇总所有健康检查未通过的设备名，便于一次性报出全部问题
        failed = [name for name, value in checks.items() if not value.get("ok")]
        # 底盘没定位就没有可靠的地图坐标，后续导航与对准的前提不成立
        if checks["base"].get("localized") is False:
            failed.append("base_not_localized")
        if failed:
            raise MissionError("预检失败: " + ", ".join(failed))

    def _pick(self) -> None:
        """阶段 PICK：接近 → 抓取 → 抬升三个动作。

        以腕部视觉给出的目标坐标 (x, y, z)（arm_base 系，单位 m）作为抓取点，
        姿态沿用观察位姿态（只平移不改姿态，避免腕部相机绕轴翻转），Z 方向再叠加
        配置里的 grasp_z_offset_m 补偿。接近位、抬升位均由抓取点垂直偏移得到。
        三个点位下发前都会做安全工作空间校验，防止机械臂撞到工作区外。
        """
        assert self.target_base is not None
        self.set_stage(Stage.PICK)
        x, y, z = self.target_base
        orientation = self._pose("observe")
        # 抓取点：X/Y 直接采用视觉结果，Z 加配置补偿量（单位 m）
        grasp = Pose3D(x, y, z + float(self.cfg["arm"]["grasp_z_offset_m"]),
                       orientation.qx, orientation.qy, orientation.qz, orientation.qw)
        # 接近位：抓取点正上方 approach_height_m 处，垂直切入避免斜插碰撞
        above = grasp.shifted(dz=float(self.cfg["arm"]["approach_height_m"]))
        # 抬升位：夹住后垂直抬高 lift_height_m，使水瓶脱离桌面
        lift = grasp.shifted(dz=float(self.cfg["arm"]["lift_height_m"]))
        for name, pose in (("above", above), ("grasp", grasp), ("lift", lift)):
            self._validate_pose(pose, name)
        # 先张开夹爪，避免带着闭合的手爪撞到瓶身
        self.devices.gripper.command("open", self.cfg["gripper"]["force_pct"],
                                     self.cfg["gripper"]["speed_pct"])
        # 以较慢速度一次走完 接近位 → 抓取位，保证末端定位稳定
        self.devices.arm.move([above, grasp], self.cfg["arm"]["slow_speed"],
                              float(self.cfg["arm"]["motion_timeout_s"]))
        # 闭合夹爪夹住水瓶
        self.devices.gripper.command("close", self.cfg["gripper"]["force_pct"],
                                     self.cfg["gripper"]["speed_pct"])
        # 等夹爪稳定夹紧后再抬臂，防止半夹状态下瓶子滑动
        time.sleep(float(self.cfg["gripper"]["settle_s"]))
        self.devices.arm.move([lift], self.cfg["arm"]["slow_speed"],
                              float(self.cfg["arm"]["motion_timeout_s"]))
        self.emit("pick_complete", above=above.to_list(), grasp=grasp.to_list(), lift=lift.to_list())

    def _place(self) -> None:
        """阶段 PLACE：把水瓶放到放置点，然后回到零位。

        放置上方点、放置位、零位都是配置中示教好的固定点位（arm_base 系）。
        先慢速走到放置位并张开夹爪放下水瓶，再经过放置上方点回到 home，
        这样回程不会贴着放置台面刮蹭。
        """
        self.set_stage(Stage.PLACE)
        above = self._pose("place_above")
        place = self._pose("place")
        home = self._pose("home")
        for name, pose in (("place_above", above), ("place", place), ("home", home)):
            self._validate_pose(pose, name)
        # 慢速从放置上方点走到放置位（避开障碍、精确落位）
        self.devices.arm.move([above, place], self.cfg["arm"]["slow_speed"],
                              float(self.cfg["arm"]["motion_timeout_s"]))
        # 张开夹爪放下水瓶
        self.devices.gripper.command("open", self.cfg["gripper"]["force_pct"],
                                     self.cfg["gripper"]["speed_pct"])
        # 先退回放置上方点再回零位；此段无夹持物，可用常规速度
        self.devices.arm.move([above, home], self.cfg["arm"]["speed"],
                              float(self.cfg["arm"]["motion_timeout_s"]))
        self.emit("place_complete", place=place.to_list(), home=home.to_list())

    def _pose(self, name: str) -> Pose3D:
        """从配置的 arm.poses 中读取示教点位，转成 arm_base 系下的 Pose3D（位置 m + 四元数）。"""
        return Pose3D.from_list(self.cfg["arm"]["poses"][name])

    def _validate_target(self, target: tuple[float, float, float]) -> None:
        """校验腕部视觉给出的目标点：先确认三个分量都是有限数，再复用位姿安全范围检查。"""
        if any(not math.isfinite(v) for v in target):
            raise MissionError("腕部定位坐标含 NaN/Inf")
        self._validate_pose(Pose3D(*target), "vision_target")

    def _validate_detection(self, detection, camera: str) -> None:
        """校验一次视觉检测结果是否可信：置信度达标，且结果足够新鲜。

        参数：
            detection —— 检测结果对象（需含 confidence、timestamp_s 等字段）
            camera    —— 相机标识（"front" / "wrist"），用于读取对应配置与拼错误消息

        timestamp_s <= 0 表示服务没给有效时间戳；age 为负说明相机与本机时钟不同步。
        这两种情况都视为不可信，直接抛 MissionError。
        """
        minimum = float(self.cfg["vision"][f"{camera}_min_confidence"])
        if detection.confidence < minimum:
            raise MissionError(f"{camera} 识别置信度 {detection.confidence:.2f} 低于 {minimum:.2f}")
        max_age = float(self.cfg["vision"]["max_detection_age_s"])
        age = time.time() - detection.timestamp_s
        # 允许 0.2s 以内的负偏差，容忍相机与本机时钟的轻微不同步
        if detection.timestamp_s <= 0 or age < -0.2 or age > max_age:
            raise MissionError(f"{camera} 识别结果已过期或时钟不同步: age={age:.3f}s")

    def _validate_pose(self, pose: Pose3D, name: str) -> None:
        """检查点位的 x/y/z（单位 m）是否落在配置的安全工作空间内，超出则拒绝下发。"""
        limits = self.cfg["safety"]["arm_workspace_m"]
        for axis, value in (("x", pose.x), ("y", pose.y), ("z", pose.z)):
            # 每个轴的上下限都以 [low, high] 形式写在配置里
            low, high = map(float, limits[axis])
            if not low <= value <= high:
                raise MissionError(f"{name}.{axis}={value:.3f}m 超出安全范围 [{low},{high}]")

    def _safe_abort(self, exc: Exception) -> None:
        """异常兜底：进入 SAFE_ABORT，停掉底盘与机械臂输出，并保持夹爪当前状态。

        刻意不主动张开夹爪：若异常发生在已经夹住水瓶之后，放手让瓶子掉落比保持夹紧更糟。
        因此这里只停止运动输出，夹爪维持最后状态，并把失败原因写入日志。
        """
        self.stage = Stage.SAFE_ABORT
        self.emit("mission_error", error=type(exc).__name__, message=str(exc))
        self.devices.base.stop()
        self.devices.arm.stop()
        # 抓到之后不要自动张开夹爪：掉瓶的后果比维持夹紧更严重。
        self.emit("outputs_stopped", gripper_policy="hold_last_state")
