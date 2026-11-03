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
    pass


class MissionRunner:
    def __init__(self, cfg: dict[str, Any], devices: DeviceSet, runtime_dir: str | Path):
        self.cfg = cfg
        self.devices = devices
        self.mission_id = time.strftime("v4-%Y%m%d-%H%M%S-") + uuid.uuid4().hex[:6]
        runtime = Path(runtime_dir).resolve()
        runtime.mkdir(parents=True, exist_ok=True)
        self.log_path = runtime / f"{self.mission_id}.jsonl"
        self.stage = Stage.CREATED
        self.target_base: tuple[float, float, float] | None = None

    def emit(self, event: str, **data: Any) -> None:
        row = {"ts": time.time(), "mission_id": self.mission_id,
               "stage": self.stage.value, "event": event, **data}
        with self.log_path.open("a", encoding="utf-8") as file:
            file.write(json.dumps(row, ensure_ascii=False, separators=(",", ":")) + "\n")
        details = " " + json.dumps(data, ensure_ascii=False) if data else ""
        print(f"[{self.stage.value}] {event}{details}", flush=True)

    def set_stage(self, stage: Stage) -> None:
        self.stage = stage
        self.emit("stage_enter")

    def run(self, map_goal: Pose2D, label: str) -> MissionResult:
        try:
            self._preflight()
            self.set_stage(Stage.NAVIGATING)
            self.emit("navigation_goal", goal=map_goal.to_dict())
            self.devices.base.navigate(map_goal, float(self.cfg["navigation"]["timeout_s"]))
            self.emit("navigation_reached", pose=self.devices.base.pose().to_dict())

            self.set_stage(Stage.FRONT_SEARCH)
            coarse = self.devices.front.detect(label, float(self.cfg["vision"]["front_timeout_s"]))
            self._validate_detection(coarse, "front")
            self.emit("front_detection", detection=coarse.to_dict())

            self.set_stage(Stage.BASE_ALIGN)
            desired = float(self.cfg["navigation"]["front_standoff_m"])
            forward = max(0.0, coarse.distance_m - desired)
            forward = min(forward, float(self.cfg["navigation"]["max_visual_approach_m"]))
            self.devices.base.align(coarse.bearing_rad, forward,
                                    float(self.cfg["navigation"]["align_timeout_s"]))
            self.emit("base_aligned", commanded_bearing_rad=coarse.bearing_rad,
                      commanded_forward_m=forward, pose=self.devices.base.pose().to_dict())

            confirmed = self.devices.front.detect(label, float(self.cfg["vision"]["front_timeout_s"]))
            self._validate_detection(confirmed, "front")
            max_bearing = float(self.cfg["navigation"]["alignment_tolerance_rad"])
            max_distance = float(self.cfg["navigation"]["arm_activation_distance_m"])
            if abs(confirmed.bearing_rad) > max_bearing or confirmed.distance_m > max_distance:
                raise MissionError("底盘视觉对准复核失败，不允许伸出机械臂: "
                                   f"bearing={confirmed.bearing_rad:.3f}, distance={confirmed.distance_m:.3f}")
            self.emit("alignment_confirmed", detection=confirmed.to_dict())

            observe = self._pose("observe")
            self.devices.arm.move([observe], self.cfg["arm"]["speed"],
                                  float(self.cfg["arm"]["motion_timeout_s"]))
            self.set_stage(Stage.WRIST_LOCALIZE)
            fine = self.devices.wrist.detect(label, float(self.cfg["vision"]["wrist_timeout_s"]))
            self._validate_detection(fine, "wrist")
            if fine.position_base_m is None:
                raise MissionError("腕部视觉没有返回 arm_base 坐标")
            if fine.frame_id not in ("arm_base", "base_link"):
                raise MissionError(f"腕部坐标系必须是 arm_base/base_link，实际为 {fine.frame_id}")
            self.target_base = fine.position_base_m
            self._validate_target(self.target_base)
            self.emit("wrist_detection", detection=fine.to_dict())

            self._pick()
            self._place()
            self.set_stage(Stage.COMPLETE)
            self.emit("mission_complete", placed=bool(self.devices.world and self.devices.world.placed)
                      if self.devices.world else True)
            return MissionResult(self.mission_id, True, self.stage, "抓取放置完成",
                                 str(self.log_path), self.target_base)
        except Exception as exc:
            self._safe_abort(exc)
            return MissionResult(self.mission_id, False, self.stage, str(exc),
                                 str(self.log_path), self.target_base)

    def _preflight(self) -> None:
        self.set_stage(Stage.PREFLIGHT)
        checks = {
            "base": self.devices.base.health(),
            "front_camera": self.devices.front.health(),
            "controller_arm": self.devices.arm.health(),
            "controller_gripper": self.devices.gripper.health(),
            "wrist_camera": self.devices.wrist.health(),
        }
        self.emit("health", devices=checks)
        failed = [name for name, value in checks.items() if not value.get("ok")]
        if checks["base"].get("localized") is False:
            failed.append("base_not_localized")
        if failed:
            raise MissionError("预检失败: " + ", ".join(failed))

    def _pick(self) -> None:
        assert self.target_base is not None
        self.set_stage(Stage.PICK)
        x, y, z = self.target_base
        orientation = self._pose("observe")
        grasp = Pose3D(x, y, z + float(self.cfg["arm"]["grasp_z_offset_m"]),
                       orientation.qx, orientation.qy, orientation.qz, orientation.qw)
        above = grasp.shifted(dz=float(self.cfg["arm"]["approach_height_m"]))
        lift = grasp.shifted(dz=float(self.cfg["arm"]["lift_height_m"]))
        for name, pose in (("above", above), ("grasp", grasp), ("lift", lift)):
            self._validate_pose(pose, name)
        self.devices.gripper.command("open", self.cfg["gripper"]["force_pct"],
                                     self.cfg["gripper"]["speed_pct"])
        self.devices.arm.move([above, grasp], self.cfg["arm"]["slow_speed"],
                              float(self.cfg["arm"]["motion_timeout_s"]))
        self.devices.gripper.command("close", self.cfg["gripper"]["force_pct"],
                                     self.cfg["gripper"]["speed_pct"])
        time.sleep(float(self.cfg["gripper"]["settle_s"]))
        self.devices.arm.move([lift], self.cfg["arm"]["slow_speed"],
                              float(self.cfg["arm"]["motion_timeout_s"]))
        self.emit("pick_complete", above=above.to_list(), grasp=grasp.to_list(), lift=lift.to_list())

    def _place(self) -> None:
        self.set_stage(Stage.PLACE)
        above = self._pose("place_above")
        place = self._pose("place")
        home = self._pose("home")
        for name, pose in (("place_above", above), ("place", place), ("home", home)):
            self._validate_pose(pose, name)
        self.devices.arm.move([above, place], self.cfg["arm"]["slow_speed"],
                              float(self.cfg["arm"]["motion_timeout_s"]))
        self.devices.gripper.command("open", self.cfg["gripper"]["force_pct"],
                                     self.cfg["gripper"]["speed_pct"])
        self.devices.arm.move([above, home], self.cfg["arm"]["speed"],
                              float(self.cfg["arm"]["motion_timeout_s"]))
        self.emit("place_complete", place=place.to_list(), home=home.to_list())

    def _pose(self, name: str) -> Pose3D:
        return Pose3D.from_list(self.cfg["arm"]["poses"][name])

    def _validate_target(self, target: tuple[float, float, float]) -> None:
        if any(not math.isfinite(v) for v in target):
            raise MissionError("腕部定位坐标含 NaN/Inf")
        self._validate_pose(Pose3D(*target), "vision_target")

    def _validate_detection(self, detection, camera: str) -> None:
        minimum = float(self.cfg["vision"][f"{camera}_min_confidence"])
        if detection.confidence < minimum:
            raise MissionError(f"{camera} 识别置信度 {detection.confidence:.2f} 低于 {minimum:.2f}")
        max_age = float(self.cfg["vision"]["max_detection_age_s"])
        age = time.time() - detection.timestamp_s
        if detection.timestamp_s <= 0 or age < -0.2 or age > max_age:
            raise MissionError(f"{camera} 识别结果已过期或时钟不同步: age={age:.3f}s")

    def _validate_pose(self, pose: Pose3D, name: str) -> None:
        limits = self.cfg["safety"]["arm_workspace_m"]
        for axis, value in (("x", pose.x), ("y", pose.y), ("z", pose.z)):
            low, high = map(float, limits[axis])
            if not low <= value <= high:
                raise MissionError(f"{name}.{axis}={value:.3f}m 超出安全范围 [{low},{high}]")

    def _safe_abort(self, exc: Exception) -> None:
        self.stage = Stage.SAFE_ABORT
        self.emit("mission_error", error=type(exc).__name__, message=str(exc))
        self.devices.base.stop()
        self.devices.arm.stop()
        # Do not automatically open after a possible successful grasp: dropping is worse.
        self.emit("outputs_stopped", gripper_policy="hold_last_state")
