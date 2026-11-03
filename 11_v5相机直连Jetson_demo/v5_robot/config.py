"""配置加载与校验模块。

负责把 JSON 配置文件读成字典（load_config），并在启动前做一次严格校验
（validate_config）：检查必需配置段是否齐全、机械臂安全包络是否合法；
在 real（真实硬件）模式下，还强制要求硬件与各项标定均已人工确认，
且不允许残留 CHANGE_ME / 0.0.0.0 这类占位地址，避免带病上真机。
sim（仿真）模式只做结构校验，不要求标定确认。
"""
from __future__ import annotations

import json
from pathlib import Path
from typing import Any


class ConfigError(ValueError):
    """配置相关的错误。

    继承 ValueError，方便上层用同一类异常统一处理配置问题。
    """

    pass


def load_config(path: str | Path) -> dict[str, Any]:
    """读取并解析 JSON 配置文件。

    参数 path：配置文件路径（相对路径会被解析成绝对路径后再读取）。
    返回：配置字典；额外写入键 "_config_path"，记录配置文件的绝对路径，
        便于日志中追溯本次实际加载的是哪份配置。
    异常：文件不存在或 JSON 语法错误时抛 ConfigError（保留原始异常链）。
    """
    config_path = Path(path).resolve()
    try:
        data = json.loads(config_path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise ConfigError(f"配置文件不存在: {config_path}") from exc
    except json.JSONDecodeError as exc:
        raise ConfigError(f"配置文件 JSON 错误: {exc}") from exc
    # 记下实际使用的配置文件绝对路径，供日志与问题排查使用
    data["_config_path"] = str(config_path)
    return data


def validate_config(cfg: dict[str, Any], mode: str) -> None:
    """校验配置内容，不合法时直接抛 ConfigError。

    参数：
        cfg：load_config 返回的配置字典；
        mode："real" 表示真实硬件模式，其余取值（如 "sim"）视为仿真模式。

    校验内容：
        1. 必需配置段齐全：network/mission/navigation/arm/gripper/vision/safety；
        2. safety.arm_workspace_m 的 x/y/z 都是 [min, max] 且 min < max；
        3. 仅 real 模式：hardware_confirmed 与 calibration 下三项标定的 confirmed
           必须全为 True，同时 network 段不得残留 CHANGE_ME / 0.0.0.0 占位地址。
    """
    # 第一重校验：配置段是否齐全
    required = ("network", "mission", "navigation", "arm", "gripper", "vision", "safety")
    missing = [name for name in required if name not in cfg]
    if missing:
        raise ConfigError("缺少配置段: " + ", ".join(missing))

    # 第二重校验：安全包络必须是合法的 [min, max] 区间（min 必须小于 max），
    # 该区间后续用于拒绝越界的目标点与机械臂点位
    workspace = cfg["safety"].get("arm_workspace_m", {})
    for axis in ("x", "y", "z"):
        bounds = workspace.get(axis)
        if not isinstance(bounds, list) or len(bounds) != 2 or bounds[0] >= bounds[1]:
            raise ConfigError(f"safety.arm_workspace_m.{axis} 必须是 [min,max]")

    if mode == "real":
        # 第三重校验：真实硬件门禁。以下几项必须由人工确认过才允许上真机，
        # 只要有一项不是 True（含缺失），就整体拒绝启动
        checks = {
            "hardware_confirmed": cfg.get("hardware_confirmed"),
            "wrist_hand_eye.confirmed": cfg.get("calibration", {}).get("wrist_hand_eye", {}).get("confirmed"),
            "front_camera_extrinsic.confirmed": cfg.get("calibration", {}).get("front_camera_extrinsic", {}).get("confirmed"),
            "base_to_arm.confirmed": cfg.get("calibration", {}).get("base_to_arm", {}).get("confirmed"),
        }
        bad = [name for name, value in checks.items() if value is not True]
        if bad:
            raise ConfigError("真实硬件模式已锁定；请标定并确认: " + ", ".join(bad))

        # 第四重校验：network 段里所有字符串值都不能含未替换的占位地址，
        # 例如模板里的 CHANGE_ME 或监听地址 0.0.0.0
        placeholders = []
        for key, value in cfg["network"].items():
            if isinstance(value, str) and ("CHANGE_ME" in value or "0.0.0.0" in value):
                placeholders.append(f"network.{key}")
        if placeholders:
            raise ConfigError("真实硬件模式仍有占位地址: " + ", ".join(placeholders))

