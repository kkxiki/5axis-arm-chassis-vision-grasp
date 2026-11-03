from __future__ import annotations

import json
from pathlib import Path
from typing import Any


class ConfigError(ValueError):
    pass


def load_config(path: str | Path) -> dict[str, Any]:
    config_path = Path(path).resolve()
    try:
        data = json.loads(config_path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise ConfigError(f"配置文件不存在: {config_path}") from exc
    except json.JSONDecodeError as exc:
        raise ConfigError(f"配置文件 JSON 错误: {exc}") from exc
    data["_config_path"] = str(config_path)
    return data


def validate_config(cfg: dict[str, Any], mode: str) -> None:
    required = ("network", "mission", "navigation", "arm", "gripper", "vision", "safety")
    missing = [name for name in required if name not in cfg]
    if missing:
        raise ConfigError("缺少配置段: " + ", ".join(missing))

    workspace = cfg["safety"].get("arm_workspace_m", {})
    for axis in ("x", "y", "z"):
        bounds = workspace.get(axis)
        if not isinstance(bounds, list) or len(bounds) != 2 or bounds[0] >= bounds[1]:
            raise ConfigError(f"safety.arm_workspace_m.{axis} 必须是 [min,max]")

    if mode == "real":
        checks = {
            "hardware_confirmed": cfg.get("hardware_confirmed"),
            "wrist_hand_eye.confirmed": cfg.get("calibration", {}).get("wrist_hand_eye", {}).get("confirmed"),
            "front_camera_extrinsic.confirmed": cfg.get("calibration", {}).get("front_camera_extrinsic", {}).get("confirmed"),
            "base_to_arm.confirmed": cfg.get("calibration", {}).get("base_to_arm", {}).get("confirmed"),
        }
        bad = [name for name, value in checks.items() if value is not True]
        if bad:
            raise ConfigError("真实硬件模式已锁定；请标定并确认: " + ", ".join(bad))

        placeholders = []
        for key, value in cfg["network"].items():
            if isinstance(value, str) and ("CHANGE_ME" in value or "0.0.0.0" in value):
                placeholders.append(f"network.{key}")
        if placeholders:
            raise ConfigError("真实硬件模式仍有占位地址: " + ", ".join(placeholders))

