#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from v5_robot.config import ConfigError, load_config, validate_config
from v5_robot.devices import build_devices
from v5_robot.mission import MissionRunner
from v5_robot.models import Pose2D


def main() -> int:
    parser = argparse.ArgumentParser(description="V5 相机直连 Jetson 抓取放置 demo")
    parser.add_argument("--config", default=str(ROOT / "config" / "v5_config.json"))
    parser.add_argument("--mode", choices=("sim", "real"), default="sim")
    parser.add_argument("--goal", nargs=3, metavar=("X", "Y", "YAW"), type=float,
                        help="覆盖地图粗目标，单位 m,m,rad")
    parser.add_argument("--label", default=None, help="目标类别，默认读取配置")
    parser.add_argument("--fail-at", choices=("base_health", "front_camera", "arm_health",
                        "gripper_health", "wrist_camera", "navigate", "front_detect",
                        "base_align", "arm_move", "wrist_detect", "gripper"),
                        help="仅仿真：注入故障验证安全中止")
    args = parser.parse_args()
    try:
        cfg = load_config(args.config)
        validate_config(cfg, args.mode)
        raw_goal = args.goal or cfg["mission"]["map_goal"]
        goal = Pose2D(float(raw_goal[0]), float(raw_goal[1]), float(raw_goal[2]))
        devices = build_devices(cfg, args.mode, args.fail_at)
        runner = MissionRunner(cfg, devices, ROOT / "runtime")
        result = runner.run(goal, args.label or cfg["mission"]["target_label"])
        print("\nRESULT=" + json.dumps(result.to_dict(), ensure_ascii=False, indent=2))
        return 0 if result.success else 2
    except ConfigError as exc:
        print(f"CONFIG_ERROR: {exc}", file=sys.stderr)
        return 3


if __name__ == "__main__":
    raise SystemExit(main())
