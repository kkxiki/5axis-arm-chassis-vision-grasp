#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from v4_robot.bottle_plan import BottlePlanError, build_bottle_pick_plan, load_handeye


def main() -> int:
    parser = argparse.ArgumentParser(description="V4 水瓶抓取 demo 就绪检查与离线规划（绝不发运动命令）")
    parser.add_argument("--config", default=str(ROOT / "config" / "bottle_demo_v4.json"))
    parser.add_argument("--target", nargs=3, type=float, metavar=("X", "Y", "Z"),
                        help="已确认的水瓶 arm_base 坐标，单位 m")
    args = parser.parse_args()
    config_path = Path(args.config).resolve()
    cfg = json.loads(config_path.read_text(encoding="utf-8"))
    local_handeye_path = (config_path.parent / cfg["handeye"]["result_path"]).resolve()
    jetson_handeye_path = Path(cfg["handeye"].get("jetson_result_path", ""))
    handeye_path = (jetson_handeye_path if jetson_handeye_path.is_file()
                    else local_handeye_path)
    handeye = load_handeye(handeye_path)
    missing = [name for name, value in cfg["poses"].items() if value.get("confirmed") is not True]
    report = {
        "safe_mode": "offline_plan_only",
        "motion_commands_sent": False,
        "robot_model": cfg["robot_model"],
        "handeye": {
            "accepted": cfg["handeye"].get("accepted") is True,
            "path": str(handeye_path),
            "method": handeye["selected_method"],
            "image_size": handeye["image_size"],
            "translation_rms_mm": handeye["methods"][handeye["selected_method"]]["consistency"]["translation_rms_mm"],
        },
        "metric_depth_required": True,
        "unconfirmed_v4_poses": missing,
        "ready_for_real_motion": not missing and cfg["handeye"].get("accepted") is True,
    }
    if args.target:
        try:
            report["pick_plan"] = build_bottle_pick_plan(cfg, args.target).to_dict()
        except BottlePlanError as exc:
            report["plan_error"] = str(exc)
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report["ready_for_real_motion"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
