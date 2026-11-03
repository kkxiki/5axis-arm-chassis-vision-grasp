#!/usr/bin/env python3
"""Record a stationary V4 TCP pose. This script never sends motion commands."""
from __future__ import annotations

import argparse
import ctypes
import json
import os
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
POSE_NAMES = ("observe", "above_ref", "grasp_ref", "place_above", "place", "home")


def main() -> int:
    parser = argparse.ArgumentParser(description="记录当前 V4 TCP 位姿（只读，不控制机械臂）")
    parser.add_argument("name", choices=POSE_NAMES)
    parser.add_argument("--config", default=str(ROOT / "config" / "bottle_demo_v4.json"))
    parser.add_argument("--controller", default="192.168.30.11")
    parser.add_argument("--robot-state-sdk", default="/home/nvidia/codex/zhenshi_ros2_py/vendor/sdk_send_recevice_demo_py/Base_SDK_py/robot_state")
    parser.add_argument("--reference-detection", nargs=3, type=float, metavar=("X", "Y", "Z"),
                        help="仅 grasp_ref：示教水瓶在 arm_base 下的检测坐标")
    args = parser.parse_args()
    if args.reference_detection and args.name != "grasp_ref":
        parser.error("--reference-detection 只能与 grasp_ref 一起使用")

    config_path = Path(args.config).resolve()
    cfg = json.loads(config_path.read_text(encoding="utf-8"))
    lib_root = Path(args.robot_state_sdk) / "lib" / "linux" / "arm"
    for lib_name in ("libprotobuf.so.32", "libzmq.so.5"):
        lib_path = next(lib_root.glob(f"*/{lib_name}"), None)
        if lib_path is None:
            raise RuntimeError(f"缺少 SDK 动态库: {lib_name}")
        ctypes.CDLL(str(lib_path), mode=ctypes.RTLD_GLOBAL)
    sys.path.insert(0, args.robot_state_sdk)
    from platform_loader import get_topic_module
    from system_state_reader import get_current_robottarget, get_model_name, has_rt_data

    get_topic_module().start_subscriber(args.controller)
    deadline = time.time() + 8.0
    while not has_rt_data() and time.time() < deadline:
        time.sleep(0.1)
    if not has_rt_data():
        raise RuntimeError("控制器实时机械臂状态不可用")
    samples = []
    for _ in range(10):
        raw = get_current_robottarget(0)
        if not isinstance(raw, list) or len(raw) < 7:
            raise RuntimeError(f"无效 TCP 位姿: {raw!r}")
        samples.append([float(value) for value in raw[:7]])
        time.sleep(0.1)
    model = str(get_model_name(0))
    if model != cfg["robot_model"]:
        raise RuntimeError(f"机器人型号不匹配: 实际 {model}，配置 {cfg['robot_model']}")
    xyz_spread_mm = max(math_dist(samples[0][:3], item[:3]) for item in samples[1:]) * 1000.0
    if xyz_spread_mm > 0.5:
        raise RuntimeError(f"机械臂仍在运动，位置变化 {xyz_spread_mm:.3f} mm")

    entry = cfg["poses"][args.name]
    entry.update({"confirmed": True, "robot_model": model, "robottarget": samples[-1],
                  "saved_at": time.time(), "source": "official_sdk.get_current_robottarget(0)"})
    if args.reference_detection:
        entry["ref_detection_base_m"] = list(args.reference_detection)
    temporary = config_path.with_suffix(".json.tmp")
    temporary.write_text(json.dumps(cfg, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    os.replace(temporary, config_path)
    print(json.dumps({"ok": True, "name": args.name, "robot_model": model,
                      "pose": samples[-1], "stationary_translation_mm": xyz_spread_mm,
                      "motion_commands_sent": False}, ensure_ascii=False, indent=2))
    return 0


def math_dist(a: list[float], b: list[float]) -> float:
    return sum((x - y) ** 2 for x, y in zip(a, b)) ** 0.5


if __name__ == "__main__":
    raise SystemExit(main())
