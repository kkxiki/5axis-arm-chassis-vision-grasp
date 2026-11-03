#!/usr/bin/env python3
"""Continuous V5 bottle pick/place loop with independent gateway recovery."""
from __future__ import annotations

import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent
ONCE = ROOT / "run_v5_fixed_demo_once.py"
GATEWAY = ROOT / "v5_gateway_manager.py"
CONTROLLER = "192.168.30.11"
NO_BOTTLE_EXIT_S = 120.0


def start_gateway() -> bool:
    cmd = [sys.executable, str(GATEWAY), "start", "--controller-ip", CONTROLLER]
    result = subprocess.run(cmd, text=True, capture_output=True, check=False)
    if result.stdout:
        print(result.stdout, end="", flush=True)
    if result.returncode != 0 and result.stderr:
        print(result.stderr, end="", file=sys.stderr, flush=True)
    return result.returncode == 0


def clear_gateway_error() -> bool:
    """Clear the controller error; do not restart or re-enable the arm."""
    cmd = [sys.executable, str(GATEWAY), "clear", "--controller-ip", CONTROLLER]
    result = subprocess.run(cmd, text=True, capture_output=True, check=False)
    if result.stdout:
        print(result.stdout, end="", flush=True)
    if result.returncode != 0 and result.stderr:
        print(result.stderr, end="", file=sys.stderr, flush=True)
    return result.returncode == 0


def recover_arm() -> bool:
    """失败恢复：若上一轮已夹住瓶子则补做放置，然后回到 Home，保证下一轮从已知位姿开始。"""
    cmd = [sys.executable, str(ONCE), "--recover", "--speed", "v25", "--slow-speed", "v25"]
    result = subprocess.run(cmd, text=True, capture_output=True, check=False)
    if result.stdout:
        print(result.stdout, end="", flush=True)
    if result.returncode != 0 and result.stderr:
        print(result.stderr, end="", file=sys.stderr, flush=True)
    return result.returncode == 0


def main() -> int:
    no_bottle_since = None
    gateway_started = False
    while True:
        # 启动时只确保网关进程存在；如果已经运行，管理器不会重启它。
        # 后续故障只清除控制器错误，不重启网关，也不重新使能机械臂。
        if not gateway_started:
            start_gateway()
            gateway_started = True
        cmd = [sys.executable, str(ONCE), "--speed", "v25", "--slow-speed", "v25"]
        result = subprocess.run(cmd, text=True, capture_output=True, check=False)
        if result.stdout:
            print(result.stdout, end="", flush=True)
        combined = (result.stdout or "") + (result.stderr or "")
        if result.returncode == 0:
            no_bottle_since = None
            print('{"loop": "cycle_complete", "next": "home_detect"}', flush=True)
            continue

        if "no bottle detected" in combined.lower():
            now = time.monotonic()
            if no_bottle_since is None:
                no_bottle_since = now
                print('{"loop": "home_waiting_no_bottle", "timeout_s": 120}', flush=True)
            elapsed = now - no_bottle_since
            if elapsed >= NO_BOTTLE_EXIT_S:
                print('{"loop": "exit_no_bottle", "elapsed_s": %.1f}' % elapsed, flush=True)
                return 0
            time.sleep(1.0)
            continue

        if result.stderr:
            print(result.stderr, end="", file=sys.stderr, flush=True)
        # 整轮中止时机械臂可能停在任意位姿（甚至夹着瓶子）；先清除错误，
        # 再补做放置 / 回到 Home，最后才重试，而不是在原地直接重跑整轮。
        print('{"loop": "cycle_error", "action": "clear_error_then_complete_place"}', flush=True)
        if not clear_gateway_error():
            print('{"loop": "clear_error_failed", "action": "stop"}', flush=True)
            return 2
        if recover_arm():
            print('{"loop": "arm_recovered", "next": "retry"}', flush=True)
        else:
            # 恢复不了说明网关或控制器仍未就绪，继续重试只会让机械臂
            # 停在未知位姿反复动作，因此停止并交人工处理。
            print('{"loop": "arm_recover_failed", "action": "stop"}', flush=True)
            return 2
        time.sleep(1.0)


if __name__ == "__main__":
    raise SystemExit(main())
