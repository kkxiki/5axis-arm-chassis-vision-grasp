#!/usr/bin/env python3
from __future__ import annotations

import argparse
import ctypes
import json
import math
import socket
import sys
import time
from pathlib import Path

from v5_motion_commands import move_blend


ROOT = Path(__file__).resolve().parents[1]
OWNER = "AUTO_TASK"


class Gateway:
    def __init__(self, host: str, port: int):
        self.host = host
        self.port = port

    def request(self, payload: dict, timeout: float = 5.0) -> dict:
        with socket.create_connection((self.host, self.port), timeout=timeout) as sock:
            sock.settimeout(timeout)
            sock.sendall((json.dumps(payload, separators=(",", ":")) + "\n").encode("utf-8"))
            line = sock.makefile("r", encoding="utf-8").readline()
        if not line:
            raise RuntimeError(f"gateway returned no reply for {payload}")
        return json.loads(line)

    def send_raw(self, raw: str, timeout: float = 5.0) -> dict:
        if "MoveAbsJ" in raw:
            raise RuntimeError("V5 forbids MoveAbsJ; use BlendL inside MoveBlend")
        return self.request({"cmd": "send", "owner": OWNER, "raw": raw}, timeout)


class RobotState:
    def __init__(self, controller_ip: str, sdk_root: str):
        lib_root = Path(sdk_root) / "lib" / "linux" / "arm"
        for lib_name in ("libprotobuf.so.32", "libzmq.so.5"):
            lib_path = next(lib_root.glob(f"*/{lib_name}"), None)
            if lib_path is None:
                raise RuntimeError(f"missing SDK library: {lib_name}")
            ctypes.CDLL(str(lib_path), mode=ctypes.RTLD_GLOBAL)
        sys.path.insert(0, sdk_root)
        from platform_loader import get_topic_module
        from system_state_reader import get_current_robottarget, has_rt_data

        get_topic_module().start_subscriber(controller_ip)
        self._pose_fn = get_current_robottarget
        self._has = has_rt_data
        deadline = time.time() + 8.0
        while not self._has() and time.time() < deadline:
            time.sleep(0.1)
        if not self._has():
            raise RuntimeError("controller realtime robot state unavailable")

    def pose(self) -> list[float]:
        raw = self._pose_fn(0)
        if not isinstance(raw, list) or len(raw) < 7:
            raise RuntimeError(f"invalid robot pose: {raw!r}")
        return [float(value) for value in raw[:7]]


def quat_angle(a: list[float], b: list[float]) -> float:
    dot = abs(sum(float(x) * float(y) for x, y in zip(a, b)))
    dot = max(-1.0, min(1.0, dot))
    return 2.0 * math.acos(dot)


def wait_pose(state: RobotState, target: list[float], timeout: float) -> dict:
    deadline = time.time() + timeout
    last = None
    while time.time() < deadline:
        current = state.pose()
        pos_err = math.dist(current[:3], target[:3])
        rot_err = quat_angle(current[3:7], target[3:7])
        last = {"position_error_mm": pos_err * 1000.0, "orientation_error_deg": math.degrees(rot_err)}
        if pos_err <= 0.003 and rot_err <= math.radians(3.0):
            return {"ok": True, **last, "current": current}
        time.sleep(0.25)
    return {"ok": False, **(last or {}), "current": state.pose()}


def main() -> int:
    parser = argparse.ArgumentParser(description="Move V5 arm to home/observe via place_above.")
    parser.add_argument("--config", default=str(ROOT / "config" / "bottle_demo_v5.json"))
    parser.add_argument("--gateway-host", default="127.0.0.1")
    parser.add_argument("--gateway-port", type=int, default=6200)
    parser.add_argument("--controller", default="192.168.30.11")
    parser.add_argument("--speed", default="v10")
    parser.add_argument("--timeout", type=float, default=180.0)
    parser.add_argument("--robot-state-sdk", default="/home/nvidia/codex/zhenshi_ros2_py/vendor/sdk_send_recevice_demo_py/Base_SDK_py/robot_state")
    args = parser.parse_args()

    cfg = json.loads(Path(args.config).read_text(encoding="utf-8"))
    poses = cfg["poses"]
    place_above = [float(v) for v in poses["place_above"]["robottarget"]]
    home = [float(v) for v in poses["home"]["robottarget"]]

    gw = Gateway(args.gateway_host, args.gateway_port)
    state = RobotState(args.controller, args.robot_state_sdk)
    start_pose = state.pose()
    owner = gw.request({"cmd": "set_owner", "owner": OWNER})
    if not owner.get("ok"):
        raise RuntimeError(f"failed to acquire gateway owner: {owner}")
    try:
        commands = move_blend((place_above, home), args.speed)
        for raw in commands[:-1]:
            reply = gw.send_raw(raw, timeout=5.0)
            if not reply.get("ok"):
                raise RuntimeError(f"gateway rejected setup {raw}: {reply}")
        try:
            reply = gw.send_raw(commands[-1], timeout=args.timeout)
            if not reply.get("ok"):
                print(json.dumps({"start_reply": reply, "note": "will verify final pose"}, ensure_ascii=False), flush=True)
        except Exception as exc:
            print(json.dumps({"start_reply_error": str(exc), "note": "will verify final pose"}, ensure_ascii=False), flush=True)
        result = wait_pose(state, home, args.timeout)
        report = {
            "ok": result["ok"],
            "motion_commands_sent": True,
            "route": ["place_above", "home"],
            "speed": args.speed,
            "start_pose": start_pose,
            "place_above": place_above,
            "home": home,
            **result,
        }
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 0 if result["ok"] else 2
    finally:
        try:
            gw.request({"cmd": "release_owner", "owner": OWNER}, timeout=2.0)
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
