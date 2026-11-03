#!/usr/bin/env python3
"""Restore the saved grasp-photo position and quaternion with MoveBlend."""

from __future__ import annotations

import json
import math
import os
import socket
import sys
import time
from pathlib import Path


POSE = Path("/home/nvidia/calibration/poses/grasp_photo_pose_v3.json")
OWNER = "AUTO_TASK"


def qangle(a, b):
    na = math.sqrt(sum(float(v) * float(v) for v in a))
    nb = math.sqrt(sum(float(v) * float(v) for v in b))
    dot = abs(sum(float(x) * float(y) for x, y in zip(a, b)) / (na * nb))
    return 2 * math.acos(max(-1.0, min(1.0, dot)))


class Gateway:
    def __init__(self):
        self.s = socket.create_connection(("127.0.0.1", 6200), 3)
        self.s.settimeout(4)
        self.f = self.s.makefile("r")

    def req(self, obj):
        self.s.sendall((json.dumps(obj, separators=(",", ":")) + "\n").encode())
        line = self.f.readline().strip()
        if not line:
            raise RuntimeError("gateway disconnected")
        return json.loads(line)

    def close(self):
        self.s.close()


def robot_state():
    sdk = Path(os.environ["ZHENSHI_SDK_ROOT"])
    sys.path[:0] = [str(sdk), str(sdk / "robot_state")]
    from robot_state.platform_loader import get_topic_module
    from robot_state.system_state_reader import get_current_robottarget, has_rt_data
    get_topic_module().start_subscriber(os.environ.get("ZHENSHI_ROBOT_IP", "192.168.11.11"))
    deadline = time.time() + 6
    while time.time() < deadline and not has_rt_data():
        time.sleep(0.1)
    if not has_rt_data():
        raise RuntimeError("no realtime robot state")
    return get_current_robottarget


def main():
    target = json.loads(POSE.read_text(encoding="utf-8"))["robottarget"][:7]
    target = [float(v) for v in target]
    get_pose = robot_state()
    current = [float(v) for v in get_pose(0)[:7]]
    if math.dist(current[:3], target[:3]) > 0.10:
        raise RuntimeError(f"current pose too far from photo point: {current[:3]} -> {target[:3]}")
    gw = Gateway()
    try:
        state = gw.req({"cmd": "state"})
        if state.get("owner") != "IDLE" or state.get("state") != "READY" or not state.get("arm_enabled"):
            raise RuntimeError(f"gateway not idle/ready: {state}")
        owner = gw.req({"cmd": "set_owner", "owner": OWNER})
        if not owner.get("ok"):
            raise RuntimeError(f"cannot acquire owner: {owner}")
        values = ",".join(f"{v:.9f}" for v in target)
        commands = [
            "{MoveBlend --type=first_insert}",
            f"{{MoveBlend --type=insert_line --robottarget_value={{{values}}} --zone={{0.0,0.0}} --speed=v5}}",
            "{MoveBlend --type=start}",
        ]
        for raw in commands:
            reply = gw.req({"cmd": "send", "owner": OWNER, "raw": raw})
            print(raw, reply, flush=True)
            if not reply.get("ok"):
                raise RuntimeError(f"controller rejected {raw}: {reply}")
        deadline = time.time() + 100
        while time.time() < deadline:
            current = [float(v) for v in get_pose(0)[:7]]
            pos_err = math.dist(current[:3], target[:3])
            rot_err = qangle(current[3:7], target[3:7])
            print(f"pos_err_mm={pos_err*1000:.2f} rot_err_deg={math.degrees(rot_err):.2f}", flush=True)
            if pos_err <= 0.004 and rot_err <= math.radians(1.0):
                print(json.dumps({"status": "restored", "target": target, "actual": current}, ensure_ascii=False, indent=2), flush=True)
                return
            time.sleep(0.5)
        raise RuntimeError("exact pose restore timeout")
    finally:
        try:
            gw.req({"cmd": "release_owner", "owner": OWNER})
        except Exception:
            pass
        gw.close()


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(f"[ABORT] {exc}", file=sys.stderr, flush=True)
        raise
