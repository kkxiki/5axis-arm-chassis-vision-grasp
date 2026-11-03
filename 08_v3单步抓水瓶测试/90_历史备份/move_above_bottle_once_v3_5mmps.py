#!/usr/bin/env python3
"""One-shot, guarded 3 mm/s base-frame move to 10 cm above the v3 bottle target."""

from __future__ import annotations

import json
import math
import os
import socket
import sys
import time
from pathlib import Path


TARGET_FILE = Path("/tmp/v3_bottle_target.json")
GATEWAY = ("127.0.0.1", 6200)
OWNER = "AUTO_TASK"
MAX_SPEED_MPS = 0.005
STOP_TOLERANCE_M = 0.005
TARGET_Z_OFFSET_M = 0.100
TIMEOUT_SEC = 140.0


class SafetyError(RuntimeError):
    pass


def quat_angle(a, b):
    dot = abs(sum(float(x) * float(y) for x, y in zip(a, b)))
    na = math.sqrt(sum(float(x) ** 2 for x in a))
    nb = math.sqrt(sum(float(x) ** 2 for x in b))
    return 2.0 * math.acos(max(-1.0, min(1.0, dot / (na * nb))))


class Gateway:
    def __init__(self):
        self.sock = socket.create_connection(GATEWAY, 3)
        self.sock.settimeout(2)
        self.file = self.sock.makefile("r")
        self.seq = int(time.time() * 1000)

    def request(self, obj):
        self.sock.sendall((json.dumps(obj, separators=(",", ":")) + "\n").encode())
        line = self.file.readline().strip()
        if not line:
            raise SafetyError("gateway disconnected")
        return json.loads(line)

    def state(self):
        return self.request({"cmd": "state"})

    def acquire(self):
        return self.request({"cmd": "set_owner", "owner": OWNER})

    def velocity(self, xyz, active=True):
        self.seq += 1
        return self.request({
            "cmd": "speedl_update",
            "owner": OWNER,
            "seq": self.seq,
            "coordinate": 0,
            "deadman": bool(active),
            "vel": [float(xyz[0]), float(xyz[1]), float(xyz[2]), 0.0, 0.0, 0.0],
        })

    def zero(self):
        for _ in range(3):
            try:
                self.velocity([0.0, 0.0, 0.0], active=False)
            except Exception:
                pass
            time.sleep(0.05)

    def release(self):
        try:
            return self.request({"cmd": "release_owner", "owner": OWNER})
        except Exception:
            return None

    def close(self):
        try:
            self.sock.close()
        except Exception:
            pass


def load_robot_state():
    sdk = Path(os.environ["ZHENSHI_SDK_ROOT"])
    sys.path[:0] = [str(sdk), str(sdk / "robot_state")]
    from robot_state.platform_loader import get_topic_module
    from robot_state.system_state_reader import (
        get_current_jointtarget,
        get_current_robottarget,
        has_rt_data,
    )
    get_topic_module().start_subscriber(os.environ.get("ZHENSHI_ROBOT_IP", "192.168.11.11"))
    deadline = time.time() + 6
    while time.time() < deadline and not has_rt_data():
        time.sleep(0.1)
    if not has_rt_data():
        raise SafetyError("realtime robot state unavailable")
    return get_current_robottarget, get_current_jointtarget, has_rt_data


def main():
    if not TARGET_FILE.exists() or time.time() - TARGET_FILE.stat().st_mtime > 180:
        raise SafetyError("bottle target is missing or older than 180 seconds")
    detected = json.loads(TARGET_FILE.read_text())
    bottle = [float(v) for v in detected["base_xyz_m"]]
    target = [bottle[0], bottle[1], bottle[2] + TARGET_Z_OFFSET_M]
    if not (0.20 <= target[0] <= 0.90 and -0.35 <= target[1] <= 0.45 and 0.30 <= target[2] <= 0.75):
        raise SafetyError(f"target outside guarded workspace: {target}")
    if float(detected.get("max_deviation_m", 1.0)) > 0.005:
        raise SafetyError("vision target is not stable enough")

    get_pose, get_joints, has_state = load_robot_state()
    initial_pose = [float(v) for v in get_pose(0)[:7]]
    initial_joints = [float(v) for v in get_joints(0)]
    initial_distance = math.dist(initial_pose[:3], target)
    if initial_distance > 0.38:
        raise SafetyError(f"requested move too long: {initial_distance:.3f} m")

    gateway = Gateway()
    result = {
        "bottle_base_xyz_m": bottle,
        "target_tcp_xyz_m": target,
        "initial_tcp": initial_pose,
        "max_speed_mps": MAX_SPEED_MPS,
        "status": "started",
    }
    print(json.dumps(result, ensure_ascii=False), flush=True)
    try:
        state = gateway.state()
        if not (state.get("state") == "READY" and state.get("connected") and state.get("arm_enabled")):
            raise SafetyError(f"gateway not ready: {state}")
        owner = gateway.acquire()
        if not owner.get("ok"):
            raise SafetyError(f"cannot acquire gateway owner: {owner}")

        previous_joints = initial_joints
        previous_pose = initial_pose
        previous_time = time.time()
        started = previous_time
        last_print = 0.0
        while True:
            now = time.time()
            if now - started > TIMEOUT_SEC:
                raise SafetyError("move timeout")
            if not has_state():
                raise SafetyError("lost realtime robot state")
            pose = [float(v) for v in get_pose(0)[:7]]
            joints = [float(v) for v in get_joints(0)]
            dt = max(1e-3, now - previous_time)
            tcp_step = math.dist(pose[:3], previous_pose[:3])
            joint_step = max(abs(a - b) for a, b in zip(joints, previous_joints))
            orientation_change = quat_angle(initial_pose[3:7], pose[3:7])
            if tcp_step / dt > 0.015:
                raise SafetyError(f"TCP overspeed guard: {tcp_step / dt:.4f} m/s")
            if joint_step / dt > 0.70:
                raise SafetyError(f"joint jump guard: {joint_step / dt:.3f} rad/s")
            if orientation_change > math.radians(12):
                raise SafetyError(f"orientation drift guard: {math.degrees(orientation_change):.2f} deg")

            error = [target[i] - pose[i] for i in range(3)]
            distance = math.sqrt(sum(v * v for v in error))
            if now - last_print >= 2.0:
                print(json.dumps({
                    "elapsed_sec": round(now - started, 1),
                    "tcp_xyz_m": [round(v, 6) for v in pose[:3]],
                    "remaining_mm": round(distance * 1000, 2),
                    "orientation_change_deg": round(math.degrees(orientation_change), 3),
                }), flush=True)
                last_print = now
            if distance <= STOP_TOLERANCE_M:
                gateway.zero()
                final_pose = [float(v) for v in get_pose(0)[:7]]
                result.update({
                    "status": "completed",
                    "final_tcp": final_pose,
                    "final_error_mm": math.dist(final_pose[:3], target) * 1000,
                    "elapsed_sec": time.time() - started,
                })
                Path("/tmp/v3_move_above_bottle_result.json").write_text(
                    json.dumps(result, ensure_ascii=False, indent=2) + "\n"
                )
                print(json.dumps(result, ensure_ascii=False, indent=2), flush=True)
                return

            commanded_speed = min(MAX_SPEED_MPS, max(0.0008, distance * 0.12))
            velocity = [v / distance * commanded_speed for v in error]
            sent_at = time.time()
            reply = gateway.velocity(velocity, active=True)
            if time.time() - sent_at > 0.45:
                raise SafetyError("gateway response latency exceeded 450 ms")
            if not reply.get("ok"):
                raise SafetyError(f"SpeedL rejected: {reply}")
            previous_pose, previous_joints, previous_time = pose, joints, now
            time.sleep(0.10)
    finally:
        gateway.zero()
        print("gateway_release", gateway.release(), flush=True)
        gateway.close()


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(json.dumps({"status": "aborted", "error": str(exc)}, ensure_ascii=False), flush=True)
        raise
