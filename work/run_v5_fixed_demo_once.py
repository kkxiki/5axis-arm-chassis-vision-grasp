#!/usr/bin/env python3
from __future__ import annotations

import argparse
import ctypes
import json
import math
import socket
import subprocess
import sys
import time
from pathlib import Path
from urllib.request import Request, urlopen

from v5_motion_commands import move_blend as build_move_blend

ROOT = Path(__file__).resolve().parents[1]
OWNER = "AUTO_TASK"
RECOVERY_STATE = ROOT / "runtime" / "v5_recovery_state.json"
# 网关每次运动后可能短暂进入 RECONNECTING，等待它恢复就绪再发下一条命令。
GATEWAY_READY_WAIT_S = 180.0


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

    def wait_ready(self, timeout: float = GATEWAY_READY_WAIT_S, poll_s: float = 1.0) -> bool:
        """Wait until the gateway reconnected to the controller and is READY."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                state = self.request({"cmd": "state"}, timeout=3.0)
            except Exception:
                time.sleep(poll_s)
                continue
            if state.get("connected") and str(state.get("state")) == "READY":
                return True
            time.sleep(poll_s)
        return False


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


class ControllerGripper:
    def __init__(self, host: str, cfg: dict):
        self.base_url = f"http://{host}"
        self.cfg = cfg

    def command(self, action: str, retries: int = 30, retry_delay_s: float = 1.0, quiet: bool = True) -> None:
        if action not in ("open", "close"):
            raise RuntimeError(f"unknown gripper action: {action}")
        distance_key = "open_distance" if action == "open" else "close_distance"
        payload = {
            "event_name": "MoveTwoFingersGripper",
            "user_action": "Click",
            "sub_commands": [{
                "selected": True,
                "distance": int(self.cfg[distance_key]),
                "interval": int(self.cfg["interval"]),
            }],
        }
        body = json.dumps(payload).encode("utf-8")
        last = None
        for attempt in range(1, retries + 1):
            req = Request(self.base_url + "/backend/send-event-cmd", body, {"Content-Type": "application/json"})
            with urlopen(req, timeout=5.0) as response:
                result = json.loads(response.read().decode("utf-8"))
            last = result
            if not quiet or int(result.get("failed", 0)) == 0:
                print(json.dumps({"event": "gripper", "action": action, "attempt": attempt, "ok": int(result.get("failed", 0)) == 0}, ensure_ascii=False), flush=True)
            if int(result.get("failed", 0)) == 0 and int(result.get("request_code", 0)) >= 0:
                return
            message = str(result.get("request_message", ""))
            retryable = "正在执行" in message or "不是合法 JSON" in message or "return_code=-1" in message
            if not retryable:
                break
            time.sleep(retry_delay_s)
        raise RuntimeError(f"gripper {action} failed after {retries} attempts: {last}")


def quat_angle(a: list[float], b: list[float]) -> float:
    dot = abs(sum(float(x) * float(y) for x, y in zip(a, b)))
    dot = max(-1.0, min(1.0, dot))
    return 2.0 * math.acos(dot)


def wait_pose(state: RobotState, target: list[float], label: str, timeout: float, quiet: bool = True) -> dict:
    deadline = time.time() + timeout
    last = None
    while time.time() < deadline:
        current = state.pose()
        pos_err = math.dist(current[:3], target[:3])
        rot_err = quat_angle(current[3:7], target[3:7])
        last = (pos_err, rot_err, current)
        if pos_err <= 0.004 and rot_err <= math.radians(3.0):
            report = {
                "event": "pose_reached",
                "label": label,
                "position_error_mm": pos_err * 1000.0,
                "orientation_error_deg": math.degrees(rot_err),
            }
            if not quiet:
                print(json.dumps(report, ensure_ascii=False), flush=True)
            return report
        time.sleep(0.25)
    pos_err, rot_err, current = last or (float("inf"), float("inf"), state.pose())
    raise RuntimeError(
        f"{label} pose timeout: pos_err_mm={pos_err*1000:.2f}, "
        f"rot_err_deg={math.degrees(rot_err):.2f}, current={current}"
    )


def move_blend(gw: Gateway, state: RobotState, targets: list[list[float]], label: str, speed: str, timeout: float, quiet: bool = True) -> dict:
    if not quiet:
        print(json.dumps({"event": "move_start", "label": label, "route_len": len(targets), "speed": speed}, ensure_ascii=False), flush=True)
    # 网关在每次运动后会短暂进入 RECONNECTING；等它恢复就绪再发 setup，
    # 否则会直接被 not_ready 拒掉，整轮中止。
    if not gw.wait_ready(timeout=GATEWAY_READY_WAIT_S):
        raise RuntimeError(f"{label}: gateway not ready after {GATEWAY_READY_WAIT_S:.0f}s")
    commands = build_move_blend(targets, speed)
    for raw in commands[:-1]:
        reply = send_gateway_setup(gw, raw, label)
        if not reply.get("ok"):
            raise RuntimeError(f"{label}: gateway rejected setup {raw}: {reply}")
    try:
        reply = gw.send_raw(commands[-1], timeout=timeout)
        if not reply.get("ok"):
            if not quiet:
                print(json.dumps({"event": "move_start_reply", "label": label, "reply": reply, "note": "verify final pose"}, ensure_ascii=False), flush=True)
    except Exception as exc:
        if not quiet:
            print(json.dumps({"event": "move_start_exception", "label": label, "error": str(exc), "note": "verify final pose"}, ensure_ascii=False), flush=True)
    return wait_pose(state, targets[-1], label, timeout, quiet=quiet)


def send_gateway_setup(gw: Gateway, raw: str, label: str) -> dict:
    """Send a MoveBlend setup command without treating gateway state as motion truth."""
    try:
        return gw.send_raw(raw, timeout=5.0)
    except Exception as exc:
        # The controller is the source of truth for motion; the gateway can be
        # between reconnects.  Give setup one quick retry, but do not wait on
        # the gateway's own long timeout model.
        print(json.dumps({"event": "gateway_setup_retry", "label": label, "error": str(exc)}, ensure_ascii=False), flush=True)
        time.sleep(0.5)
        return gw.send_raw(raw, timeout=5.0)


def write_recovery_state(grasped: bool, target, place_count) -> None:
    """记录本轮中止时的进度，供恢复流程判断是否需要补做放置。"""
    RECOVERY_STATE.parent.mkdir(parents=True, exist_ok=True)
    RECOVERY_STATE.write_text(json.dumps({
        "grasped": bool(grasped),
        "target_base_m": [float(value) for value in target] if target is not None else None,
        "place_count": int(place_count or 0),
    }, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def read_recovery_state() -> dict:
    try:
        return json.loads(RECOVERY_STATE.read_text(encoding="utf-8"))
    except (FileNotFoundError, json.JSONDecodeError):
        return {}


def clear_recovery_state() -> None:
    try:
        RECOVERY_STATE.unlink()
    except FileNotFoundError:
        pass


def recover_after_failure(gw: Gateway, state: RobotState, cfg: dict, gripper: ControllerGripper,
                          args: argparse.Namespace) -> int:
    """失败恢复：等网关就绪。若上一轮已夹住瓶子，先补做放置（放置上方→放置→放开夹爪→放置上方），
    再回 Home；否则直接回 Home。全程不改变"是否夹持"以外的夹爪动作。"""
    record = read_recovery_state()
    target = record.get("target_base_m")
    grasped = record.get("grasped") is True
    place_count = int(record.get("place_count") or 0)
    home = [float(value) for value in cfg["poses"]["home"]["robottarget"]]

    if not gw.wait_ready(timeout=GATEWAY_READY_WAIT_S):
        print(json.dumps({"ok": False, "recovered": "none", "error": "gateway not ready"},
                         ensure_ascii=False), flush=True)
        return 2

    owner = gw.request({"cmd": "set_owner", "owner": OWNER})
    if not owner.get("ok"):
        raise RuntimeError(f"failed to acquire gateway owner: {owner}")
    placed = False
    try:
        if grasped and isinstance(target, list) and len(target) == 3:
            # 上一轮抓取后中止，瓶子还夹在夹爪里：按同样的放置点补做放置，避免夹着瓶子回 Home。
            plan = run_json(["python3", "scripts/plan_v5_bottle_demo.py",
                             "--target", *map(str, target), "--place-count", str(place_count)])
            poses = plan["pick_plan"]["poses"]
            print(json.dumps({"event": "recover_place_start", "place_count": place_count},
                             ensure_ascii=False), flush=True)
            move_blend(gw, state, [poses["place_above"]], "recover_place_above", args.speed, args.timeout, quiet=not args.verbose)
            move_blend(gw, state, [poses["place"]], "recover_place_down", args.speed, args.timeout, quiet=not args.verbose)
            gripper.command("open")
            time.sleep(0.3)
            print(json.dumps({"event": "recover_placed"}, ensure_ascii=False), flush=True)
            move_blend(gw, state, [poses["place_above"]], "recover_return_place_above", args.speed, args.timeout, quiet=not args.verbose)
            placed = True
        move_blend(gw, state, [home], "recover_home", args.speed, args.timeout, quiet=not args.verbose)
        print(json.dumps({"ok": True, "recovered": "home", "placed": placed, "pose": state.pose()},
                         ensure_ascii=False), flush=True)
    finally:
        try:
            gw.request({"cmd": "release_owner", "owner": OWNER}, timeout=2.0)
        except Exception:
            pass
    clear_recovery_state()
    return 0


def run_json(cmd: list[str]) -> dict:
    completed = subprocess.run(cmd, text=True, capture_output=True, check=False)
    if completed.returncode != 0:
        raise RuntimeError(f"command failed {cmd}: {completed.stderr or completed.stdout}")
    decoder = json.JSONDecoder()
    parsed = []
    text = completed.stdout
    for index, char in enumerate(text):
        if char != "{":
            continue
        try:
            value, _ = decoder.raw_decode(text[index:])
        except json.JSONDecodeError:
            continue
        if isinstance(value, dict):
            parsed.append(value)
    if not parsed:
        raise RuntimeError(f"command did not emit JSON: {cmd}")
    return max(parsed, key=lambda item: len(item.keys()))


def emit(stage: str, **data) -> None:
    print(json.dumps({"stage": stage, **data}, ensure_ascii=False), flush=True)


def main() -> int:
    parser = argparse.ArgumentParser(description="Run one fixed V5 bottle pick/place cycle.")
    parser.add_argument("--config", default=str(ROOT / "config" / "bottle_demo_v5.json"))
    parser.add_argument("--gateway-host", default="127.0.0.1")
    parser.add_argument("--gateway-port", type=int, default=6200)
    parser.add_argument("--controller", default="192.168.30.11")
    parser.add_argument("--python-bin", default="/home/nvidia/robot_grasp-main/.venv/bin/python")
    parser.add_argument("--robot-state-sdk", default="/home/nvidia/codex/zhenshi_ros2_py/vendor/sdk_send_recevice_demo_py/Base_SDK_py/robot_state")
    parser.add_argument("--speed", default="v25")
    parser.add_argument("--slow-speed", default="v25")
    parser.add_argument("--timeout", type=float, default=180.0)
    parser.add_argument("--verbose", action="store_true")
    parser.add_argument("--resume-from-above", action="store_true",
                        help="continue from already-reached grasp_above using latest detection JSON")
    parser.add_argument("--resume-from-grasp", action="store_true",
                        help="continue from already-reached grasp pose using latest detection JSON")
    parser.add_argument("--resume-after-close", action="store_true",
                        help="continue after the gripper has already closed at grasp pose")
    parser.add_argument("--resume-from-place-above", action="store_true",
                        help="continue from already-reached place_above; assumes latest detection and place_count=0")
    parser.add_argument("--resume-from-place-photo", action="store_true",
                        help="continue from already-reached place_photo with grasped bottle")
    parser.add_argument("--recover", action="store_true",
                        help="failure recovery only: wait for the gateway, complete the pending place if a bottle "
                             "is still grasped, then move back to home; no detection")
    args = parser.parse_args()

    cfg = json.loads(Path(args.config).read_text(encoding="utf-8"))
    gw = Gateway(args.gateway_host, args.gateway_port)
    state = RobotState(args.controller, args.robot_state_sdk)
    gripper = ControllerGripper(args.controller, {
        "open_distance": 70,
        "close_distance": 0,
        "interval": 25,
    })

    if args.recover:
        return recover_after_failure(gw, state, cfg, gripper, args)

    owner = gw.request({"cmd": "set_owner", "owner": OWNER})
    if not owner.get("ok"):
        raise RuntimeError(f"failed to acquire gateway owner: {owner}")
    grasped = bool(args.resume_after_close or args.resume_from_place_above or args.resume_from_place_photo)
    target = None
    place_count = 0
    try:
        if args.resume_from_above or args.resume_from_grasp or args.resume_after_close or args.resume_from_place_above or args.resume_from_place_photo:
            detect = json.loads(Path("runtime/v5_bottle_detection_latest.json").read_text(encoding="utf-8"))
            print(json.dumps({
                "event": (
                    "resume_from_place_photo" if args.resume_from_place_photo
                    else "resume_from_place_above" if args.resume_from_place_above
                    else
                    "resume_after_close" if args.resume_after_close
                    else "resume_from_grasp" if args.resume_from_grasp
                    else "resume_from_above"
                ),
                "target_base_m": detect["position_base_m"],
            }, ensure_ascii=False), flush=True)
        else:
            detect = run_json([args.python_bin, "scripts/detect_v5_bottle_once.py", "--camera-url", "http://127.0.0.1:8092", "--controller", args.controller])
            emit("detected", confidence=round(float(detect.get("confidence", 0.0)), 3),
                 target_base_m=[round(float(v), 4) for v in detect["position_base_m"]])
        target = detect["position_base_m"]
        tmp_plan = run_json(["python3", "scripts/plan_v5_bottle_demo.py", "--target", *map(str, target), "--place-count", "0"])
        plan = tmp_plan
        poses = tmp_plan["pick_plan"]["poses"]

        if not args.resume_from_above and not args.resume_from_grasp and not args.resume_after_close and not args.resume_from_place_above and not args.resume_from_place_photo:
            gripper.command("open")
            move_blend(gw, state, [poses["above"]], "grasp_above", args.speed, args.timeout, quiet=not args.verbose)
        if not args.resume_from_grasp and not args.resume_after_close and not args.resume_from_place_above and not args.resume_from_place_photo:
            move_blend(gw, state, [poses["grasp"]], "grasp_down", args.speed, args.timeout, quiet=not args.verbose)
        if not args.resume_after_close and not args.resume_from_place_above and not args.resume_from_place_photo:
            gripper.command("close")
            time.sleep(0.5)
            emit("grasped")
            grasped = True
        if not args.resume_from_place_above and not args.resume_from_place_photo:
            place_photo = cfg["poses"]["place_photo"]["robottarget"]
            move_blend(gw, state, [poses["lift"]], "lift", args.speed, args.timeout, quiet=not args.verbose)
            move_blend(gw, state, [[float(value) for value in place_photo]], "place_photo", args.speed, args.timeout, quiet=not args.verbose)
            emit("at_place_photo")

        if args.resume_from_place_above:
            count = 0
        else:
            count = run_json([args.python_bin, "scripts/count_v5_place_bottles.py"])["count_in_roi"]
            emit("place_counted", count=int(count))
            plan = run_json(["python3", "scripts/plan_v5_bottle_demo.py", "--target", *map(str, target), "--place-count", str(count)])
            poses = plan["pick_plan"]["poses"]
        place_count = int(count)

        if not args.resume_from_place_above:
            move_blend(gw, state, [poses["place_above"]], "place_above", args.speed, args.timeout, quiet=not args.verbose)
            move_blend(gw, state, [poses["place"]], "place_down", args.speed, args.timeout, quiet=not args.verbose)
        else:
            move_blend(gw, state, [poses["place"]], "place_down", args.speed, args.timeout, quiet=not args.verbose)
        gripper.command("open")
        time.sleep(0.3)
        emit("placed")
        grasped = False
        move_blend(gw, state, [poses["place_above"]], "return_place_above", args.speed, args.timeout, quiet=not args.verbose)
        final_move = move_blend(gw, state, [poses["home"]], "return_home", args.speed, args.timeout, quiet=not args.verbose)

        print(json.dumps({
            "ok": True,
            "motion_commands_sent": True,
            "target_base_m": target,
            "place_count": count,
            "place_offset_m": plan["pick_plan"]["place_offset_m"],
            "final_position_error_mm": final_move.get("position_error_mm"),
            "final_pose": state.pose(),
        }, ensure_ascii=False, indent=2), flush=True)
        return 0
    except Exception:
        # 中止时记录进度（是否已夹住瓶子、目标点、放置计数），恢复流程据此决定
        # 是"补做放置再回 Home"还是"直接回 Home"。
        write_recovery_state(grasped, target, place_count)
        raise
    finally:
        try:
            gw.request({"cmd": "release_owner", "owner": OWNER}, timeout=2.0)
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
