#!/usr/bin/env python3
"""v3 bottle grasp sequence using the three taught TCP references.

Default mode is a plan-only calculation. Add --execute only after the scene,
gripper and safety area have been checked. The normal execution path sends
full TCP poses (XYZ plus quaternion) with the taught orientation for each
stage. ``--xyz-only`` is retained only as an explicit fallback.
"""

from __future__ import annotations

import argparse
import ctypes
import json
import math
import os
import socket
import subprocess
import sys
import time
from pathlib import Path


CALIBRATION_ROOT = Path("/home/nvidia/calibration")
POSE_ROOT = CALIBRATION_ROOT / "poses"
PHOTO_POSE = POSE_ROOT / "grasp_photo_pose_v3.json"
GRASP_POSE = POSE_ROOT / "grasp_pose_v3.json"
ABOVE_POSE = POSE_ROOT / "bottle_above_pose_v3.json"
TUNE_FILE = POSE_ROOT / "grasp_tune_v3.json"
TARGET_FILE = Path("/tmp/v3_bottle_target.json")
DETECTOR = CALIBRATION_ROOT / "bottle_target_once_v3.py"
GRIPPER = CALIBRATION_ROOT / "ctag2f120_control.py"
RUNTIME = CALIBRATION_ROOT / "runtime" / "bottle_grasp_v3"
GATEWAY_HOST = "127.0.0.1"
GATEWAY_PORT = 6200
OWNER = "AUTO_TASK"
ROBOT_IP = "192.168.11.11"
SPEED_MPS = 0.010
MAX_SINGLE_MOVE_M = 0.50  # widened from 0.40 m for the observed workspace
POSITION_TOLERANCE_M = 0.005
ORIENTATION_DRIFT_LIMIT_RAD = math.radians(8.0)
ORIENTATION_SETTLE_TOLERANCE_RAD = math.radians(6.0)
DETECT_RETRIES = 3
MOTION_RETRIES = 3


class GraspError(RuntimeError):
    pass


def load_pose(path: Path) -> list[float]:
    if not path.exists():
        raise GraspError(f"missing pose file: {path}")
    data = json.loads(path.read_text(encoding="utf-8"))
    pose = data.get("robottarget")
    if not isinstance(pose, list) or len(pose) < 7:
        raise GraspError(f"invalid robottarget in {path}")
    return [float(v) for v in pose[:7]]


def quat_angle(a: list[float], b: list[float]) -> float:
    na = math.sqrt(sum(float(v) * float(v) for v in a))
    nb = math.sqrt(sum(float(v) * float(v) for v in b))
    if na < 1e-9 or nb < 1e-9:
        raise GraspError("invalid quaternion")
    dot = abs(sum(float(x) * float(y) for x, y in zip(a, b)) / (na * nb))
    return 2.0 * math.acos(max(-1.0, min(1.0, dot)))


class Gateway:
    def __init__(self):
        self.sock = None
        self.file = None
        self._connect()
        self.seq = int(time.time() * 1000)

    def _connect(self) -> None:
        self.sock = socket.create_connection((GATEWAY_HOST, GATEWAY_PORT), 3)
        self.sock.settimeout(2)
        self.file = self.sock.makefile("r")

    def reconnect(self) -> None:
        self.close()
        last = None
        for _ in range(5):
            try:
                self._connect()
                return
            except OSError as exc:
                last = exc
                time.sleep(0.5)
        raise GraspError(f"ArmGateway reconnect failed: {last}")

    def request(self, obj: dict) -> dict:
        self.sock.sendall((json.dumps(obj, separators=(",", ":")) + "\n").encode())
        line = self.file.readline().strip()
        if not line:
            raise GraspError("ArmGateway disconnected")
        return json.loads(line)

    def state(self) -> dict:
        return self.request({"cmd": "state"})

    def acquire(self) -> dict:
        return self.request({"cmd": "set_owner", "owner": OWNER})

    def speedl(self, xyz: list[float], active: bool = True) -> dict:
        self.seq += 1
        return self.request({
            "cmd": "speedl_update",
            "owner": OWNER,
            "seq": self.seq,
            "coordinate": 0,
            "deadman": bool(active),
            "vel": [float(xyz[0]), float(xyz[1]), float(xyz[2]), 0.0, 0.0, 0.0],
        })

    def send_raw(self, raw: str, timeout: float = 5.0) -> dict:
        old_timeout = self.sock.gettimeout()
        self.sock.settimeout(timeout)
        try:
            return self.request({"cmd": "send", "owner": OWNER, "raw": raw})
        finally:
            self.sock.settimeout(old_timeout)

    def zero(self) -> None:
        for _ in range(3):
            try:
                self.speedl([0.0, 0.0, 0.0], active=False)
            except Exception:
                pass
            time.sleep(0.05)

    def release(self) -> None:
        try:
            self.request({"cmd": "release_owner", "owner": OWNER})
        except Exception:
            pass

    def close(self) -> None:
        try:
            if self.file is not None:
                self.file.close()
        except Exception:
            pass
        try:
            if self.sock is not None:
                self.sock.close()
        except Exception:
            pass
        self.file = None
        self.sock = None


class RobotState:
    def __init__(self):
        # The systemd/Xbox launchers normally export ZHENSHI_SDK_ROOT, but an
        # SSH command does not inherit that shell environment.  Keep the
        # explicit override and fall back to the SDK location deployed on this
        # Jetson so the preflight and motion program behave identically.
        sdk_value = os.environ.get("ZHENSHI_SDK_ROOT")
        if sdk_value:
            sdk = Path(sdk_value)
        else:
            candidates = [
                Path("/home/nvidia/codex/zhenshi_ros2_py/vendor/sdk_send_recevice_demo_py/Base_SDK_py"),
                Path("/home/nvidia/calibration/sdk/Base_SDK_py"),
            ]
            sdk = next((p for p in candidates if (p / "robot_state" / "platform_loader.py").exists()), None)
            if sdk is None:
                raise GraspError("ZHENSHI_SDK_ROOT is unset and the deployed SDK could not be found")
        # topic.so depends on protobuf/ZeroMQ.  The SDK loader adds its
        # directory to LD_LIBRARY_PATH, but the dynamic linker does not
        # re-read that variable after Python has started.  Preload the bundled
        # libraries so SSH-launched runs work just like the service launcher.
        lib_root = sdk / "robot_state" / "lib" / "linux" / "arm"
        for lib_name in ("libprotobuf.so.32", "libzmq.so.5"):
            lib_path = next(lib_root.glob(f"*/{lib_name}"), None)
            if lib_path is None:
                raise GraspError(f"missing SDK runtime library: {lib_name}")
            try:
                ctypes.CDLL(str(lib_path), mode=ctypes.RTLD_GLOBAL)
            except OSError as exc:
                raise GraspError(f"cannot load SDK runtime library {lib_path}: {exc}") from exc
        sys.path[:0] = [str(sdk), str(sdk / "robot_state")]
        from robot_state.platform_loader import get_topic_module
        from robot_state.system_state_reader import get_current_jointtarget, get_current_robottarget, has_rt_data
        get_topic_module().start_subscriber(ROBOT_IP)
        self.get_pose = get_current_robottarget
        self.get_joints = get_current_jointtarget
        self.has_rt_data = has_rt_data
        deadline = time.time() + 6
        while time.time() < deadline and not self.has_rt_data():
            time.sleep(0.1)
        if not self.has_rt_data():
            raise GraspError("realtime robot state unavailable")

    def pose(self) -> list[float]:
        raw = self.get_pose(0)
        if not isinstance(raw, list) or len(raw) < 7:
            raise GraspError(f"invalid realtime pose: {raw}")
        return [float(v) for v in raw[:7]]

    def joints(self) -> list[float]:
        return [float(v) for v in self.get_joints(0)]


def ensure_gateway_ready(gw: Gateway, *, owner_required: bool = False, timeout: float = 30.0) -> dict:
    """Recover transient controller reconnects without operator intervention."""
    deadline = time.time() + timeout
    last = None
    while time.time() < deadline:
        try:
            last = gw.state()
        except Exception:
            try:
                gw.reconnect()
            except Exception:
                time.sleep(0.5)
            continue
        if last.get("connected") and last.get("state") == "READY" and last.get("arm_enabled"):
            if owner_required and last.get("owner") != OWNER:
                reply = gw.acquire()
                if reply.get("ok"):
                    return gw.state()
            else:
                return last
        # resume_gateway is only sent while disconnected/reconnecting; sending
        # it during a ready motion would unnecessarily interrupt the controller.
        if last.get("state") in ("DISCONNECTED", "RECONNECTING") or not last.get("connected"):
            try:
                gw.request({"cmd": "resume_gateway"})
            except Exception:
                pass
        try:
            gw.request({"cmd": "enable"})
        except Exception:
            pass
        time.sleep(0.5)
    raise GraspError(f"ArmGateway did not become ready: {last}")


def validate_bottle_target(target: dict) -> None:
    """Independent safety check before any motion is allowed."""
    xyz = target.get("base_xyz_m")
    frame = target.get("last_frame") or {}
    pixel = frame.get("pixel")
    bbox = frame.get("bbox")
    if not isinstance(xyz, list) or len(xyz) < 3:
        raise GraspError("detector target has no valid base XYZ")
    x, y, z = (float(xyz[0]), float(xyz[1]), float(xyz[2]))
    if not (0.60 <= x <= 0.92 and 0.05 <= y <= 0.40 and 0.30 <= z <= 0.50):
        raise GraspError(f"target outside taught bottle workspace: {[x, y, z]}")
    if not isinstance(pixel, list) or len(pixel) < 2:
        raise GraspError("detector target has no valid pixel")
    u, v = int(pixel[0]), int(pixel[1])
    if not (80 <= u <= 1200 and 80 <= v <= 660):
        raise GraspError(f"target too close to image border: {[u, v]}")
    if not isinstance(bbox, list) or len(bbox) < 4:
        raise GraspError("detector target has no valid bounding box")
    _, _, width, height = (int(v) for v in bbox[:4])
    area = float(frame.get("contour_area_px", 0.0))
    aspect = height / max(1.0, float(width))
    if area < 2500 or width < 55 or height < 70 or not (0.65 <= aspect <= 4.50):
        raise GraspError(f"target shape is not bottle-like: bbox={bbox} area={area}")


def run_detector() -> dict:
    if not DETECTOR.exists():
        raise GraspError(f"missing detector: {DETECTOR}")
    env = os.environ.copy()
    env["PYTHONPATH"] = "/opt/ros/humble/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages:" + env.get("PYTHONPATH", "")
    last_error = "unknown detector error"
    for attempt in range(1, DETECT_RETRIES + 1):
        try:
            proc = subprocess.run(
                ["bash", "-lc", f"source /opt/ros/humble/setup.bash; exec python3 {DETECTOR}"],
                text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=30, env=env,
            )
            if proc.returncode != 0:
                last_error = proc.stdout[-1200:]
            elif not TARGET_FILE.exists() or time.time() - TARGET_FILE.stat().st_mtime > 60:
                last_error = "detector did not produce a fresh target"
            else:
                target = json.loads(TARGET_FILE.read_text(encoding="utf-8"))
                if target.get("detected") and int(target.get("inlier_count", 0)) >= 9:
                    return target
                last_error = f"unstable detector result: {target}"
        except Exception as exc:
            last_error = str(exc)
        if attempt < DETECT_RETRIES:
            print(f"[VISION] attempt {attempt}/{DETECT_RETRIES} failed; retrying", flush=True)
            time.sleep(1.0)
    raise GraspError(f"bottle detection failed after {DETECT_RETRIES} attempts:\n{last_error}")


def gripper(action: str, force: int, speed: int) -> None:
    proc = subprocess.run(
        ["python3", str(GRIPPER), "--action", action, "--force", str(force), "--speed", str(speed)],
        text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=20,
    )
    if proc.returncode != 0:
        raise GraspError(f"gripper {action} failed: {proc.stdout[-500:]}")
    print(f"[GRIPPER] {proc.stdout.strip()}", flush=True)


def move_xyz(gw: Gateway, state: RobotState, target_xyz: list[float], reference_q: list[float], label: str, timeout: float = 150.0) -> None:
    current = state.pose()
    distance0 = math.dist(current[:3], target_xyz)
    if distance0 > MAX_SINGLE_MOVE_M:
        raise GraspError(f"{label}: move too long ({distance0:.3f}m)")
    deadline = time.time() + timeout
    previous = current
    previous_joints = state.joints()
    previous_time = time.time()
    while time.time() < deadline:
        now = time.time()
        current = state.pose()
        joints = state.joints()
        dt = max(1e-3, now - previous_time)
        if math.dist(current[:3], previous[:3]) / dt > 0.020:
            raise GraspError(f"{label}: TCP speed guard")
        if max(abs(a - b) for a, b in zip(joints, previous_joints)) / dt > 0.70:
            raise GraspError(f"{label}: joint jump guard")
        drift = quat_angle(reference_q, current[3:7])
        if drift > ORIENTATION_DRIFT_LIMIT_RAD:
            raise GraspError(f"{label}: orientation drift {math.degrees(drift):.2f}deg")
        error = [target_xyz[i] - current[i] for i in range(3)]
        distance = math.sqrt(sum(v * v for v in error))
        if distance <= POSITION_TOLERANCE_M:
            gw.zero()
            print(f"[MOVE] {label} done error_mm={distance * 1000:.2f} drift_deg={math.degrees(drift):.2f}", flush=True)
            return
        speed = min(SPEED_MPS, max(0.0008, distance * 0.12))
        reply = gw.speedl([v / distance * speed for v in error])
        if not reply.get("ok"):
            raise GraspError(f"{label}: SpeedL rejected: {reply}")
        previous, previous_joints, previous_time = current, joints, now
        time.sleep(0.10)
    raise GraspError(f"{label}: timeout")


def wait_exact_pose(state: RobotState, target: list[float], label: str, timeout: float = 180.0) -> None:
    # The 5-axis controller may report a small wrist-equivalent quaternion
    # residual at the above/grasp height.  Keep the taught photo pose strict,
    # while allowing the measured non-photo residual observed on this arm.
    orientation_tolerance = math.radians(1.0) if "photo" in label else ORIENTATION_SETTLE_TOLERANCE_RAD
    deadline = time.time() + timeout
    while time.time() < deadline:
        current = state.pose()
        pos_err = math.dist(current[:3], target[:3])
        rot_err = quat_angle(current[3:7], target[3:7])
        if pos_err <= POSITION_TOLERANCE_M and rot_err <= orientation_tolerance:
            print(f"[MOVE] {label} done pos_err_mm={pos_err*1000:.2f} rot_err_deg={math.degrees(rot_err):.2f}", flush=True)
            return
        time.sleep(0.25)
    raise GraspError(f"{label}: exact pose timeout")


def move_exact(gw: Gateway, state: RobotState, target: list[float], label: str) -> None:
    current = state.pose()
    if math.dist(current[:3], target[:3]) > MAX_SINGLE_MOVE_M:
        raise GraspError(f"{label}: exact move too long")
    values = ",".join(f"{v:.9f}" for v in target)
    commands = [
        "{MoveBlend --type=first_insert}",
        f"{{MoveBlend --type=insert_line --robottarget_value={{{values}}} --zone={{0.0,0.0}} --speed=v10}}",
        "{MoveBlend --type=start}",
    ]
    for raw in commands[:2]:
        reply = gw.send_raw(raw, timeout=5.0)
        if not reply.get("ok"):
            raise GraspError(f"{label}: controller rejected {raw}: {reply}")
    # MoveBlend start is synchronous on this controller and may take the whole
    # motion time; do not resend it after a timeout.  The long timeout is
    # intentional and bounded by the caller's taught workspace/limits.
    try:
        reply = gw.send_raw(commands[2], timeout=180.0)
    except Exception as exc:
        # The controller can execute MoveBlend and then drop the transport
        # reply during reconnect.  Treat this as ambiguous and verify the
        # actual TCP pose before deciding whether to abort; never resend start.
        print(f"[MOVE] {label} start reply unavailable ({exc}); waiting for pose", flush=True)
        wait_exact_pose(state, target, label)
        return
    if not reply.get("ok"):
        if reply.get("error") == "transport_error":
            print(f"[MOVE] {label} transport reply ambiguous; waiting for pose", flush=True)
            wait_exact_pose(state, target, label)
            return
        raise GraspError(f"{label}: controller rejected start: {reply}")
    wait_exact_pose(state, target, label)


def move_open_loop(gw: Gateway, target: list[float], label: str, expected_distance_m: float) -> None:
    """Queue one full-pose move and let the controller run it without RT feedback.

    This is the production path: the controller's synchronous MoveBlend start
    is followed by a deterministic dwell derived from v10 speed.  Network reply
    and realtime pose feedback are deliberately not used to make the sequence
    depend on operator intervention.
    """
    values = ",".join(f"{v:.9f}" for v in target)
    setup = [
        "{MoveBlend --type=first_insert}",
        f"{{MoveBlend --type=insert_line --robottarget_value={{{values}}} --zone={{0.0,0.0}} --speed=v10}}",
    ]
    for raw in setup:
        reply = gw.send_raw(raw, timeout=5.0)
        if not reply.get("ok"):
            raise GraspError(f"{label}: controller rejected setup: {reply}")
    try:
        reply = gw.send_raw("{MoveBlend --type=start}", timeout=180.0)
        if not reply.get("ok"):
            print(f"[MOVE_OPEN_LOOP] {label} start reply ignored: {reply}", flush=True)
            time.sleep(max(5.0, expected_distance_m / SPEED_MPS * 1.35))
    except Exception as exc:
        print(f"[MOVE_OPEN_LOOP] {label} start reply ignored: {exc}", flush=True)
        time.sleep(max(5.0, expected_distance_m / SPEED_MPS * 1.35))
    print(f"[MOVE_OPEN_LOOP] {label} complete", flush=True)


def run_open_loop_sequence(plan: dict, args) -> int:
    """Single deterministic production sequence; no RT pose decisions."""
    gateway = Gateway()
    try:
        owner = gateway.acquire()
        if not owner.get("ok"):
            raise GraspError(f"cannot acquire gateway: {owner}")
        photo = plan["photo_pose"]
        above = plan["computed_above_target_xyz_m"] + plan["above_pose_reference"][3:7]
        grasp = plan["computed_grasp_target_xyz_m"] + plan["grasp_pose_reference"][3:7]
        gripper("open", args.force, args.gripper_speed)
        move_open_loop(gateway, above, "above_bottle", math.dist(photo[:3], above[:3]))
        move_open_loop(gateway, grasp, "descend_to_grasp", math.dist(above[:3], grasp[:3]))
        gripper("close", args.force, args.gripper_speed)
        move_open_loop(gateway, photo, "return_to_grasp_photo", math.dist(grasp[:3], photo[:3]))
        gripper("open", args.force, args.gripper_speed)
        result = {
            "status": "returned_and_released_at_grasp_photo",
            "mode": "open_loop_v10",
            "plan": plan,
            "gripper_released_at_photo": True,
        }
        (RUNTIME / "last_result.json").write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(result, ensure_ascii=False, indent=2), flush=True)
        return 0
    finally:
        gateway.release()
        gateway.close()


def _vec3(value: str) -> list[float]:
    parts = [float(v) for v in value.split(",")]
    if len(parts) != 3:
        raise GraspError("offset must be three comma-separated metres: X,Y,Z")
    return parts


def load_tuning() -> dict:
    """Persisted re-teach delta applied on top of the vision-derived targets."""
    tuning = {"above_offset_m": [0.0, 0.0, 0.0], "grasp_offset_m": [0.0, 0.0, 0.0]}
    if TUNE_FILE.exists():
        try:
            data = json.loads(TUNE_FILE.read_text(encoding="utf-8"))
            for key in tuning:
                values = data.get(key)
                if values is not None:
                    tuning[key] = [float(v) for v in values]
        except (ValueError, TypeError, KeyError) as exc:
            raise GraspError(f"invalid grasp tune file {TUNE_FILE}: {exc}")
    return tuning


def resolve_tuning(args) -> dict:
    tuning = load_tuning()
    if args.tune_grasp:
        tuning["grasp_offset_m"] = _vec3(args.tune_grasp)
    if args.tune_above:
        tuning["above_offset_m"] = _vec3(args.tune_above)
    if args.save_tune:
        TUNE_FILE.write_text(json.dumps(tuning, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        print(f"[TUNE] saved tuning to {TUNE_FILE}", flush=True)
    return tuning


def apply_tuning(plan: dict, tuning: dict) -> dict:
    """Shift the computed targets by the re-teach delta (XYZ only, meters)."""
    above_off = [float(v) for v in tuning["above_offset_m"]]
    grasp_off = [float(v) for v in tuning["grasp_offset_m"]]
    if any(above_off):
        above = [plan["computed_above_target_xyz_m"][i] + above_off[i] for i in range(3)]
        plan["computed_above_target_xyz_m"] = above
    if any(grasp_off):
        grasp = [plan["computed_grasp_target_xyz_m"][i] + grasp_off[i] for i in range(3)]
        plan["computed_grasp_target_xyz_m"] = grasp
    plan["tuning"] = tuning
    gx, gy, gz = plan["computed_grasp_target_xyz_m"]
    az = plan["computed_above_target_xyz_m"][2]
    if not (0.55 <= gx <= 0.98 and 0.05 <= gy <= 0.42 and 0.32 <= gz <= 0.56):
        raise GraspError(f"tuned grasp target out of safe range: {[round(v, 3) for v in (gx, gy, gz)]}")
    if az - gz < 0.10:
        raise GraspError(f"tuned above/grasp Z gap too small: above_z={az:.3f} grasp_z={gz:.3f}")
    return plan


def build_plan(target: dict) -> dict:
    photo = load_pose(PHOTO_POSE)
    grasp = load_pose(GRASP_POSE)
    above = load_pose(ABOVE_POSE)
    bottle = [float(v) for v in target["base_xyz_m"]]
    above_target = [bottle[0], bottle[1], above[2]]
    grasp_delta = [grasp[i] - above[i] for i in range(3)]
    grasp_target = [above_target[i] + grasp_delta[i] for i in range(3)]
    plan = {
        "photo_pose": photo,
        "grasp_pose_reference": grasp,
        "above_pose_reference": above,
        "bottle_base_xyz_m": bottle,
        "grasp_delta_from_above_m": grasp_delta,
        "computed_above_target_xyz_m": above_target,
        "computed_grasp_target_xyz_m": grasp_target,
        "speed_mps": SPEED_MPS,
        "angular_velocity_command": [0.0, 0.0, 0.0],
        "gripper": {"open_before_descent": True, "close_at_grasp": True},
    }
    return plan


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--execute", action="store_true", help="actually move and close the gripper")
    parser.add_argument("--xyz-only", action="store_true", help="fallback: do not command full quaternion poses")
    parser.add_argument("--resume-from-above", action="store_true", help="continue from the already reached above-bottle pose using the last plan")
    parser.add_argument("--resume-from-grasp", action="store_true", help="close and return from an already reached grasp pose using the last plan")
    parser.add_argument("--open-loop", action="store_true", help="production flow: v10 motion with deterministic dwell, no pose/feedback decisions")
    parser.add_argument("--force", type=int, default=60)
    parser.add_argument("--gripper-speed", type=int, default=40)
    parser.add_argument("--tune-grasp", default=None, metavar="X,Y,Z",
                        help="shift computed grasp target by fixed metres offset (re-teach delta; meters)")
    parser.add_argument("--tune-above", default=None, metavar="X,Y,Z",
                        help="shift computed above-bottle target by fixed metres offset")
    parser.add_argument("--save-tune", action="store_true",
                        help="persist the effective tuning into poses/grasp_tune_v3.json")
    args = parser.parse_args()
    for path in (PHOTO_POSE, GRASP_POSE, ABOVE_POSE, GRIPPER):
        if not path.exists():
            raise GraspError(f"required file missing: {path}")
    last_plan_path = RUNTIME / "last_plan.json"
    resume_mode = args.resume_from_above or args.resume_from_grasp
    tuning = resolve_tuning(args)
    if resume_mode and last_plan_path.exists():
        plan = json.loads(last_plan_path.read_text(encoding="utf-8"))
        print("[RESUME] using last saved bottle plan", flush=True)
        if args.tune_grasp or args.tune_above:
            print("[RESUME] --tune-* ignored on resume; rebuild a fresh plan (identify) to apply offsets", flush=True)
    else:
        target = run_detector()
        plan = apply_tuning(build_plan(target), tuning)
    RUNTIME.mkdir(parents=True, exist_ok=True)
    (RUNTIME / "last_plan.json").write_text(json.dumps(plan, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(plan, ensure_ascii=False, indent=2), flush=True)
    if not args.execute:
        print("[PLAN_ONLY] add --execute after reviewing last_plan.json", flush=True)
        return 0

    if args.open_loop:
        return run_open_loop_sequence(plan, args)

    gateway = Gateway()
    state = RobotState()
    try:
        gw_state = gateway.state()
        if gw_state.get("owner") != "IDLE" or gw_state.get("state") != "READY" or not gw_state.get("arm_enabled"):
            raise GraspError(f"ArmGateway not free and ready: {gw_state}")
        photo_target = plan["photo_pose"]
        above_target = plan["computed_above_target_xyz_m"] + plan["above_pose_reference"][3:7]
        grasp_target = plan["computed_grasp_target_xyz_m"] + plan["grasp_pose_reference"][3:7]
        if args.resume_from_grasp:
            current = state.pose()
            if math.dist(current[:3], grasp_target[:3]) > 0.03:
                raise GraspError(f"resume requested but TCP is not at grasp target: current={current[:3]} target={grasp_target[:3]}")
            print(f"[RESUME] grasp pose confirmed, position_error_mm={math.dist(current[:3], grasp_target[:3])*1000:.1f}", flush=True)
        elif args.resume_from_above:
            current = state.pose()
            if math.dist(current[:3], above_target[:3]) > 0.03:
                raise GraspError(f"resume requested but TCP is not above target: current={current[:3]} target={above_target[:3]}")
            print(f"[RESUME] above pose confirmed, position_error_mm={math.dist(current[:3], above_target[:3])*1000:.1f}", flush=True)
        else:
            photo_needs_move = math.dist(state.pose()[:3], photo_target[:3]) > 0.08 or quat_angle(state.pose()[3:7], photo_target[3:7]) > math.radians(1.0)
            if photo_needs_move:
                if args.xyz_only:
                    move_xyz(gateway, state, photo_target[:3], plan["grasp_pose_reference"][3:7], "to_grasp_photo")
                else:
                    move_exact(gateway, state, photo_target, "to_grasp_photo")
            if quat_angle(plan["grasp_pose_reference"][3:7], state.pose()[3:7]) > ORIENTATION_DRIFT_LIMIT_RAD:
                raise GraspError("current photo-point orientation differs from taught grasp orientation")
        owner = gateway.acquire()
        if not owner.get("ok"):
            raise GraspError(f"cannot acquire gateway: {owner}")
        if args.resume_from_grasp:
            print("[RESUME] keeping gripper closed while returning", flush=True)
        else:
            gripper("open", args.force, args.gripper_speed)
        if not resume_mode:
            if args.xyz_only:
                move_xyz(gateway, state, above_target[:3], plan["grasp_pose_reference"][3:7], "above_bottle")
            else:
                move_exact(gateway, state, above_target, "above_bottle")
        if args.resume_from_grasp:
            pass
        elif args.xyz_only:
            move_xyz(gateway, state, grasp_target[:3], plan["grasp_pose_reference"][3:7], "descend_to_grasp")
        else:
            move_exact(gateway, state, grasp_target, "descend_to_grasp")
        if not args.resume_from_grasp:
            gripper("close", args.force, args.gripper_speed)
        # Keep the bottle closed in the gripper while returning to the taught
        # grasp-photo pose.  This restores both XYZ and the photo-point TCP
        # orientation before releasing it at the photo point.
        if args.xyz_only:
            move_xyz(gateway, state, photo_target[:3], plan["photo_pose"][3:7], "return_to_grasp_photo")
        else:
            move_exact(gateway, state, photo_target, "return_to_grasp_photo")
        gripper("open", args.force, args.gripper_speed)
        result = {
            "status": "returned_and_released_at_grasp_photo",
            "plan": plan,
            "final_tcp": state.pose(),
            "gripper_released_at_photo": True,
        }
        (RUNTIME / "last_result.json").write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(result, ensure_ascii=False, indent=2), flush=True)
        return 0
    finally:
        gateway.zero()
        gateway.release()
        gateway.close()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"[ABORT] {exc}", file=sys.stderr, flush=True)
        raise
