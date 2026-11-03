#!/usr/bin/env python3
"""v3 bottle pick-and-place demo (linear staged flow, no extra safety).

Stages:
  S1  start at the taught grasp-photo pose
  S2  YOLOv8-detect a bottle in the grasp area (class 39, fixed-pose hand-eye)
  S3  ONE MoveBlend: pass through the above-bottle point (XY = detection,
      Z+quat taught) as a WAYPOINT, then dwell at the grasp pose -> close
  S4  (part of S3) final target = grasp pose
  S5  ONE MoveBlend: rise back through the above-bottle high point then
      travel to the taught place-photo pose (wrist camera on the place area)
  S6  count bottles in the place area:
        0 bottles      -> taught place pose
        1 bottle       -> X - 0.125 m
        2 bottles      -> Y + 0.125 m
        3+ bottles     -> X - 0.125 m, Y + 0.125 m
  S7  ONE MoveBlend: place-above pass-through waypoint -> place pose -> open
  S8  return to the grasp-photo pose through the above-bottle high point

Every fixed point is loaded from poses/*.json.  Fine-tune the points by
editing the JSON values (the "teach pendant" equivalent).  The three
place-side files ship as placeholders: set their "confirmed": true only
after you have taught the real positions.

Simplified on purpose: no orientation drift / TCP speed / joint jump
guards, no resume modes, no persisted last_plan.  Motion completion is
verified against the realtime pose so that sequential MoveBlend stages
do not overlap.

Coexistence with the Xbox SpeedL teleop: the controller is not single
threaded, only commands of the SAME kind (e.g. two Motion queues) would
conflict.  The ArmGateway still tracks ONE control owner, so this program
claims ownership only while it runs and releases it when finished.  The
Xbox process can stay up the whole time: while this program owns the
gateway the Xbox sender is ignored, and the moment ownership is released
the Xbox re-claims it on its next tick.  No need to start/stop the Xbox
process around the demo.  If the Xbox currently holds control
(owner=xbox), the demo waits a short while for it to be released before
giving up.

Run (on the Jetson, gateway + vision stack already up):
    python3 bottle_pick_place_demo_v3.py --record <name>  # teach current TCP as a pose
    python3 bottle_pick_place_demo_v3.py                   # execute
    python3 bottle_pick_place_demo_v3.py --plan-only       # show targets, no motion

Teaching a point: jog the arm with the Xbox to the exact target, keep it
still, then run --record with one of: grasp_photo, above_ref, grasp_ref,
place_photo, place_above/place_mid, place.  The current realtime TCP pose
is written into poses/<file> and that pose becomes "confirmed".
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
POSE_FILES = {
    "grasp_photo": "grasp_photo_pose_v3.json",
    "above_ref": "bottle_above_pose_v3.json",
    "grasp_ref": "grasp_pose_v3.json",
    "place_photo": "place_photo_pose_v3.json",
    "place_mid": "place_mid_pose_v3.json",
    "place": "place_pose_v3.json",
}
TARGET_FILE = Path("/tmp/v3_bottle_target.json")
DETECTOR = CALIBRATION_ROOT / "bottle_target_once_v3.py"
GRIPPER = CALIBRATION_ROOT / "ctag2f120_control.py"
GATEWAY_HOST = "127.0.0.1"
GATEWAY_PORT = 6200
OWNER = "AUTO_TASK"  # gateway normalizes names; only XBOX/AUTO_TASK/MOVEIT are accepted
SDK_ROOT = Path("/home/nvidia/codex/zhenshi_ros2_py/vendor/sdk_send_recevice_demo_py/Base_SDK_py")
REACH_TOL_M = 0.003
DETECT_RETRIES = 3
GRASP_Z_EXTRA_M = -0.01  # grip 1 cm lower than the compensated taught Z
DEFAULT_GRIP_SETTLE_SEC = 1.5
# Allow a short settling window, then one gateway recovery nudge can happen.
# This does not issue the controller's explicit enable command.
PASSIVE_READY_SEC = 2.0


class DemoError(RuntimeError):
    pass


def load_pose(name: str) -> list[float]:
    """Return the 7-value [x,y,z,qx,qy,qz,qw] robottarget of a pose file."""
    path = POSE_ROOT / POSE_FILES[name]
    if not path.exists():
        raise DemoError(f"missing pose file: {path}")
    data = json.loads(path.read_text(encoding="utf-8"))
    pose = data.get("robottarget")
    if not isinstance(pose, list) or len(pose) < 7:
        raise DemoError(f"invalid robottarget in {path}")
    return [float(v) for v in pose[:7]]


def place_pose_confirmed() -> bool:
    """The three place-side files must be taught (confirmed) before S5-S8 run."""
    for name in ("place_photo", "place_mid", "place"):
        path = POSE_ROOT / POSE_FILES[name]
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
        except Exception:
            return False
        if not data.get("confirmed", False):
            return False
    return True


class Gateway:
    """Minimal ArmGateway JSON client (no owner juggling beyond acquire)."""

    def __init__(self):
        self.sock = None
        self.file = None
        self.connect()
        self.seq = int(time.time() * 1000)

    def connect(self) -> None:
        self.sock = socket.create_connection((GATEWAY_HOST, GATEWAY_PORT), 3)
        self.sock.settimeout(2)
        self.file = self.sock.makefile("r")

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

    def request(self, obj: dict, timeout: float = 5.0) -> dict:
        old = self.sock.gettimeout()
        self.sock.settimeout(timeout)
        try:
            self.sock.sendall((json.dumps(obj, separators=(",", ":")) + "\n").encode())
            line = self.file.readline().strip()
            if not line:
                raise DemoError("ArmGateway disconnected")
            return json.loads(line)
        finally:
            self.sock.settimeout(old)

    def send_raw(self, raw: str, timeout: float = 5.0) -> dict:
        return self.request({"cmd": "send", "owner": OWNER, "raw": raw}, timeout)

    def ensure_ready(self, timeout: float = 60.0, allow_recovery: bool = True,
                     allow_enable: bool = True) -> dict:
        """Bring the gateway back to READY before each command stage.

        Long MoveBlend legs reliably drop the gateway transport to
        RECONNECTING on this controller.  When that happens the gateway
        re-initialises the controller itself (Clear/Recover/Enable), which is
        audible.  Hammering it with repeated resume_gateway/enable every 0.5 s
        made that re-enable churn several times per stop, so here we only
        nudge the recovery every few seconds and issue a single enable when
        the transport is back and the arm is not enabled yet.
        """
        deadline = time.time() + timeout
        last = None
        last_resume_at = 0.0
        passive_deadline = time.time() + PASSIVE_READY_SEC
        while time.time() < deadline:
            try:
                last = self.request({"cmd": "state"})
            except Exception:
                try:
                    self.close()
                    self.connect()
                    print("[GW] socket reconnected by client", flush=True)
                except Exception:
                    time.sleep(0.5)
                continue
            ready = last.get("connected") and last.get("state") == "READY" and last.get("arm_enabled")
            if ready:
                if last.get("owner") != OWNER:
                    reply = self.request({"cmd": "set_owner", "owner": OWNER})
                    if not reply.get("ok"):
                        if reply.get("error") == "owner_busy":
                            # Another live client (e.g. the Xbox teleop) owns
                            # the gateway right now.  It is NOT an error to
                            # coexist with that process: wait for it to drop
                            # to IDLE (or be released) and claim afterwards.
                            holder = reply.get("owner", "?")
                            print(f"[GW] control held by '{holder}'; waiting for release (no need to restart Xbox)", flush=True)
                            time.sleep(1.0)
                            continue
                        raise DemoError(f"cannot acquire gateway: {reply}")
                return last
            recovering = not last.get("connected") or last.get("state") in ("DISCONNECTED", "RECONNECTING")
            if recovering:
                if not allow_recovery or time.time() < passive_deadline:
                    print(f"[GW] passive wait (state={last.get('state')} connected={last.get('connected')})", flush=True)
                    time.sleep(1.0)
                    continue
                # Let the gateway recover on its own; only nudge it if it is
                # stuck for a few seconds instead of re-issuing every 0.5 s.
                if time.time() - last_resume_at > 5.0:
                    last_resume_at = time.time()
                    print(f"[GW] nudging recovery (state={last.get('state')} connected={last.get('connected')})", flush=True)
                    try:
                        self.request({"cmd": "resume_gateway"})
                    except Exception:
                        pass
                time.sleep(1.0)
                continue
            # Transport back (READY/CONNECTED) but the arm reports disabled:
            # a single Clear+Enable is enough to finish the recovery.
            if not last.get("arm_enabled"):
                if not allow_enable or time.time() < passive_deadline:
                    print(f"[GW] passive wait for arm enable (state={last.get('state')} arm_enabled=False)", flush=True)
                    time.sleep(1.0)
                    continue
                print(f"[GW] single enable (state={last.get('state')} arm_enabled=False)", flush=True)
                try:
                    self.request({"cmd": "enable"})
                except Exception:
                    pass
            time.sleep(1.0)
        raise DemoError(f"gateway did not become ready: {last}")

    def release(self) -> None:
        try:
            self.request({"cmd": "release_owner", "owner": OWNER})
        except Exception:
            pass


class RealState:
    """Realtime TCP pose reader through the official SDK."""

    def __init__(self, robot_ip: str):
        lib_root = SDK_ROOT / "robot_state" / "lib" / "linux" / "arm"
        for lib_name in ("libprotobuf.so.32", "libzmq.so.5"):
            lib_path = next(lib_root.glob(f"*/{lib_name}"), None)
            if lib_path is None:
                raise DemoError(f"missing SDK runtime library: {lib_name}")
            ctypes.CDLL(str(lib_path), mode=ctypes.RTLD_GLOBAL)
        sys.path[:0] = [str(SDK_ROOT), str(SDK_ROOT / "robot_state")]
        from robot_state.platform_loader import get_topic_module
        from robot_state.system_state_reader import get_current_robottarget, has_rt_data
        get_topic_module().start_subscriber(robot_ip)
        self._pose_fn = get_current_robottarget
        self._has = has_rt_data
        deadline = time.time() + 6
        while time.time() < deadline and not self._has():
            time.sleep(0.1)
        if not self._has():
            raise DemoError("realtime robot state unavailable")

    def pose(self) -> list[float]:
        raw = self._pose_fn(0)
        if not isinstance(raw, list) or len(raw) < 7:
            raise DemoError(f"invalid realtime pose: {raw}")
        return [float(v) for v in raw[:7]]


def move_blend(gw: Gateway, state: RealState, targets: list[list[float]], label: str,
               timeout: float = 180.0, allow_recovery: bool = True,
               allow_enable: bool = True) -> None:
    """Queue ONE MoveBlend through every target as a pass-through waypoint.

    All ``targets`` are inserted as ``insert_line`` entries before a single
    ``--type=start``, so the controller blends continuously through the
    intermediate points and only the final target is dwelled on.  Completion
    is verified against the realtime pose at the FINAL target.
    """
    if not targets:
        raise DemoError(f"{label}: empty waypoint list")
    gw.ensure_ready(allow_recovery=allow_recovery, allow_enable=allow_enable)
    lines = ["{MoveBlend --type=first_insert}"]
    for target in targets:
        values = ",".join(f"{v:.9f}" for v in target)
        lines.append(
            f"{{MoveBlend --type=insert_line --robottarget_value={{{values}}} --zone={{0.0,0.0}} --speed=v25}}"
        )
    for raw in lines:
        reply = gw.send_raw(raw, timeout=5.0)
        if not reply.get("ok"):
            raise DemoError(f"{label}: controller rejected setup: {reply}")
    # MoveBlend --type=start runs synchronously and may drop the transport
    # while executing (known behaviour on this controller).  Never resend it;
    # completion is decided from the realtime pose below.
    try:
        gw.send_raw("{MoveBlend --type=start}", timeout=180.0)
    except Exception as exc:
        print(f"[{label}] start reply unavailable ({exc}); waiting for pose", flush=True)
    final = targets[-1]
    deadline = time.time() + timeout
    while time.time() < deadline:
        current = state.pose()
        err_mm = math.dist(current[:3], final[:3]) * 1000.0
        print(f"[{label}] wait reach err={err_mm:.1f}mm", flush=True)
        if err_mm <= REACH_TOL_M * 1000.0:
            print(f"[MOVE] {label} done err={err_mm:.1f}mm", flush=True)
            try:
                s = gw.request({"cmd": "state"})
                print(f"[GW-STATE] after {label}: state={s.get('state')} connected={s.get('connected')} "
                      f"arm_enabled={s.get('arm_enabled')} owner={s.get('owner')}", flush=True)
            except Exception as exc:
                print(f"[GW-STATE] query failed after {label}: {exc}", flush=True)
            return
        time.sleep(0.3)
    raise DemoError(f"{label}: did not reach target within {timeout:.0f}s")


def run_detector(env_extra: dict | None = None) -> dict:
    """Run the YOLOv8 bottle locator and return its /tmp target JSON."""
    env = os.environ.copy()
    if env_extra:
        env.update(env_extra)
    env["PYTHONPATH"] = ("/opt/ros/humble/lib/python3.10/site-packages:"
                         "/opt/ros/humble/local/lib/python3.10/dist-packages:"
                         + env.get("PYTHONPATH", ""))
    last_error = "unknown detector error"
    for attempt in range(1, DETECT_RETRIES + 1):
        try:
            proc = subprocess.run(
                ["bash", "-lc", f"source /opt/ros/humble/setup.bash; exec python3 {DETECTOR}"],
                text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                timeout=30, env=env,
            )
            if proc.returncode != 0:
                last_error = proc.stdout[-800:]
            elif not TARGET_FILE.exists() or time.time() - TARGET_FILE.stat().st_mtime > 60:
                last_error = "detector did not produce a fresh target"
            else:
                target = json.loads(TARGET_FILE.read_text(encoding="utf-8"))
                if target.get("detected"):
                    return target
                last_error = f"detector returned not-detected: {target}"
        except Exception as exc:
            last_error = str(exc)
        if attempt < DETECT_RETRIES:
            print(f"[VISION] attempt {attempt}/{DETECT_RETRIES} failed; retrying", flush=True)
            time.sleep(1.0)
    raise DemoError(f"bottle detection failed after {DETECT_RETRIES} attempts:\n{last_error}")


def run_place_check(roi: list[float] | None) -> int:
    """Quick bottle-count probe at the place photo (<= ~9 s).

    Runs the detector in BOTTLE_CHECK_MODE and returns the largest stable
    number of YOLO bottle boxes whose centres are inside ``roi``.
    """
    env = os.environ.copy()
    if roi:
        env["BOTTLE_ROI_NORM"] = " ".join(str(v) for v in roi)
    env["BOTTLE_CHECK_MODE"] = "1"
    env["PYTHONPATH"] = ("/opt/ros/humble/lib/python3.10/site-packages:"
                         "/opt/ros/humble/local/lib/python3.10/dist-packages:"
                         + env.get("PYTHONPATH", ""))
    try:
        proc = subprocess.run(
            ["bash", "-lc", f"source /opt/ros/humble/setup.bash; exec python3 {DETECTOR}"],
            text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=6, env=env,
        )
    except Exception as exc:
        raise DemoError(f"place-check probe error: {exc}") from exc
    if proc.returncode != 0:
        raise DemoError(f"place-check probe failed: {proc.stdout[-600:]}")
    if not TARGET_FILE.exists() or time.time() - TARGET_FILE.stat().st_mtime > 60:
        raise DemoError("place-check probe produced no fresh target")
    target = json.loads(TARGET_FILE.read_text(encoding="utf-8"))
    return int(target.get("bottle_count", 1 if target.get("detected") else 0))


def gripper(action: str, force: int = 60, speed: int = 40) -> None:
    proc = subprocess.run(
        ["python3", str(GRIPPER), "--action", action, "--force", str(force), "--speed", str(speed)],
        text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=20,
    )
    if proc.returncode != 0:
        raise DemoError(f"gripper {action} failed: {proc.stdout[-400:]}")
    print(f"[GRIPPER] {action} -> {proc.stdout.strip()}", flush=True)


def grip_and_settle(state: RealState, force: int, speed: int, settle_sec: float) -> None:
    """Close the gripper, then hold still long enough for force to build."""
    gripper("close", force, speed)
    dwell_still(state, "S4_after_close_motion", 0.3)
    deadline = time.monotonic() + max(0.0, settle_sec)
    while time.monotonic() < deadline:
        time.sleep(0.1)
        state.pose()
    print(f"[GRIP] close command held for {settle_sec:.1f}s before carrying", flush=True)


def shifted(pose: list[float], delta: list[float]) -> list[float]:
    """Translate pose XYZ by delta, keep taught orientation."""
    return [pose[0] + delta[0], pose[1] + delta[1], pose[2] + delta[2]] + pose[3:7]


def load_grasp_ref_detection() -> list[float]:
    """Teach-time detection (at the photo pose) that grasp_ref was aligned to.

    The taught TCP and that reference detection define a fixed compensation
    K = taught_TCP - reference_detection.  At run time the grasp/above targets
    are the taught poses shifted by (live_detection - reference_detection), so
    the systematic eye-hand/gripper offset is removed instead of grasping at
    the raw detection point.
    """
    data = json.loads((POSE_ROOT / POSE_FILES["grasp_ref"]).read_text(encoding="utf-8"))
    ref = data.get("ref_detection_xyz")
    if not isinstance(ref, list) or len(ref) < 3:
        raise DemoError(
            "grasp_pose_v3.json is missing 'ref_detection_xyz' (the photo-pose "
            "detection of the bottle that grasp_ref was taught on); add it and "
            "re-run, or compensation is disabled."
        )
    return [float(v) for v in ref[:3]]


def load_place_check_roi() -> list[float] | None:
    """Optional normalized ROI [x0,y0,x1,y1] (0..1) stored in the place-photo
    pose file.  Only bottles whose bbox centre falls inside the ROI are
    considered by the place-area occupancy check."""
    data = json.loads((POSE_ROOT / POSE_FILES["place_photo"]).read_text(encoding="utf-8"))
    roi = data.get("check_roi_norm")
    if not isinstance(roi, list) or len(roi) != 4:
        return None
    return [float(v) for v in roi]


def record_pose(name: str, robot_ip: str) -> int:
    """Teach the CURRENT realtime TCP pose into poses/<name>.json.

    Read-only: does not acquire gateway ownership, so it can run while the
    Xbox teleop owns the gateway.  The arm must be still while sampling.
    """
    aliases = {"place_above": "place_mid"}
    name = aliases.get(name, name)
    if name not in POSE_FILES:
        names = ", ".join(list(POSE_FILES) + ["place_above"])
        print(f"[RECORD] unknown pose name '{name}'. valid names: {names}", flush=True)
        return 2
    path = POSE_ROOT / POSE_FILES[name]
    if not path.exists():
        print(f"[RECORD] pose file missing: {path}", flush=True)
        return 2
    state = RealState(robot_ip)
    stable = []
    previous = state.pose()
    while len(stable) < 10:
        time.sleep(0.15)
        current = state.pose()
        if math.dist(current[:3], previous[:3]) > 0.0005:
            stable = []
            print("[RECORD] arm is moving; hold it still to sample...", flush=True)
        else:
            stable.append(current)
        previous = current
    pose = stable[-1]
    data = json.loads(path.read_text(encoding="utf-8"))
    data["robottarget"] = [float(v) for v in pose]
    data["confirmed"] = True
    data["placeholder"] = False
    data["saved_at"] = time.time()
    data["note"] = f"taught live pose {name} ({len(stable)} stable samples)"
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"[RECORD] {name} -> {[round(v, 4) for v in pose[:3]]} saved & confirmed", flush=True)
    return 0


def dwell_still(state, label: str, seconds: float = 0.6) -> None:
    """Hold at the current pose until the arm stays within 0.5 mm for `seconds`.

    Used right after MoveBlend reaches a critical pose (e.g. the grasp point)
    so that a following gripper action is never issued while the arm is still
    creeping.  A safety cap avoids an endless loop if the arm never settles.
    """
    prev = state.pose()
    stable_since = time.monotonic()
    deadline = time.monotonic() + 10.0
    while time.monotonic() - stable_since < seconds:
        if time.monotonic() > deadline:
            print(f"[DWELL] {label}: arm did not fully settle within 10 s, proceeding", flush=True)
            return
        time.sleep(0.1)
        cur = state.pose()
        if math.dist(cur[:3], prev[:3]) > 0.0005:
            stable_since = time.monotonic()  # still moving, restart the window
        prev = cur
    print(f"[DWELL] {label} settled {seconds:.1f}s without drift", flush=True)


def main() -> int:
    parser = argparse.ArgumentParser(description="v3 bottle pick-and-place demo")
    parser.add_argument("--plan-only", action="store_true", help="print all stage targets, no motion")
    parser.add_argument("--allow-placeholders", action="store_true",
                        help="run even if place-side poses are still placeholders")
    parser.add_argument("--robot-ip", default="192.168.11.11")
    parser.add_argument("--force", type=int, default=60)
    parser.add_argument("--gripper-speed", type=int, default=40)
    parser.add_argument("--grip-settle-sec", type=float, default=DEFAULT_GRIP_SETTLE_SEC,
                        help="seconds to wait after close before moving with the bottle")
    parser.add_argument("--allow-gateway-reenable", action="store_true",
                        help="allow explicit resume_gateway/enable if the controller drops disabled")
    parser.add_argument("--record", metavar="POSE_NAME", default=None,
                        help="teach current TCP pose into a poses JSON and exit "
                             "(grasp_photo|above_ref|grasp_ref|place_photo|place_above|place_mid|place)")
    args = parser.parse_args()
    if args.record:
        return record_pose(args.record, args.robot_ip)

    poses = {name: load_pose(name) for name in POSE_FILES}
    for name, pose in poses.items():
        print(f"[POSE] {name}: {[round(v, 4) for v in pose[:3]]} q={[round(v, 4) for v in pose[3:7]]}", flush=True)

    if not place_pose_confirmed() and not args.allow_placeholders:
        print("[WARN] place-side poses (place_photo/place_mid/place) are not taught yet.", flush=True)
        print("[WARN] teach them and set \"confirmed\": true inside the JSON files,", flush=True)
        print("[WARN] or re-run with --allow-placeholders to skip this check.", flush=True)
        if args.plan_only:
            print("[PLAN_ONLY] no motion requested; placeholders only affect execution", flush=True)
            return 0
        return 2

    photo = poses["grasp_photo"]
    above_ref = poses["above_ref"]
    grasp_ref = poses["grasp_ref"]
    place_photo = poses["place_photo"]
    place_mid = poses["place_mid"]
    place = poses["place"]

    print("[PLAN] S1 grasp-photo:", [round(v, 4) for v in photo], flush=True)
    print("[PLAN] S5 place-photo :", [round(v, 4) for v in place_photo], flush=True)
    print("[PLAN] S7 place-above :", [round(v, 4) for v in place_mid], flush=True)
    print("[PLAN] S7 place       :", [round(v, 4) for v in place], flush=True)

    if args.plan_only:
        print("[PLAN_ONLY] no motion requested", flush=True)
        return 0

    gw = Gateway()
    state = RealState(args.robot_ip)
    try:
        # S1 - start at the grasp-photo pose
        move_blend(gw, state, [photo], "S1_to_grasp_photo",
                   allow_recovery=args.allow_gateway_reenable,
                   allow_enable=args.allow_gateway_reenable)

        # S2 - find a bottle in the grasp area (needs the arm at the photo pose)
        bottle = [float(v) for v in run_detector()["base_xyz_m"]]
        print(f"[VISION] bottle at {[round(v, 4) for v in bottle]}", flush=True)

        # S3/S4 - ONE MoveBlend: above-bottle as pass-through waypoint, then
        # dwell only at the grasp pose and close.  The targets are the TAUGHT
        # poses shifted by (live detection - teach reference detection) so the
        # systematic grasp offset is compensated (not the raw detection point).
        try:
            d0 = load_grasp_ref_detection()
            delta = [b - r for b, r in zip(bottle, d0)]
            print(f"[COMP] teach ref detection {[round(v, 4) for v in d0]}", flush=True)
            print(f"[COMP] delta (live - ref) = {[round(v, 4) for v in delta]}", flush=True)
        except DemoError as exc:
            print(f"[WARN] compensation disabled: {exc}", flush=True)
            delta = [0.0, 0.0, 0.0]
        above = shifted(above_ref, delta)
        grasp = shifted(grasp_ref, delta)
        grasp[2] += GRASP_Z_EXTRA_M  # grasp target Z is 1 cm lower by request
        print(f"[PLAN] S3 above-bottle (waypoint): {[round(v, 4) for v in above[:3]]}", flush=True)
        print(f"[PLAN] S4 grasp (stop)           : {[round(v, 4) for v in grasp[:3]]}", flush=True)
        gripper("open", args.force, args.gripper_speed)
        move_blend(gw, state, [above, grasp], "S3_above_then_S4_to_grasp")
        # S4 - dwell at the grasp pose until the arm is completely still, then
        # close the gripper and only then continue with S5.
        dwell_still(state, "S4_grasp", 0.6)
        grip_and_settle(state, args.force, args.gripper_speed, args.grip_settle_sec)

        # S5 - ONE MoveBlend carrying the bottle: rise back through the
        # above-bottle high point (waypoint) then travel to the place-photo pose.
        move_blend(gw, state, [above, place_photo], "S5_above_then_to_place_photo",
                   allow_recovery=args.allow_gateway_reenable,
                   allow_enable=args.allow_gateway_reenable)

        # S6 - is the place area already occupied?  Only the ROI stored in the
        # place-photo pose file is examined, so the gripped bottle or objects
        # outside the ROI do not cause a false positive.
        try:
            roi = load_place_check_roi()
            print(f"[PLACE-CHECK] ROI={roi if roi else 'full frame'}", flush=True)
            bottle_count = run_place_check(roi)
            print(f"[PLACE-CHECK] bottle_count={bottle_count}", flush=True)
        except DemoError as exc:
            print(f"[PLACE-CHECK] probe inconclusive (treated as empty): {exc}", flush=True)
            bottle_count = 0
        place_above_target = list(place_mid)
        place_target = list(place)
        dx = dy = 0.0
        if bottle_count == 1:
            dx = -0.125
            branch = "1 bottle -> X - 0.125 m"
        elif bottle_count == 2:
            dy = 0.125
            branch = "2 bottles -> Y + 0.125 m"
        elif bottle_count >= 3:
            dx, dy = -0.125, 0.125
            branch = "3+ bottles -> X - 0.125 m, Y + 0.125 m"
        else:
            branch = "0 bottles -> taught place pose"
        place_above_target[0] += dx
        place_above_target[1] += dy
        place_target[0] += dx
        place_target[1] += dy
        print(f"[PLACE-CHECK] {branch}", flush=True)
        print(f"[PLAN] S7 place-above: {[round(v, 4) for v in place_above_target[:3]]}", flush=True)
        print(f"[PLAN] S7 place      : {[round(v, 4) for v in place_target[:3]]}", flush=True)
        placed_offset = bool(dx or dy)

        # S7 - ONE MoveBlend: place-above pass-through waypoint, dwell at the
        # place pose, release.
        move_blend(gw, state, [place_above_target, place_target], "S7_place_above_then_to_place",
                   allow_recovery=True,
                   allow_enable=False)
        gripper("open", args.force, args.gripper_speed)

        # S8 - straight back to the grasp-photo pose (no need to swing over
        # the grasp/bottle area again).
        move_blend(gw, state, [photo], "S8_return_to_grasp_photo",
                   allow_recovery=True,
                   allow_enable=False)
        if placed_offset:
            print(f"[DONE] bottle placed with count-based offset, robot back at the grasp-photo pose", flush=True)
        else:
            print("[DONE] bottle placed, robot back at the grasp-photo pose", flush=True)
        return 0
    finally:
        gw.release()
        gw.close()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"[ABORT] {exc}", file=sys.stderr, flush=True)
        raise
