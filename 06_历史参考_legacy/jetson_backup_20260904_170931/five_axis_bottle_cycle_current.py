#!/usr/bin/env python3
"""Five-axis bottle pick/place cycle for the Jetson deployment.

Flow:
1. Optionally start camera/YOLO/localizer stack.
2. Move to saved grasp photo pose.
3. Detect bottle and move above it.
4. Descend, close CTAG2F120 gripper, lift.
5. Move to saved place photo pose.
6. Detect whether the nominal place position is occupied.
7. If occupied, compute a side-offset XY place point; otherwise use nominal place.
8. Descend, open gripper, lift.
9. Return to saved grasp photo pose, release ArmGateway owner.

Motion uses SpeedL Cartesian velocity control with realtime TCP feedback read
directly from the Zhenshi state SDK; /tmp/five_axis_pose.json is only kept as a
live cache for the existing TF/vision bridge.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import signal
import socket
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path


POSE_JSON = Path("/tmp/five_axis_pose.json")
POSE_DIR = Path("/home/nvidia/calibration/poses")
GRASP_PHOTO = POSE_DIR / "grasp_photo_pose_1.json"
PLACE_POSE = POSE_DIR / "place_pose_1.json"
PLACE_PHOTO = POSE_DIR / "place_photo_pose_1.json"
RUNTIME_PLACE = POSE_DIR / "runtime_place_target.json"
LOG_DIR = Path("/home/nvidia/calibration/runtime")
GRIPPER_SCRIPT = Path("/home/nvidia/calibration/ctag2f120_control.py")
TF_BRIDGE = Path("/home/nvidia/five_axis_bottle_ws/src/five_axis_bottle_sort/five_axis_bottle_sort/five_axis_tf_bridge_node.py")
LOCALIZER = Path("/home/nvidia/five_axis_bottle_ws/src/five_axis_bottle_sort/five_axis_bottle_sort/wrist_localizer_node.py")
YOLO_NODE = Path("/home/nvidia/camera_1/yolov8_ros2_ws/src/yolov8_detector/yolov8_detector/yolov8_node.py")
YOLO_MODEL = Path("/home/nvidia/camera_1/yolov8_ros2_ws/yolov8s.pt")
VENV_PYTHON = Path("/home/nvidia/robot_grasp-main/.venv/bin/python")
SDK_ROOT = Path("/home/nvidia/codex/zhenshi_ros2_py/vendor/sdk_send_recevice_demo_py/Base_SDK_py")
SDK_LIB = SDK_ROOT / "robot_state/lib/linux/arm/22.04"


class CycleError(RuntimeError):
    pass


def ensure_sdk_process_env() -> None:
    lib = str(SDK_LIB)
    current = os.environ.get("LD_LIBRARY_PATH", "")
    if lib in current.split(":"):
        return
    env = os.environ.copy()
    env["LD_LIBRARY_PATH"] = lib + (":" + current if current else "")
    env["PYTHONPATH"] = str(SDK_ROOT) + ":" + str(SDK_ROOT / "robot_state") + (":" + env["PYTHONPATH"] if env.get("PYTHONPATH") else "")
    os.execve(sys.executable, [sys.executable] + sys.argv, env)


@dataclass
class Point:
    x: float
    y: float
    z: float

    def as_list(self) -> list[float]:
        return [self.x, self.y, self.z]

    def xy_distance(self, other: "Point") -> float:
        return math.hypot(self.x - other.x, self.y - other.y)


@dataclass
class BottleDetection:
    score: float
    point: Point


def run(cmd: list[str], timeout: float = 30.0, check: bool = True, env: dict | None = None) -> subprocess.CompletedProcess:
    proc = subprocess.run(cmd, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=timeout, env=env)
    if check and proc.returncode != 0:
        raise CycleError(f"command failed rc={proc.returncode}: {' '.join(cmd)}\n{proc.stdout}")
    return proc


def bash(script: str, timeout: float = 30.0, check: bool = True) -> subprocess.CompletedProcess:
    return run(["bash", "-lc", script], timeout=timeout, check=check)


def ros_env(extra_pythonpath: list[Path] | None = None) -> dict[str, str]:
    env = os.environ.copy()
    py = [
        "/opt/ros/humble/lib/python3.10/site-packages",
        "/opt/ros/humble/local/lib/python3.10/dist-packages",
    ]
    if extra_pythonpath:
        py = [str(p) for p in extra_pythonpath] + py
    if env.get("PYTHONPATH"):
        py.append(env["PYTHONPATH"])
    env["PYTHONPATH"] = ":".join(py)
    env["PATH"] = "/opt/ros/humble/bin:" + env.get("PATH", "")
    env["LD_LIBRARY_PATH"] = "/opt/ros/humble/lib:" + env.get("LD_LIBRARY_PATH", "")
    return env


def sdk_env(robot_ip: str) -> dict[str, str]:
    env = os.environ.copy()
    env["ZHENSHI_SDK_ROOT"] = str(SDK_ROOT)
    env["ZHENSHI_ROBOT_IP"] = robot_ip
    env["PYTHONPATH"] = str(SDK_ROOT) + ":" + str(SDK_ROOT / "robot_state") + (":" + env["PYTHONPATH"] if env.get("PYTHONPATH") else "")
    env["LD_LIBRARY_PATH"] = str(SDK_LIB) + (":" + env["LD_LIBRARY_PATH"] if env.get("LD_LIBRARY_PATH") else "")
    return env


def kill_camera_processes() -> None:
    for pat in ["realsense2_camera", "mjpeg_server.py", "yolov8_node.py", "wrist_localizer_node.py", "five_axis_tf_bridge_node.py", "direct_handeye_capture.py", "sdk_pose_writer"]:
        bash(
            "ps -eo pid,cmd | awk -v p=%r 'index($0,p)>0 && index($0,\"awk \")==0 {print $1}' | xargs -r kill"
            % pat,
            timeout=5,
            check=False,
        )


def start_background(name: str, cmd: list[str], log: Path, env: dict[str, str]) -> subprocess.Popen:
    log.parent.mkdir(parents=True, exist_ok=True)
    fp = open(log, "ab", buffering=0)
    proc = subprocess.Popen(cmd, stdout=fp, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL, start_new_session=True, env=env)
    print(f"[PROC] {name} pid={proc.pid} log={log}", flush=True)
    return proc


def start_background_bash(name: str, script: str, log: Path) -> subprocess.Popen:
    return start_background(name, ["bash", "-lc", script], log, os.environ.copy())


POSE_WRITER_CODE = r'''
import json, os, pathlib, sys, time
root = pathlib.Path(os.environ["ZHENSHI_SDK_ROOT"])
sys.path.insert(0, str(root))
sys.path.insert(0, str(root / "robot_state"))
from robot_state.platform_loader import get_topic_module
from robot_state.system_state_reader import get_current_jointtarget, get_current_robottarget, get_model_name, has_rt_data
robot_ip = os.environ.get("ZHENSHI_ROBOT_IP", "192.168.11.11")
out = pathlib.Path("/tmp/five_axis_pose.json")
get_topic_module().start_subscriber(robot_ip)
print("sdk_pose_writer started", robot_ip, flush=True)
while True:
    if has_rt_data():
        rt = get_current_robottarget(0)
        jt = get_current_jointtarget(0)
        if isinstance(rt, list) and len(rt) >= 7:
            tmp = out.with_suffix(".json.tmp")
            tmp.write_text(json.dumps({
                "time": time.time(),
                "model_index": 0,
                "model_name": get_model_name(0),
                "robottarget": [float(x) for x in rt[:7]],
                "jointtarget": [float(x) for x in jt] if isinstance(jt, list) else [],
                "source": "sdk_pose_writer",
            }), encoding="utf-8")
            tmp.replace(out)
    time.sleep(0.05)
'''


def start_camera_stack(args) -> list[subprocess.Popen]:
    kill_camera_processes()
    procs = []
    procs.append(start_background(
        "sdk_pose_writer",
        ["python3", "-u", "-c", POSE_WRITER_CODE],
        LOG_DIR / "sdk_pose_writer.log",
        sdk_env(args.robot_ip),
    ))
    time.sleep(1.0)
    procs.append(start_background_bash(
        "realsense",
        "source /opt/ros/humble/setup.bash; "
        "ros2 run realsense2_camera realsense2_camera_node "
        "--ros-args -r __ns:=/camera -r __node:=camera "
        "-p rgb_camera.color_profile:=1280x720x15 "
        "-p enable_depth:=true "
        "-p align_depth.enable:=true",
        LOG_DIR / "realsense.log",
    ))
    time.sleep(8)
    procs.append(start_background("tf_bridge", ["python3", str(TF_BRIDGE)], LOG_DIR / "five_axis_tf_bridge.log", ros_env([Path("/home/nvidia/five_axis_bottle_ws/src/five_axis_bottle_sort")])))
    procs.append(start_background("mjpeg", ["python3", "/home/nvidia/calibration/mjpeg_server.py"], LOG_DIR / "mjpeg_server.log", ros_env()))
    procs.append(start_background(
        "yolo",
        [
            str(VENV_PYTHON), str(YOLO_NODE), "--ros-args",
            "-p", f"model_path:={args.yolo_model}",
            "-p", "image_topic:=/camera/camera/color/image_raw",
            "-p", f"conf_threshold:={args.detect_conf}",
            "-p", f"device:={args.yolo_device}",
            "-p", "imgsz:=640",
            "-p", "debug:=true",
        ],
        LOG_DIR / "yolov8_detector.log",
        ros_env([YOLO_NODE.parent.parent]),
    ))
    time.sleep(6)
    procs.append(start_background(
        "localizer",
        [
            "python3", "-u", str(LOCALIZER), "--ros-args",
            "-p", "detection_topic:=/yolov8/detections",
            "-p", "depth_topic:=/camera/camera/aligned_depth_to_color/image_raw",
            "-p", "camera_info_topic:=/camera/camera/color/camera_info",
            "-p", "base_frame:=base_link",
            "-p", f"min_score:={args.detect_conf}",
            "-p", "stable_samples:=3",
        ],
        LOG_DIR / "wrist_localizer.log",
        ros_env([Path("/home/nvidia/five_axis_bottle_ws/src/five_axis_bottle_sort")]),
    ))
    time.sleep(args.stack_warmup_sec)
    return procs


class Gateway:
    def __init__(self, host: str, port: int, owner: str, seq_start: int):
        self.host = host
        self.port = port
        self.owner = owner
        self.seq = seq_start
        self.sock: socket.socket | None = None
        self.file = None

    @staticmethod
    def one_shot(obj: dict, host: str = "127.0.0.1", port: int = 6200, timeout: float = 5.0) -> dict:
        sock = socket.create_connection((host, port), timeout=2)
        sock.settimeout(timeout)
        sock.sendall((json.dumps(obj, separators=(",", ":")) + "\n").encode())
        line = sock.makefile().readline().strip()
        sock.close()
        if not line:
            raise CycleError("empty ArmGateway response")
        return json.loads(line)

    def connect(self) -> None:
        self.sock = socket.create_connection((self.host, self.port), timeout=2)
        self.sock.settimeout(5)
        self.file = self.sock.makefile("r")

    def request(self, obj: dict) -> dict:
        if self.sock is None or self.file is None:
            raise CycleError("gateway socket not connected")
        self.sock.sendall((json.dumps(obj, separators=(",", ":")) + "\n").encode())
        line = self.file.readline().strip()
        if not line:
            raise CycleError("ArmGateway disconnected")
        return json.loads(line)

    def set_owner(self) -> dict:
        return self.request({"cmd": "set_owner", "owner": self.owner})

    def release(self) -> dict:
        return self.request({"cmd": "release_owner", "owner": self.owner})

    def send_raw(self, raw: str) -> dict:
        return self.request({"cmd": "send", "owner": self.owner, "raw": raw})

    def speedl(self, vel: list[float], coordinate: int = 0, deadman: bool = True) -> dict:
        self.seq += 1
        return self.request({
            "cmd": "speedl_update",
            "owner": self.owner,
            "seq": self.seq,
            "coordinate": coordinate,
            "deadman": deadman,
            "vel": [float(v) for v in vel],
        })

    def speedl_stop(self) -> None:
        try:
            reply = self.speedl([0, 0, 0, 0, 0, 0], coordinate=0, deadman=False)
            print("[MOVE] speedl_stop", reply, flush=True)
        except Exception as exc:
            print("[MOVE] speedl_stop failed", exc, flush=True)

    def close(self) -> None:
        try:
            if self.sock:
                self.sock.close()
        except Exception:
            pass


class RobotState:
    def __init__(self, robot_ip: str, timeout_sec: float):
        self.robot_ip = robot_ip
        self.timeout_sec = timeout_sec
        sys_path_added = False
        import sys
        for path in (str(SDK_ROOT), str(SDK_ROOT / "robot_state")):
            if path not in sys.path:
                sys.path.insert(0, path)
                sys_path_added = True
        lib = str(SDK_LIB)
        os.environ["LD_LIBRARY_PATH"] = lib + (":" + os.environ["LD_LIBRARY_PATH"] if os.environ.get("LD_LIBRARY_PATH") else "")
        # The dynamic loader reads LD_LIBRARY_PATH at process start on some
        # systems. The script is normally started with sdk_env-compatible
        # values; this assignment is kept for direct interactive runs.
        from robot_state.platform_loader import get_topic_module
        from robot_state.system_state_reader import get_current_jointtarget, get_current_robottarget, get_model_count_rt, get_model_name, has_rt_data
        self._get_current_jointtarget = get_current_jointtarget
        self._get_current_robottarget = get_current_robottarget
        self._get_model_count_rt = get_model_count_rt
        self._get_model_name = get_model_name
        self._has_rt_data = has_rt_data
        get_topic_module().start_subscriber(robot_ip)

    def wait_ready(self) -> None:
        deadline = time.time() + self.timeout_sec
        while time.time() < deadline:
            if self._has_rt_data():
                count = self._get_model_count_rt()
                if count >= 1:
                    print(f"[STATE] sdk rt ready model_count={count} model0={self._get_model_name(0)}", flush=True)
                    return
            time.sleep(0.1)
        raise CycleError(f"no realtime robot state from {self.robot_ip}:19091")

    def current_raw(self) -> list[float]:
        if not self._has_rt_data():
            raise CycleError("realtime robot state unavailable")
        raw = self._get_current_robottarget(0)
        if not isinstance(raw, list) or len(raw) < 7:
            raise CycleError(f"invalid realtime robottarget: {raw}")
        return [float(x) for x in raw[:7]]


def require_file(path: Path, name: str) -> None:
    if not path.exists():
        raise CycleError(f"missing {name}: {path}")


def load_pose(path: Path) -> list[float]:
    require_file(path, "saved pose")
    raw = json.loads(path.read_text()).get("robottarget")
    if not isinstance(raw, list) or len(raw) < 7:
        raise CycleError(f"invalid saved pose: {path}")
    return [float(x) for x in raw]


def wait_gateway_ready(args) -> None:
    try:
        print("[GW] resume", Gateway.one_shot({"cmd": "resume_gateway"}, args.gateway_host, args.gateway_port, 5), flush=True)
    except Exception as exc:
        print(f"[GW] resume nonfatal: {exc}", flush=True)
    deadline = time.time() + args.gateway_ready_timeout
    last = None
    while time.time() < deadline:
        last = Gateway.one_shot({"cmd": "state"}, args.gateway_host, args.gateway_port, 5)
        print("[GW] state", last, flush=True)
        if last.get("state") == "READY" and last.get("connected") and last.get("arm_enabled"):
            return
        time.sleep(1)
    raise CycleError(f"gateway not ready; last={last}")


def fmt(vals: list[float]) -> str:
    return ",".join(f"{float(v):.8f}" for v in vals)


def quat_normalize(q: list[float]) -> list[float]:
    norm = math.sqrt(sum(float(v) * float(v) for v in q))
    if norm < 1e-9:
        raise CycleError(f"invalid quaternion: {q}")
    return [float(v) / norm for v in q]


def quat_conj(q: list[float]) -> list[float]:
    return [-q[0], -q[1], -q[2], q[3]]


def quat_mul(a: list[float], b: list[float]) -> list[float]:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return [
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ]


def quat_error_vector(target: list[float], current: list[float]) -> tuple[list[float], float]:
    qt = quat_normalize(target[:4])
    qc = quat_normalize(current[:4])
    if sum(qt[i] * qc[i] for i in range(4)) < 0:
        qt = [-v for v in qt]
    qe = quat_mul(qt, quat_conj(qc))
    qe = quat_normalize(qe)
    if qe[3] < 0:
        qe = [-v for v in qe]
    vec = qe[:3]
    vnorm = math.sqrt(sum(v * v for v in vec))
    angle = 2.0 * math.atan2(vnorm, max(-1.0, min(1.0, qe[3])))
    if angle > math.pi:
        angle -= 2.0 * math.pi
    if abs(angle) < 1e-6 or vnorm < 1e-9:
        return [0.0, 0.0, 0.0], 0.0
    axis = [v / vnorm for v in vec]
    return [axis[0] * angle, axis[1] * angle, axis[2] * angle], abs(angle)


def move_pose(
    gw: Gateway,
    state: RobotState,
    target: list[float],
    label: str,
    tol: Point,
    timeout_sec: float,
    max_speed: float,
    max_angular_speed: float,
    orient_tol: float,
) -> None:
    if len(target) != 7:
        raise CycleError(f"invalid target pose for {label}: {target}")
    goal = Point(*target[:3])
    max_speed = max(0.001, float(max_speed))
    max_angular_speed = max(0.01, float(max_angular_speed))
    orient_tol = max(0.005, float(orient_tol))
    print(f"[MOVE] {label} target={[round(x, 5) for x in target]} max_speed_mps={max_speed} max_w_rps={max_angular_speed}", flush=True)
    deadline = time.time() + timeout_sec
    last_print = 0
    try:
        while time.time() < deadline:
            cur_raw = state.current_raw()
            cur = Point(*cur_raw[:3])
            err = [goal.x - cur.x, goal.y - cur.y, goal.z - cur.z]
            dist = math.sqrt(sum(e * e for e in err))
            rot_vec, rot_angle = quat_error_vector(target[3:7], cur_raw[3:7])
            if time.time() - last_print > 1.5:
                print(f"[MOVE] {label} pos={[round(x,4) for x in cur.as_list()]} err_mm={[round(e*1000,1) for e in err]} dist_mm={dist*1000:.1f} rot_deg={math.degrees(rot_angle):.1f}", flush=True)
                last_print = time.time()
            if abs(err[0]) <= tol.x and abs(err[1]) <= tol.y and abs(err[2]) <= tol.z:
                gw.speedl_stop()
                print(f"[MOVE] {label} done", flush=True)
                return
            # Proportional velocity command in base frame. This avoids stale
            # pose-file feedback and only sends a command for the current error.
            speed = min(max_speed, max(0.0015, dist * 0.30))
            vel_xyz = [e / dist * speed for e in err] if dist > 1e-6 else [0.0, 0.0, 0.0]
            # Do not command rx/ry/rz with SpeedL on this 5-axis arm. A live
            # test showed that even a small Cartesian angular velocity can map
            # to a large wrist joint flip. Horizontal grasp/place orientation
            # is therefore enforced by a guard against a taught reference pose,
            # not by online angular velocity control.
            omega = [0.0, 0.0, 0.0]
            reply = gw.speedl(vel_xyz + omega, coordinate=0, deadman=True)
            if not reply.get("ok"):
                raise CycleError(f"SpeedL rejected in {label}: {reply}")
            time.sleep(0.10)
    finally:
        gw.speedl_stop()
    raise CycleError(f"move timeout: {label}; current={state.current_raw()[:3]}; goal={goal.as_list()}")


def pose_with_xyz(reference_pose: list[float], point: Point) -> list[float]:
    if len(reference_pose) < 7:
        raise CycleError(f"invalid reference pose: {reference_pose}")
    return [point.x, point.y, point.z] + [float(x) for x in reference_pose[3:7]]


def assert_horizontal_orientation(state: RobotState, horizontal_pose_ref: list[float], label: str, tolerance_rad: float) -> None:
    cur_raw = state.current_raw()
    _, angle = quat_error_vector(horizontal_pose_ref[3:7], cur_raw[3:7])
    print(f"[ORIENT] {label} horizontal_err_deg={math.degrees(angle):.2f} tol_deg={math.degrees(tolerance_rad):.2f}", flush=True)
    if angle > tolerance_rad:
        raise CycleError(
            f"{label}: current TCP orientation is not horizontal enough; "
            f"err_deg={math.degrees(angle):.2f}, tol_deg={math.degrees(tolerance_rad):.2f}. "
            "Refuse to rotate wrist by SpeedL. Teach a safe horizontal waypoint or use a validated controller pose move."
        )


def gripper(action: str, force: int, speed: int) -> None:
    reply = run(["python3", str(GRIPPER_SCRIPT), "--action", action, "--force", str(force), "--speed", str(speed)], timeout=20, check=False)
    print(f"[GRIPPER] {action} rc={reply.returncode} {reply.stdout.strip()}", flush=True)
    if reply.returncode != 0:
        raise CycleError(f"gripper {action} failed")


def parse_point(text: str) -> Point | None:
    vals = {}
    for line in text.splitlines():
        line = line.strip()
        if line.startswith("x:"):
            vals["x"] = float(line.split(":", 1)[1])
        elif line.startswith("y:"):
            vals["y"] = float(line.split(":", 1)[1])
        elif line.startswith("z:"):
            vals["z"] = float(line.split(":", 1)[1])
    if all(k in vals for k in ("x", "y", "z")):
        return Point(vals["x"], vals["y"], vals["z"])
    return None


def get_bottle_target(timeout_sec: float) -> Point:
    deadline = time.time() + timeout_sec
    while time.time() < deadline:
        out = bash("source /opt/ros/humble/setup.bash; timeout 5 ros2 topic echo /bottle_target/base_point --once", timeout=8, check=False).stdout
        pt = parse_point(out)
        if pt:
            print("[DETECT] target", pt.as_list(), flush=True)
            return pt
        time.sleep(0.5)
    print("[DETECT] /bottle_target/base_point unavailable, falling back to direct YOLO+depth+TF", flush=True)
    detections = all_bottle_coords()
    if detections:
        best = max(detections, key=lambda d: d.score)
        print("[DETECT] fallback target", best.point.as_list(), "score", best.score, flush=True)
        return best.point
    raise CycleError("no bottle target detected")


def all_bottle_coords() -> list[BottleDetection]:
    helper = r'''
import json, time, numpy as np, rclpy, tf2_geometry_msgs
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener
class N(Node):
    def __init__(self):
        super().__init__("coords_once")
        self.bridge=CvBridge(); self.depth=None; self.info=None; self.det=None
        self.tf=Buffer(); self.tfl=TransformListener(self.tf,self)
        self.create_subscription(Image,"/camera/camera/aligned_depth_to_color/image_raw",self.depth_cb,10)
        self.create_subscription(CameraInfo,"/camera/camera/color/camera_info",self.info_cb,10)
        self.create_subscription(Detection2DArray,"/yolov8/detections",self.det_cb,10)
    def depth_cb(self,msg):
        img=self.bridge.imgmsg_to_cv2(msg,desired_encoding="passthrough")
        self.depth=img.astype(np.float32)*(0.001 if msg.encoding=="16UC1" else 1.0)
    def info_cb(self,msg): self.info=msg
    def det_cb(self,msg): self.det=msg
def center(b):
    c=b.center
    return (float(c.position.x), float(c.position.y)) if hasattr(c,"position") else (float(c.x), float(c.y))
rclpy.init(); n=N(); start=time.time()
while time.time()-start<5 and (n.depth is None or n.info is None or n.det is None):
    rclpy.spin_once(n,timeout_sec=0.1)
for _ in range(10): rclpy.spin_once(n,timeout_sec=0.05)
rows=[]
if n.depth is not None and n.info is not None and n.det is not None:
    h,w=n.depth.shape[:2]; fx,fy,cx,cy=n.info.k[0],n.info.k[4],n.info.k[2],n.info.k[5]
    for d in n.det.detections:
        if not d.results: continue
        hypo=d.results[0].hypothesis if hasattr(d.results[0],"hypothesis") else d.results[0]
        if str(hypo.class_id)!="39": continue
        u,v=center(d.bbox)
        if 0<=u<=1.5 and 0<=v<=1.5: u*=w; v*=h
        ui,vi=int(round(u)),int(round(v))
        if not(2<=ui<w-2 and 2<=vi<h-2): continue
        patch=n.depth[vi-2:vi+3,ui-2:ui+3]; valid=patch[(patch>=0.15)&(patch<=1.5)]
        if valid.size<5: continue
        z=float(np.median(valid)); p=PointStamped(); p.header.frame_id=n.det.header.frame_id; p.header.stamp=Time().to_msg()
        p.point.x=(u-cx)*z/fx; p.point.y=(v-cy)*z/fy; p.point.z=z
        try:
            bp=n.tf.transform(p,"base_link",timeout=Duration(seconds=0.5))
            rows.append({"score":float(hypo.score),"x":bp.point.x,"y":bp.point.y,"z":bp.point.z})
        except Exception: pass
print(json.dumps(rows))
n.destroy_node(); rclpy.shutdown()
'''
    out = bash("source /opt/ros/humble/setup.bash; python3 - <<'PY'\n" + helper + "\nPY", timeout=15, check=False).stdout
    try:
        rows = json.loads(out.strip().splitlines()[-1])
    except Exception as exc:
        raise CycleError(f"cannot parse detection coordinates: {exc}; output={out[:500]}")
    detections = [BottleDetection(float(r["score"]), Point(float(r["x"]), float(r["y"]), float(r["z"]))) for r in rows]
    print("[DETECT] all", [(round(d.score, 3), [round(v, 4) for v in d.point.as_list()]) for d in detections], flush=True)
    return detections


def choose_place(nominal: Point, detections: list[BottleDetection], occupied_radius: float, side_offset: float) -> tuple[Point, bool]:
    if not detections:
        return nominal, False
    nearest = min(detections, key=lambda d: d.point.xy_distance(nominal))
    occupied = nearest.point.xy_distance(nominal) <= occupied_radius
    if not occupied:
        return nominal, False
    dx, dy = nominal.x - nearest.point.x, nominal.y - nearest.point.y
    norm = math.hypot(dx, dy) or 1.0
    return Point(nominal.x + dx / norm * side_offset, nominal.y + dy / norm * side_offset, nominal.z), True


def run_cycle(args) -> None:
    for path, name in [(GRIPPER_SCRIPT, "gripper script"), (args.grasp_photo_pose, "grasp photo pose"), (args.place_pose, "place pose"), (args.place_photo_pose, "place photo pose")]:
        require_file(path, name)

    started: list[subprocess.Popen] = []
    if args.start_camera_stack:
        started = start_camera_stack(args)

    wait_gateway_ready(args)
    state = RobotState(args.robot_ip, args.state_ready_timeout)
    state.wait_ready()
    gw = Gateway(args.gateway_host, args.gateway_port, args.owner, args.seq_start)
    gw.connect()
    try:
        owner = gw.set_owner()
        print("[GW] owner", owner, flush=True)
        if not owner.get("ok"):
            raise CycleError(f"cannot get owner: {owner}")

        grasp_photo_pose = load_pose(args.grasp_photo_pose)
        place_photo_pose = load_pose(args.place_photo_pose)
        place_pose = load_pose(args.place_pose)
        horizontal_pose_ref = place_pose

        move_pose(gw, state, grasp_photo_pose, "move_to_grasp_photo", Point(0.018, 0.018, 0.018), args.long_move_timeout, args.max_speed, args.max_angular_speed, args.orient_tolerance_rad)
        bottle = get_bottle_target(args.detect_timeout)
        cur_raw = state.current_raw()
        cur = Point(*cur_raw[:3])
        above = Point(bottle.x, bottle.y, max(cur.z, bottle.z + args.approach_z_offset))
        grip = Point(bottle.x, bottle.y, bottle.z + args.grasp_z_offset)
        lift = Point(bottle.x, bottle.y, max(above.z, bottle.z + args.lift_z_offset))

        gripper("open", args.gripper_force, args.gripper_speed)
        move_pose(gw, state, pose_with_xyz(horizontal_pose_ref, above), "move_above_bottle_horizontal", Point(0.008, 0.008, 0.008), args.move_timeout, args.max_speed, args.max_angular_speed, args.orient_tolerance_rad)
        if args.enforce_horizontal_actions:
            assert_horizontal_orientation(state, horizontal_pose_ref, "before_grasp_descent", args.horizontal_tolerance_rad)
        move_pose(gw, state, pose_with_xyz(horizontal_pose_ref, grip), "descend_to_grasp_horizontal", Point(0.008, 0.008, 0.008), args.move_timeout, args.max_speed, args.max_angular_speed, args.orient_tolerance_rad)
        if args.enforce_horizontal_actions:
            assert_horizontal_orientation(state, horizontal_pose_ref, "before_gripper_close", args.horizontal_tolerance_rad)
        time.sleep(args.grip_settle_sec)
        gripper("close", args.gripper_force, args.gripper_speed)
        time.sleep(args.grip_settle_sec)
        move_pose(gw, state, pose_with_xyz(horizontal_pose_ref, lift), "lift_after_grasp_horizontal", Point(0.012, 0.012, 0.010), args.move_timeout, args.max_speed, args.max_angular_speed, args.orient_tolerance_rad)

        move_pose(gw, state, place_photo_pose, "move_to_place_photo", Point(0.018, 0.018, 0.018), args.long_move_timeout, args.max_speed, args.max_angular_speed, args.orient_tolerance_rad)
        nominal = Point(*place_pose[:3])
        detections = all_bottle_coords()
        place, occupied = choose_place(nominal, detections, args.place_occupied_radius, args.place_side_offset)
        RUNTIME_PLACE.write_text(json.dumps({"saved_at": time.time(), "occupied": occupied, "nominal_place_xyz": nominal.as_list(), "target_place_xyz": place.as_list()}, ensure_ascii=False, indent=2))
        print("[PLACE] occupied", occupied, "target", place.as_list(), flush=True)

        cur2_raw = state.current_raw()
        cur2 = Point(*cur2_raw[:3])
        safe_z = max(cur2.z, place.z + args.place_safe_z_offset)
        move_pose(gw, state, pose_with_xyz(horizontal_pose_ref, Point(cur2.x, cur2.y, safe_z)), "raise_before_place_horizontal", Point(0.012, 0.012, 0.012), args.move_timeout, args.max_speed, args.max_angular_speed, args.orient_tolerance_rad)
        move_pose(gw, state, pose_with_xyz(horizontal_pose_ref, Point(place.x, place.y, safe_z)), "move_above_place_horizontal", Point(0.018, 0.018, 0.018), args.long_move_timeout, args.max_speed, args.max_angular_speed, args.orient_tolerance_rad)
        if args.enforce_horizontal_actions:
            assert_horizontal_orientation(state, horizontal_pose_ref, "before_place_descent", args.horizontal_tolerance_rad)
        move_pose(gw, state, pose_with_xyz(horizontal_pose_ref, place), "descend_to_place_horizontal", Point(0.010, 0.010, 0.010), args.move_timeout, args.max_speed, args.max_angular_speed, args.orient_tolerance_rad)
        if args.enforce_horizontal_actions:
            assert_horizontal_orientation(state, horizontal_pose_ref, "before_gripper_open", args.horizontal_tolerance_rad)
        time.sleep(0.5)
        gripper("open", args.gripper_force, args.gripper_speed)
        move_pose(gw, state, pose_with_xyz(horizontal_pose_ref, Point(place.x, place.y, safe_z)), "lift_after_release_horizontal", Point(0.015, 0.015, 0.015), args.move_timeout, args.max_speed, args.max_angular_speed, args.orient_tolerance_rad)
        move_pose(gw, state, grasp_photo_pose, "return_to_grasp_photo", Point(0.018, 0.018, 0.018), args.long_move_timeout, args.max_speed, args.max_angular_speed, args.orient_tolerance_rad)
    finally:
        try:
            gw.speedl_stop()
        except Exception:
            pass
        try:
            print("[GW] release", gw.release(), flush=True)
        except Exception as exc:
            print("[GW] release failed", exc, flush=True)
        gw.close()
        if args.stop_camera_after:
            for proc in reversed(started):
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                except Exception:
                    pass


def parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser()
    p.add_argument("--start-camera-stack", action="store_true")
    p.add_argument("--stop-camera-after", action="store_true")
    p.add_argument("--pose-json", type=Path, default=POSE_JSON)
    p.add_argument("--grasp-photo-pose", type=Path, default=GRASP_PHOTO)
    p.add_argument("--place-pose", type=Path, default=PLACE_POSE)
    p.add_argument("--place-photo-pose", type=Path, default=PLACE_PHOTO)
    p.add_argument("--gateway-host", default="127.0.0.1")
    p.add_argument("--gateway-port", type=int, default=6200)
    p.add_argument("--robot-ip", default="192.168.11.11")
    p.add_argument("--owner", default="AUTO_TASK")
    p.add_argument("--seq-start", type=int, default=2_000_000_000_000)
    p.add_argument("--max-speed", type=float, default=0.015)
    p.add_argument("--max-angular-speed", type=float, default=0.12)
    p.add_argument("--orient-tolerance-rad", type=float, default=0.08)
    p.add_argument("--horizontal-tolerance-rad", type=float, default=0.12)
    p.add_argument("--no-enforce-horizontal-actions", dest="enforce_horizontal_actions", action="store_false")
    p.set_defaults(enforce_horizontal_actions=True)
    p.add_argument("--move-timeout", type=float, default=100.0)
    p.add_argument("--long-move-timeout", type=float, default=180.0)
    p.add_argument("--gateway-ready-timeout", type=float, default=70.0)
    p.add_argument("--state-ready-timeout", type=float, default=8.0)
    p.add_argument("--pose-max-age-sec", type=float, default=1.0)
    p.add_argument("--detect-timeout", type=float, default=12.0)
    p.add_argument("--detect-conf", type=float, default=0.35)
    p.add_argument("--yolo-device", default="cpu")
    p.add_argument("--yolo-model", type=Path, default=YOLO_MODEL)
    p.add_argument("--stack-warmup-sec", type=float, default=8.0)
    p.add_argument("--approach-z-offset", type=float, default=0.13)
    p.add_argument("--grasp-z-offset", type=float, default=0.0)
    p.add_argument("--lift-z-offset", type=float, default=0.15)
    p.add_argument("--place-safe-z-offset", type=float, default=0.12)
    p.add_argument("--place-occupied-radius", type=float, default=0.13)
    p.add_argument("--place-side-offset", type=float, default=0.12)
    p.add_argument("--gripper-force", type=int, default=60)
    p.add_argument("--gripper-speed", type=int, default=60)
    p.add_argument("--grip-settle-sec", type=float, default=0.8)
    return p


def main() -> int:
    ensure_sdk_process_env()
    args = parser().parse_args()
    try:
        run_cycle(args)
        print("[DONE] bottle cycle completed", flush=True)
        return 0
    except Exception as exc:
        print("[ERROR]", exc, flush=True)
        try:
            Gateway.one_shot({"cmd": "state"}, args.gateway_host, args.gateway_port, 3)
        except Exception:
            pass
        return 1


if __name__ == "__main__":
    raise SystemExit(main())

