#!/usr/bin/env python3
"""One-button hand-eye sample capture for the current five-axis arm."""

import argparse
import json
import os
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Trigger

from handeye_common import find_board, pose_xyzw_to_transform, solve_board_pose


class Capture(Node):
    def __init__(self, args):
        super().__init__("right_handeye_capture")
        self.args = args
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.image = None
        self.info = None
        self.image_stamp = None
        self.image_rx = 0.0
        self.image_wall_rx = 0.0
        self.out = Path(args.output_dir)
        self.out.mkdir(parents=True, exist_ok=True)
        self.manifest = self.out / "samples.json"

        self.create_subscription(Image, args.image_topic, self.on_image, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, args.info_topic, self.on_info, qos_profile_sensor_data)
        self.create_service(Trigger, args.service, self.on_capture)
        self.get_logger().info(
            f"READY service={args.service}, board={args.cols}x{args.rows}, "
            f"square={args.square_mm}mm, output={self.out}"
        )

    def on_image(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        with self.lock:
            self.image = image.copy()
            self.image_stamp = msg.header.stamp
            self.image_rx = time.monotonic()
            self.image_wall_rx = time.time()

    def on_info(self, msg):
        with self.lock:
            self.info = msg

    def on_capture(self, _request, response):
        try:
            sid, rms = self.capture_one()
            response.success = True
            response.message = f"captured sample_{sid:03d}; reprojection_rms={rms:.3f}px"
        except Exception as exc:
            response.success = False
            response.message = str(exc)
            self.get_logger().warning(f"capture rejected: {exc}")
        return response

    def capture_one(self):
        with self.lock:
            image = None if self.image is None else self.image.copy()
            info = self.info
            stamp = self.image_stamp
            age = time.monotonic() - self.image_rx
            image_wall_rx = self.image_wall_rx
        if image is None or info is None or age > 1.0:
            raise RuntimeError("D435i image/CameraInfo unavailable or stale")
        pose_path = Path(self.args.pose_file)
        if not pose_path.exists():
            raise RuntimeError("five-axis controller state unavailable")
        pose_data = json.loads(pose_path.read_text(encoding="utf-8"))
        pose_time = float(pose_data["time"])
        if time.time() - pose_time > 0.2:
            raise RuntimeError("five-axis controller state is stale")
        sync_delta = abs(pose_time - image_wall_rx)
        if sync_delta > 0.12:
            raise RuntimeError(f"image/robot timestamp mismatch: {sync_delta:.3f}s")
        linear_speed = float(pose_data.get("linear_speed_mps", 999.0))
        angular_speed = float(pose_data.get("angular_speed_radps", 999.0))
        stable_duration = float(pose_data.get("stable_duration_sec", 0.0))
        if linear_speed > 0.002 or angular_speed > np.deg2rad(0.5):
            raise RuntimeError(
                f"arm is not stationary: {linear_speed*1000:.2f}mm/s, "
                f"{np.rad2deg(angular_speed):.2f}deg/s"
            )
        if stable_duration < 0.8:
            raise RuntimeError(f"arm has been stationary for only {stable_duration:.2f}s; need 0.80s")
        raw = np.asarray(pose_data["robottarget"], dtype=np.float64)
        if raw.shape != (7,) or not np.isfinite(raw).all():
            raise RuntimeError(f"invalid robot pose: {raw.tolist()}")
        found, corners = find_board(image, self.args.cols, self.args.rows, accurate=True)
        if not found:
            raise RuntimeError(f"{self.args.cols}x{self.args.rows} board not fully detected")
        k = np.asarray(info.k, dtype=np.float64).reshape(3, 3)
        d = np.asarray(info.d, dtype=np.float64)
        camera_t_target, rms = solve_board_pose(
            corners, self.args.cols, self.args.rows,
            self.args.square_mm / 1000.0, k, d
        )
        data = self.load_manifest(info)
        for previous in data["samples"]:
            old = np.asarray(previous["robot_pose"]["raw"], dtype=np.float64)
            distance_mm = float(np.linalg.norm(raw[:3] - old[:3]) * 1000.0)
            quat_dot = float(abs(np.dot(raw[3:7], old[3:7])))
            quat_dot = min(1.0, max(-1.0, quat_dot))
            angle_deg = float(np.rad2deg(2.0 * np.arccos(quat_dot)))
            if distance_mm < 3.0 and angle_deg < 1.0:
                raise RuntimeError(
                    f"duplicate pose: only {distance_mm:.2f}mm/{angle_deg:.2f}deg from sample_{int(previous['id']):03d}"
                )
        sid = max((int(s["id"]) for s in data["samples"]), default=-1) + 1
        image_name = f"sample_{sid:03d}.png"
        debug_name = f"sample_{sid:03d}_corners.png"
        debug = image.copy()
        cv2.drawChessboardCorners(debug, (self.args.cols, self.args.rows), corners, True)
        if not cv2.imwrite(str(self.out / image_name), image):
            raise RuntimeError("cannot save image")
        if not cv2.imwrite(str(self.out / debug_name), debug):
            raise RuntimeError("cannot save debug image")
        base_t_gripper = pose_xyzw_to_transform(raw, 1.0)
        data["samples"].append({
            "id": sid,
            "image": image_name,
            "debug_image": debug_name,
            "image_ros_stamp": float(stamp.sec) + float(stamp.nanosec) * 1e-9,
            "robot_state_wall_stamp": pose_time,
            "receive_time_delta_sec": sync_delta,
            "robot_linear_speed_mps": linear_speed,
            "robot_angular_speed_radps": angular_speed,
            "robot_stable_duration_sec": stable_duration,
            "model_index": 0,
            "model_name": pose_data["model_name"],
            "robot_pose": {
                "source_field": "official_sdk.get_current_robottarget(0)",
                "order": "x,y,z,qx,qy,qz,qw",
                "raw": raw.tolist(),
                "translation_unit": "m",
                "base_T_gripper_m": base_t_gripper.tolist(),
            },
            "camera_T_target_m": camera_t_target.tolist(),
            "reprojection_rms_px": float(rms),
        })
        tmp = self.manifest.with_suffix(".json.tmp")
        tmp.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        os.replace(tmp, self.manifest)
        self.get_logger().info(f"SAVED sample_{sid:03d}, rms={rms:.3f}px, pose={raw.tolist()}")
        return sid, rms

    def load_manifest(self, info):
        board = {"inner_corners_cols": self.args.cols,
                 "inner_corners_rows": self.args.rows,
                 "square_size_mm": self.args.square_mm}
        camera = {"image_width": int(info.width), "image_height": int(info.height),
                  "distortion_model": info.distortion_model,
                  "k": list(info.k), "d": list(info.d)}
        if not self.manifest.exists():
            return {"schema_version": 1, "calibration_mode": "eye_in_hand",
                    "board": board, "camera": camera,
                    "image_topic": self.args.image_topic,
                    "camera_info_topic": self.args.info_topic, "samples": []}
        data = json.loads(self.manifest.read_text(encoding="utf-8"))
        if data.get("board") != board:
            raise RuntimeError("existing samples use another board geometry")
        return data


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--robot-ip", default="192.168.11.11")
    p.add_argument("--pose-file", default="/tmp/five_axis_pose.json")
    p.add_argument("--image-topic", default="/camera/camera/color/image_raw")
    p.add_argument("--info-topic", default="/camera/camera/color/camera_info")
    p.add_argument("--service", default="/right_handeye/capture")
    p.add_argument("--output-dir", default="/home/nvidia/calibration/right")
    p.add_argument("--cols", type=int, default=5)
    p.add_argument("--rows", type=int, default=7)
    p.add_argument("--square-mm", type=float, default=30.5)
    args = p.parse_args()
    rclpy.init()
    node = Capture(args)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
