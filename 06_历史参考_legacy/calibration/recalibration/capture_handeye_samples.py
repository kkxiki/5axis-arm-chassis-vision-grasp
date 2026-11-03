#!/usr/bin/env python3
"""Capture synchronized chessboard images and read-only robot poses."""

from __future__ import annotations

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
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Trigger

from robot_state_bridge.msg import SystemRtState

from handeye_common import (
    find_board,
    pose_xyzw_to_transform,
    solve_board_pose,
)


def stamp_to_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def infer_translation_unit(raw_xyz: np.ndarray, requested: str) -> tuple[str, float]:
    if requested == "m":
        return "m", 1.0
    if requested == "mm":
        return "mm", 0.001
    # Typical controller values are either around 0.1-2.0 m or 100-2000 mm.
    if float(np.max(np.abs(raw_xyz))) > 10.0:
        return "mm", 0.001
    return "m", 1.0


class HandEyeCaptureNode(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("handeye_sample_capture")
        self.args = args
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.latest_image = None
        self.latest_image_msg = None
        self.latest_image_rx = 0.0
        self.latest_info = None
        self.latest_state = None
        self.latest_state_rx = 0.0
        self.last_board_found = None
        self.output_dir = Path(args.output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.manifest_path = self.output_dir / "samples.json"

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.create_subscription(Image, args.image_topic, self.on_image, sensor_qos)
        self.create_subscription(
            CameraInfo, args.camera_info_topic, self.on_camera_info, sensor_qos
        )
        self.create_subscription(
            SystemRtState, args.robot_state_topic, self.on_robot_state, 10
        )
        self.debug_pub = self.create_publisher(Image, args.debug_topic, sensor_qos)
        self.capture_service = self.create_service(
            Trigger, args.capture_service, self.on_capture
        )
        self.create_timer(1.0, self.publish_board_preview)
        self.get_logger().info(
            f"Capture ready: service={args.capture_service}, output={self.output_dir}"
        )
        self.get_logger().info(
            f"Board inner corners={args.cols}x{args.rows}, "
            f"square={args.square_mm:.3f} mm"
        )

    def on_image(self, message: Image):
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            with self.lock:
                self.latest_image = image.copy()
                self.latest_image_msg = message
                self.latest_image_rx = time.monotonic()
        except Exception as exc:
            self.get_logger().error(f"Image conversion failed: {exc}")

    def on_camera_info(self, message: CameraInfo):
        with self.lock:
            self.latest_info = message

    def on_robot_state(self, message: SystemRtState):
        with self.lock:
            self.latest_state = message
            self.latest_state_rx = time.monotonic()

    def publish_board_preview(self):
        with self.lock:
            image = None if self.latest_image is None else self.latest_image.copy()
            source_msg = self.latest_image_msg
        if image is None or source_msg is None:
            return
        found, corners = find_board(
            image, self.args.cols, self.args.rows, accurate=False
        )
        if found:
            cv2.drawChessboardCorners(
                image, (self.args.cols, self.args.rows), corners, True
            )
        label = "BOARD FOUND" if found else "BOARD NOT FOUND"
        color = (0, 200, 0) if found else (0, 0, 255)
        cv2.putText(
            image, label, (20, 45), cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 2
        )
        debug_msg = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
        debug_msg.header = source_msg.header
        self.debug_pub.publish(debug_msg)
        if found != self.last_board_found:
            self.get_logger().info(label)
            self.last_board_found = found

    def on_capture(self, request, response):
        del request
        try:
            sample_id, reprojection_rms = self.capture_one()
            response.success = True
            response.message = (
                f"captured sample_{sample_id:03d}; "
                f"reprojection_rms={reprojection_rms:.3f}px"
            )
        except Exception as exc:
            response.success = False
            response.message = str(exc)
            self.get_logger().warning(f"Capture rejected: {exc}")
        return response

    def capture_one(self) -> tuple[int, float]:
        now = time.monotonic()
        with self.lock:
            image = None if self.latest_image is None else self.latest_image.copy()
            image_msg = self.latest_image_msg
            image_rx = self.latest_image_rx
            info = self.latest_info
            state = self.latest_state
            state_rx = self.latest_state_rx

        if image is None or image_msg is None or info is None:
            raise RuntimeError("camera image/CameraInfo has not arrived")
        if state is None:
            raise RuntimeError("robot state has not arrived")
        if now - image_rx > self.args.max_age_sec:
            raise RuntimeError("latest camera image is stale")
        if now - state_rx > self.args.max_age_sec:
            raise RuntimeError("latest robot state is stale")
        if state.robot_status != "RUNNING":
            raise RuntimeError(f"robot state is not valid: {state.robot_status}")

        raw_pose = np.asarray(state.robottarget, dtype=np.float64)
        if raw_pose.shape != (7,) or not np.isfinite(raw_pose).all():
            raise RuntimeError(
                f"robottarget must contain 7 finite values, got {raw_pose.tolist()}"
            )
        unit, scale = infer_translation_unit(
            raw_pose[:3], self.args.pose_translation_unit
        )
        base_t_gripper = pose_xyzw_to_transform(raw_pose, scale)

        found, corners = find_board(
            image, self.args.cols, self.args.rows, accurate=True
        )
        if not found:
            raise RuntimeError(
                f"{self.args.cols}x{self.args.rows} inner-corner board not detected"
            )

        camera_matrix = np.asarray(info.k, dtype=np.float64).reshape(3, 3)
        distortion = np.asarray(info.d, dtype=np.float64)
        camera_t_target, reprojection_rms = solve_board_pose(
            corners,
            self.args.cols,
            self.args.rows,
            self.args.square_mm / 1000.0,
            camera_matrix,
            distortion,
        )

        manifest = self.load_manifest()
        sample_id = 0
        if manifest["samples"]:
            sample_id = max(int(item["id"]) for item in manifest["samples"]) + 1
        image_name = f"sample_{sample_id:03d}.png"
        debug_name = f"sample_{sample_id:03d}_corners.png"
        debug = image.copy()
        cv2.drawChessboardCorners(
            debug, (self.args.cols, self.args.rows), corners, True
        )
        if not cv2.imwrite(str(self.output_dir / image_name), image):
            raise RuntimeError("failed to write sample image")
        if not cv2.imwrite(str(self.output_dir / debug_name), debug):
            raise RuntimeError("failed to write debug image")

        sample = {
            "id": sample_id,
            "image": image_name,
            "debug_image": debug_name,
            "image_ros_stamp": stamp_to_seconds(image_msg.header.stamp),
            "robot_state_ros_stamp": stamp_to_seconds(state.header.stamp),
            "receive_time_delta_sec": abs(image_rx - state_rx),
            "model_index": int(state.model_index),
            "model_name": state.model_name,
            "robot_pose": {
                "source_field": "robottarget",
                "order": "x,y,z,qx,qy,qz,qw",
                "raw": raw_pose.tolist(),
                "translation_unit": unit,
                "base_T_gripper_m": base_t_gripper.tolist(),
            },
            "camera_T_target_m": camera_t_target.tolist(),
            "reprojection_rms_px": reprojection_rms,
        }
        manifest["samples"].append(sample)
        self.write_manifest(manifest)
        self.get_logger().info(
            f"Saved sample_{sample_id:03d}: model={state.model_index}/"
            f"{state.model_name}, raw_pose_unit={unit}, "
            f"reprojection={reprojection_rms:.3f}px"
        )
        return sample_id, reprojection_rms

    def load_manifest(self) -> dict:
        board = {
            "inner_corners_cols": self.args.cols,
            "inner_corners_rows": self.args.rows,
            "square_size_mm": self.args.square_mm,
        }
        info = self.latest_info
        camera = {
            "image_width": int(info.width),
            "image_height": int(info.height),
            "distortion_model": info.distortion_model,
            "k": list(info.k),
            "d": list(info.d),
        }
        if not self.manifest_path.exists():
            return {
                "schema_version": 1,
                "calibration_mode": "eye_to_hand",
                "board": board,
                "camera": camera,
                "image_topic": self.args.image_topic,
                "camera_info_topic": self.args.camera_info_topic,
                "robot_state_topic": self.args.robot_state_topic,
                "samples": [],
            }
        with self.manifest_path.open("r", encoding="utf-8") as stream:
            manifest = json.load(stream)
        if manifest.get("board") != board:
            raise RuntimeError("existing manifest uses a different board geometry")
        old_k = np.asarray(manifest["camera"]["k"], dtype=np.float64)
        if not np.allclose(old_k, np.asarray(camera["k"]), rtol=1e-5, atol=1e-5):
            raise RuntimeError("CameraInfo intrinsics changed since the first sample")
        return manifest

    def write_manifest(self, manifest: dict):
        temp_path = self.manifest_path.with_suffix(".json.tmp")
        with temp_path.open("w", encoding="utf-8") as stream:
            json.dump(manifest, stream, ensure_ascii=False, indent=2)
            stream.write("\n")
        os.replace(temp_path, self.manifest_path)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--image-topic", default="/gemini335le/color/image_raw"
    )
    parser.add_argument(
        "--camera-info-topic", default="/gemini335le/color/camera_info"
    )
    parser.add_argument("--robot-state-topic", default="/robot/right_state")
    parser.add_argument("--debug-topic", default="/handeye/debug_image")
    parser.add_argument("--capture-service", default="/handeye/capture")
    parser.add_argument(
        "--output-dir", default="/root/XuK/calibration/recalibration/right"
    )
    parser.add_argument("--cols", type=int, default=8)
    parser.add_argument("--rows", type=int, default=6)
    parser.add_argument("--square-mm", type=float, default=29.0)
    parser.add_argument(
        "--pose-translation-unit", choices=("auto", "m", "mm"), default="auto"
    )
    parser.add_argument("--max-age-sec", type=float, default=1.0)
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = HandEyeCaptureNode(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

