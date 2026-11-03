#!/usr/bin/env python3
"""ROS2-to-HTTP bridge for the Ethernet Gemini 335Le front camera.

The Orbbec driver and the existing YOLO node stay responsible for camera I/O
and inference. This process synchronizes their latest colour/depth/detection
messages, exposes V4's /health and /detect API, and provides an MJPEG page for
the operator's Windows browser.
"""
from __future__ import annotations

import argparse
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from vision_msgs.msg import Detection2DArray

from v4_robot.front_geometry import (
    bearing_from_pixel,
    normalized_bbox_to_pixels,
    robust_depth_m,
)


COCO_CLASS_IDS = {"person": "0", "bottle": "39", "cup": "41"}


class FrontVisionNode(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("v4_front_vision_http")
        self.args = args
        self.bridge = CvBridge()
        self.lock = threading.RLock()
        self.condition = threading.Condition(self.lock)
        self.color = None
        self.color_stamp = 0.0
        self.depth = None
        self.depth_scale_m = 0.001
        self.depth_stamp = 0.0
        self.camera_info = None
        self.latest_by_label: dict[str, dict] = {}
        self.jpeg = None
        self.error = "waiting for ROS topics"
        self.create_subscription(Image, args.color_topic, self.on_color, qos_profile_sensor_data)
        self.create_subscription(Image, args.depth_topic, self.on_depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, args.info_topic, self.on_info, qos_profile_sensor_data)
        self.create_subscription(Detection2DArray, args.detection_topic, self.on_detections, 10)

    @staticmethod
    def stamp_s(message) -> float:
        stamp = message.header.stamp
        value = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        return value if value > 0 else time.time()

    def on_color(self, message: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            if self.args.max_width and frame.shape[1] > self.args.max_width:
                scale = self.args.max_width / float(frame.shape[1])
                frame = cv2.resize(frame, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
            ok, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, self.args.jpeg_quality])
            with self.condition:
                self.color = frame
                self.color_stamp = self.stamp_s(message)
                if ok:
                    self.jpeg = encoded.tobytes()
                self.error = ""
                self.condition.notify_all()
        except Exception as exc:
            with self.lock:
                self.error = f"color conversion failed: {exc}"

    def on_depth(self, message: Image) -> None:
        try:
            if message.encoding in ("16UC1", "mono16"):
                array = self.bridge.imgmsg_to_cv2(message, desired_encoding="passthrough")
                scale = self.args.depth_unit_m
            elif message.encoding == "32FC1":
                array = self.bridge.imgmsg_to_cv2(message, desired_encoding="passthrough")
                scale = 1.0
            else:
                raise ValueError(f"unsupported depth encoding {message.encoding}")
            with self.condition:
                self.depth = np.asarray(array)
                self.depth_scale_m = scale
                self.depth_stamp = self.stamp_s(message)
                self.condition.notify_all()
        except Exception as exc:
            with self.lock:
                self.error = f"depth conversion failed: {exc}"

    def on_info(self, message: CameraInfo) -> None:
        with self.condition:
            self.camera_info = message
            self.condition.notify_all()

    def on_detections(self, message: Detection2DArray) -> None:
        with self.condition:
            if self.depth is None or self.camera_info is None:
                return
            depth = self.depth
            info = self.camera_info
            capture_stamp = self.stamp_s(message)
            best_by_label: dict[str, dict] = {}
            if abs(capture_stamp - self.depth_stamp) > self.args.max_sync_delta_s:
                self.error = "detection/depth timestamps are not synchronized"
                return
            height, width = depth.shape[:2]
            for detection in message.detections:
                if not detection.results:
                    continue
                best = max(detection.results, key=lambda result: float(result.hypothesis.score))
                class_id = str(best.hypothesis.class_id)
                label = next((name for name, value in COCO_CLASS_IDS.items() if value == class_id), class_id)
                bbox = detection.bbox
                center = bbox.center
                center_x = float(getattr(center, "position", center).x)
                center_y = float(getattr(center, "position", center).y)
                try:
                    x1, y1, x2, y2, cx, _ = normalized_bbox_to_pixels(
                        center_x, center_y, float(bbox.size_x), float(bbox.size_y), width, height
                    )
                    # The central third rejects background pixels near box edges.
                    roi_w = max(3, (x2 - x1) // 3)
                    roi_h = max(3, (y2 - y1) // 3)
                    roi_x1 = max(x1, cx - roi_w // 2)
                    roi_x2 = min(x2, cx + roi_w // 2 + 1)
                    cy = max(y1, min(y2 - 1, int(round(center_y * height if abs(center_y) <= 2 else center_y))))
                    roi_y1 = max(y1, cy - roi_h // 2)
                    roi_y2 = min(y2, cy + roi_h // 2 + 1)
                    distance = robust_depth_m(
                        depth[roi_y1:roi_y2, roi_x1:roi_x2].reshape(-1), self.depth_scale_m
                    )
                    bearing = bearing_from_pixel(cx, float(info.k[0]), float(info.k[2]))
                except ValueError as exc:
                    self.error = str(exc)
                    continue
                candidate = {
                    "detected": True,
                    "label": label,
                    "class_id": class_id,
                    "confidence": float(best.hypothesis.score),
                    "bearing_rad": bearing,
                    "distance_m": distance,
                    "frame_id": "camera_front",
                    "source_frame_id": message.header.frame_id,
                    "timestamp_s": capture_stamp,
                    "bbox_px": [x1, y1, x2, y2],
                }
                current = best_by_label.get(label)
                if current is None or candidate["confidence"] >= current["confidence"]:
                    best_by_label[label] = candidate
            # Replace older high-confidence results with the newest frame. Keeping
            # the historical maximum would make detection permanently stale.
            self.latest_by_label.update(best_by_label)
            self.condition.notify_all()

    def health(self) -> dict:
        now = time.time()
        with self.lock:
            image_age = now - self.color_stamp if self.color_stamp else None
            depth_age = now - self.depth_stamp if self.depth_stamp else None
            ok = (
                image_age is not None
                and depth_age is not None
                and self.camera_info is not None
                and image_age <= self.args.health_max_age_s
                and depth_age <= self.args.health_max_age_s
            )
            return {
                "ok": ok,
                "camera_id": "front",
                "model": "Orbbec Gemini 335Le",
                "camera_ip": self.args.camera_ip,
                "image_age_s": image_age,
                "depth_age_s": depth_age,
                "stream_url": f"http://{self.args.advertise_host}:{self.args.port}/mjpeg",
                "error": self.error or None,
            }

    def detect(self, label: str, timeout_s: float) -> dict:
        deadline = time.monotonic() + timeout_s
        with self.condition:
            while True:
                value = self.latest_by_label.get(label)
                if value is not None and time.time() - value["timestamp_s"] <= self.args.detection_max_age_s:
                    return dict(value)
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return {"detected": False, "label": label, "timestamp_s": time.time()}
                self.condition.wait(min(remaining, 0.25))


def make_handler(node: FrontVisionNode):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_args) -> None:
            return

        def send_json(self, value: dict, status: int = 200) -> None:
            payload = json.dumps(value, ensure_ascii=False).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)

        def do_GET(self) -> None:
            parsed = urlparse(self.path)
            query = parse_qs(parsed.query)
            if parsed.path in ("/", "/index.html"):
                body = ("<!doctype html><meta charset='utf-8'><title>V4 Gemini 335Le</title>"
                        "<style>body{margin:0;background:#111;color:#eee;font-family:sans-serif}"
                        "img{display:block;max-width:100vw;max-height:100vh;margin:auto}</style>"
                        "<img src='/mjpeg' alt='Gemini 335Le live'>").encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            elif parsed.path == "/health":
                camera_id = query.get("camera_id", ["front"])[0]
                if camera_id != "front":
                    self.send_json({"ok": False, "error": "this service only owns camera_id=front"}, 404)
                else:
                    self.send_json(node.health())
            elif parsed.path == "/detect":
                camera_id = query.get("camera_id", ["front"])[0]
                label = query.get("label", ["bottle"])[0]
                timeout_s = min(10.0, max(0.0, float(query.get("timeout_s", ["1.0"])[0])))
                if camera_id != "front":
                    self.send_json({"detected": False, "error": "unknown camera_id"}, 404)
                else:
                    self.send_json(node.detect(label, timeout_s))
            elif parsed.path == "/mjpeg":
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Cache-Control", "no-cache")
                self.end_headers()
                try:
                    while True:
                        with node.lock:
                            jpeg = node.jpeg
                        if jpeg:
                            self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " +
                                             str(len(jpeg)).encode() + b"\r\n\r\n" + jpeg + b"\r\n")
                        time.sleep(1.0 / max(1.0, node.args.stream_fps))
                except (BrokenPipeError, ConnectionResetError):
                    return
            else:
                self.send_json({"ok": False, "error": "not found"}, 404)

    return Handler


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Gemini 335Le V4 HTTP detection/MJPEG bridge")
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8088)
    parser.add_argument("--advertise-host", default="192.168.50.20")
    parser.add_argument("--camera-ip", default="192.168.30.120")
    parser.add_argument("--color-topic", default="/gemini335le/color/image_raw")
    parser.add_argument("--depth-topic", default="/gemini335le/depth/image_raw")
    parser.add_argument("--info-topic", default="/gemini335le/color/camera_info")
    parser.add_argument("--detection-topic", default="/yolov8/detections")
    parser.add_argument("--depth-unit-m", type=float, default=0.001)
    parser.add_argument("--max-sync-delta-s", type=float, default=0.25)
    parser.add_argument("--detection-max-age-s", type=float, default=0.5)
    parser.add_argument("--health-max-age-s", type=float, default=2.0)
    parser.add_argument("--max-width", type=int, default=960)
    parser.add_argument("--jpeg-quality", type=int, default=75)
    parser.add_argument("--stream-fps", type=float, default=12.0)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = FrontVisionNode(args)
    server = ThreadingHTTPServer((args.bind, args.port), make_handler(node))
    threading.Thread(target=server.serve_forever, daemon=True).start()
    node.get_logger().info(f"HTTP/MJPEG listening on {args.bind}:{args.port}")
    try:
        rclpy.spin(node)
    finally:
        server.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
