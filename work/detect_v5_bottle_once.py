#!/usr/bin/env python3
from __future__ import annotations

import argparse
import ctypes
import json
import math
import statistics
import sys
import time
from pathlib import Path
from urllib.request import urlopen

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from v5_robot.bottle_plan import camera_point_to_arm_base, load_handeye, pixel_depth_to_camera
from v5_robot.front_geometry import robust_depth_m

# 连续多少帧取不到有效检测就判定"视野里没有瓶子"，不再空转采样。
EMPTY_STREAK_LIMIT = 3


def load_robot_pose(controller_ip: str, sdk_root: str) -> list[float]:
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
    deadline = time.time() + 8.0
    while not has_rt_data() and time.time() < deadline:
        time.sleep(0.1)
    if not has_rt_data():
        raise RuntimeError("controller realtime robot state unavailable")
    pose = get_current_robottarget(0)
    if not isinstance(pose, list) or len(pose) < 7:
        raise RuntimeError(f"invalid TCP pose: {pose!r}")
    return [float(value) for value in pose[:7]]


def fetch_rgb_depth(base_url: str, width: int, height: int) -> tuple[np.ndarray, np.ndarray, dict]:
    with urlopen(base_url.rstrip("/") + "/health", timeout=5) as response:
        health = json.loads(response.read().decode("utf-8"))
    with urlopen(base_url.rstrip("/") + "/frame.jpg", timeout=5) as response:
        rgb_jpeg = response.read()
    with urlopen(base_url.rstrip("/") + "/depth.raw", timeout=5) as response:
        depth_raw = response.read()
    image = cv2.imdecode(np.frombuffer(rgb_jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
    if image is None or image.size == 0:
        raise RuntimeError("failed to decode RGB frame")
    depth = np.frombuffer(depth_raw, dtype=np.uint16).reshape((height, width))
    return image, depth, health


def choose_bottle(results, class_id: int, min_conf: float):
    best = None
    for result in results:
        boxes = getattr(result, "boxes", None)
        if boxes is None:
            continue
        for box in boxes:
            cls = int(box.cls[0].item())
            conf = float(box.conf[0].item())
            if cls != class_id or conf < min_conf:
                continue
            xyxy = box.xyxy[0].detach().cpu().numpy().astype(float).tolist()
            if best is None or conf > best["confidence"]:
                best = {"xyxy": xyxy, "confidence": conf, "class_id": cls}
    return best


def depth_from_detection(depth: np.ndarray, detection: dict, cfg: dict, width: int, height: int):
    """从一帧的检测框内采样深度块，返回（深度中值, 采样像素, 像素框）。"""
    x1, y1, x2, y2 = [int(round(value)) for value in detection["xyxy"]]
    x1 = max(0, min(width - 1, x1))
    y1 = max(0, min(height - 1, y1))
    x2 = max(x1 + 1, min(width, x2))
    y2 = max(y1 + 1, min(height, y2))
    sample_y_ratio = float(cfg["detection"].get("depth_sample_y_ratio_in_bbox", 0.5))
    center_x = int(round((x1 + x2) / 2.0))
    sample_y = int(round(y1 + (y2 - y1) * sample_y_ratio))
    radius = int(cfg["depth"].get("median_patch_radius_px", 7))
    y0, y3 = max(0, sample_y - radius), min(height, sample_y + radius + 1)
    x0, x3 = max(0, center_x - radius), min(width, center_x + radius + 1)
    patch = depth[y0:y3, x0:x3]
    depth_m = robust_depth_m(
        patch.ravel(),
        0.001,
        float(cfg["depth"]["min_m"]),
        float(cfg["depth"]["max_m"]),
    )
    return depth_m, center_x, sample_y, (x1, y1, x2, y2)


def consistent_inliers(values: list[float], max_deviation_m: float) -> list[int]:
    """返回最大一致簇的下标：簇内每个值与簇中心的偏差都不超过阈值。"""
    best: list[int] = []
    for candidate in values:
        inliers = [index for index, value in enumerate(values) if abs(value - candidate) <= max_deviation_m]
        if len(inliers) > len(best):
            best = inliers
    return best


def main() -> int:
    parser = argparse.ArgumentParser(description="Detect one V5 wrist-camera bottle and compute arm_base coordinates.")
    parser.add_argument("--config", default=str(ROOT / "config" / "bottle_demo_v5.json"))
    parser.add_argument("--controller", default="192.168.30.11")
    parser.add_argument("--camera-url", default="http://127.0.0.1:8092")
    parser.add_argument("--model", default="/home/nvidia/camera_1/yolov8_ros2_ws/yolov8s.pt")
    parser.add_argument("--robot-state-sdk", default="/home/nvidia/codex/zhenshi_ros2_py/vendor/sdk_send_recevice_demo_py/Base_SDK_py/robot_state")
    parser.add_argument("--output", default=str(ROOT / "runtime" / "v5_bottle_detection_latest.jpg"))
    parser.add_argument("--json-output", default=str(ROOT / "runtime" / "v5_bottle_detection_latest.json"))
    parser.add_argument("--attempts", type=int, default=3,
                        help="多帧一致性筛选失败后重采样的轮数；不降低 detection.minimum_confidence")
    args = parser.parse_args()

    from ultralytics import YOLO

    cfg = json.loads(Path(args.config).read_text(encoding="utf-8"))
    width, height = cfg.get("image_size", [1280, 720])
    handeye = load_handeye(cfg["handeye"].get("jetson_result_path") or cfg["handeye"]["result_path"])
    tcp_pose = load_robot_pose(args.controller, args.robot_state_sdk)

    model = YOLO(args.model)
    min_conf = float(cfg["detection"]["minimum_confidence"])
    samples_needed = max(1, int(cfg["detection"].get("samples", 15)))
    max_deviation_m = float(cfg["detection"].get("maximum_inlier_deviation_m", 0.008))
    minimum_inliers = max(1, int(cfg["detection"].get("minimum_inliers", 9)))
    rounds = max(1, int(args.attempts))

    # 单帧深度中值容易被反光/边缘跳变带偏，导致同一点位识别结果忽高忽低。
    # 这里连续采 samples 帧，取"最大一致簇"的中值作为最终深度。
    frames: list[dict] = []
    inliers: list[int] = []
    health = None
    rounds_used = 0
    for round_index in range(1, rounds + 1):
        rounds_used = round_index
        frames = []
        miss_streak = 0
        for _ in range(samples_needed):
            image, depth, health = fetch_rgb_depth(args.camera_url, width, height)
            results = model.predict(image, imgsz=640, conf=min_conf, verbose=False)
            detection = choose_bottle(results, int(cfg["detection"]["coco_class_id"]), min_conf)
            sample = None
            if detection is not None:
                try:
                    depth_m, center_x, sample_y, bbox = depth_from_detection(depth, detection, cfg, width, height)
                    sample = {
                        "depth_m": depth_m,
                        "center_x": center_x,
                        "sample_y": sample_y,
                        "bbox": bbox,
                        "confidence": detection["confidence"],
                        "image": image,
                        "health": health,
                    }
                except ValueError:
                    # 深度块全是无效像素（瓶子贴太近或反光），这一帧作废。
                    sample = None
            if sample is None:
                miss_streak += 1
                # 连续几帧都没有有效检测，说明视野里根本没有瓶子，提前收工。
                if not frames and miss_streak >= EMPTY_STREAK_LIMIT:
                    break
                continue
            miss_streak = 0
            frames.append(sample)
            if len(frames) >= minimum_inliers:
                candidate_inliers = consistent_inliers([frame["depth_m"] for frame in frames], max_deviation_m)
                if len(candidate_inliers) >= minimum_inliers:
                    inliers = candidate_inliers
                    break
        if inliers:
            break
        if not frames:
            # 整轮一帧都没检测到，重采样也不会变好。
            break
        time.sleep(0.15)

    if not inliers:
        print(json.dumps({
            "ok": False,
            "error": "no bottle detected" if not frames else "no consistent bottle depth",
            "attempts": rounds_used,
            "samples_requested": samples_needed,
            "samples_valid": len(frames),
            "minimum_inliers": minimum_inliers,
            "maximum_inlier_deviation_m": max_deviation_m,
            "minimum_confidence": min_conf,
            "depth_samples_m": [round(frame["depth_m"], 4) for frame in frames],
            "health": health,
        }, ensure_ascii=False, indent=2))
        return 2

    inlier_depths = [frames[index]["depth_m"] for index in inliers]
    depth_m = float(statistics.median(inlier_depths))
    # 用最接近内点中值的那一帧出图和像素坐标，保证 bbox / 采样点 / 深度自洽。
    representative = min(inliers, key=lambda index: abs(frames[index]["depth_m"] - depth_m))
    frame = frames[representative]
    image = frame["image"]
    health = frame["health"]
    x1, y1, x2, y2 = frame["bbox"]
    center_x = frame["center_x"]
    sample_y = frame["sample_y"]
    confidence = frame["confidence"]
    point_camera = pixel_depth_to_camera(center_x, sample_y, depth_m, handeye["camera_matrix"])
    point_base = camera_point_to_arm_base(point_camera, tcp_pose, handeye["camera_to_tcp_matrix"])

    annotated = image.copy()
    cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
    cv2.drawMarker(annotated, (center_x, sample_y), (0, 0, 255), cv2.MARKER_CROSS, 28, 2)
    label = f"bottle {confidence:.2f} base=({point_base[0]:.3f},{point_base[1]:.3f},{point_base[2]:.3f})m z={depth_m:.3f}m"
    cv2.putText(annotated, label, (max(10, x1), max(30, y1 - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(output), annotated)

    report = {
        "ok": True,
        "label": "bottle",
        "confidence": confidence,
        "attempts": rounds_used,
        "samples_requested": samples_needed,
        "samples_valid": len(frames),
        "inlier_count": len(inliers),
        "minimum_inliers": minimum_inliers,
        "maximum_inlier_deviation_m": max_deviation_m,
        "depth_samples_m": [round(item["depth_m"], 4) for item in frames],
        "minimum_confidence": min_conf,
        "bbox_xyxy_px": [x1, y1, x2, y2],
        "depth_sample_pixel": [center_x, sample_y],
        "depth_m": depth_m,
        "point_camera_m": list(point_camera),
        "position_base_m": list(point_base),
        "tcp_pose": tcp_pose,
        "handeye_method": handeye["selected_method"],
        "annotated_image": str(output),
        "health": health,
        "motion_commands_sent": False,
    }
    json_output = Path(args.json_output)
    json_output.parent.mkdir(parents=True, exist_ok=True)
    json_output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    report["json_output"] = str(json_output)
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
