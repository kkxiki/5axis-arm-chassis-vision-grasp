#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path
from urllib.request import urlopen

import cv2
import numpy as np


def read_image(url: str) -> np.ndarray:
    raw = urlopen(url, timeout=5).read()
    image = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError("cannot decode controller camera JPEG")
    return image


def board_corners(image: np.ndarray, cols: int, rows: int) -> np.ndarray:
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    flags = cv2.CALIB_CB_NORMALIZE_IMAGE
    flags |= getattr(cv2, "CALIB_CB_EXHAUSTIVE", 0)
    flags |= getattr(cv2, "CALIB_CB_ACCURACY", 0)
    found, corners = cv2.findChessboardCornersSB(gray, (cols, rows), flags=flags)
    if not found:
        raise RuntimeError(f"{cols}x{rows} chessboard not fully detected")
    return np.asarray(corners, dtype=np.float32)


def pose_distance(a: np.ndarray, b: np.ndarray) -> tuple[float, float]:
    translation_mm = float(np.linalg.norm(a[:3] - b[:3]) * 1000.0)
    qa, qb = a[3:] / np.linalg.norm(a[3:]), b[3:] / np.linalg.norm(b[3:])
    angle_deg = float(np.degrees(2.0 * np.arccos(np.clip(abs(np.dot(qa, qb)), -1.0, 1.0))))
    return translation_mm, angle_deg


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--controller", default="192.168.30.11")
    parser.add_argument("--frame-url", default="http://127.0.0.1:8092/frame.jpg")
    parser.add_argument("--output-dir", default="/home/nvidia/5axis_v5/calibration/wrist_v4")
    parser.add_argument("--robot-state-sdk", default="/home/nvidia/codex/zhenshi_ros2_py/vendor/sdk_send_recevice_demo_py/Base_SDK_py/robot_state")
    parser.add_argument("--cols", type=int, default=5)
    parser.add_argument("--rows", type=int, default=7)
    parser.add_argument("--square-mm", type=float, default=30.5)
    args = parser.parse_args()

    sys.path.insert(0, args.robot_state_sdk)
    from platform_loader import get_topic_module
    topic = get_topic_module()
    from system_state_reader import get_current_robottarget, get_model_name, has_rt_data
    topic.start_subscriber(args.controller)
    deadline = time.time() + 8.0
    while not has_rt_data() and time.time() < deadline:
        time.sleep(0.1)
    if not has_rt_data():
        raise RuntimeError("controller robot state unavailable")

    poses = []
    for _ in range(10):
        poses.append(np.asarray(get_current_robottarget(0), dtype=np.float64))
        time.sleep(0.1)
    if any(p.shape != (7,) or not np.isfinite(p).all() for p in poses):
        raise RuntimeError("invalid controller TCP pose")
    motion = [pose_distance(poses[0], pose) for pose in poses[1:]]
    max_mm = max(value[0] for value in motion)
    max_deg = max(value[1] for value in motion)
    if max_mm > 0.5 or max_deg > 0.2:
        raise RuntimeError(f"arm is moving: {max_mm:.3f}mm/{max_deg:.3f}deg")

    image = read_image(args.frame_url)
    captured_at = time.time()
    corners = board_corners(image, args.cols, args.rows)
    output = Path(args.output_dir)
    output.mkdir(parents=True, exist_ok=True)
    manifest_path = output / "samples.json"
    if manifest_path.exists():
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    else:
        manifest = {"schema_version": 2, "calibration_mode": "eye_in_hand",
                    "board": {"inner_corners_cols": args.cols,
                              "inner_corners_rows": args.rows,
                              "square_size_mm": args.square_mm},
                    "camera": {"image_width": int(image.shape[1]),
                               "image_height": int(image.shape[0]),
                               "intrinsics_status": "solve_from_v4_samples"},
                    "samples": []}
    current = poses[-1]
    for old in manifest["samples"]:
        mm, deg = pose_distance(current, np.asarray(old["robot_pose"]["raw"], dtype=np.float64))
        if mm < 3.0 and deg < 1.0:
            raise RuntimeError(f"duplicate robot pose: {mm:.2f}mm/{deg:.2f}deg")
    sample_id = max((int(item["id"]) for item in manifest["samples"]), default=-1) + 1
    image_name = f"sample_{sample_id:03d}.jpg"
    debug_name = f"sample_{sample_id:03d}_corners.jpg"
    debug = image.copy()
    cv2.drawChessboardCorners(debug, (args.cols, args.rows), corners, True)
    cv2.imwrite(str(output / image_name), image)
    cv2.imwrite(str(output / debug_name), debug)
    manifest["samples"].append({"id": sample_id, "image": image_name,
        "debug_image": debug_name, "captured_at": captured_at,
        "corners_px": corners.reshape(-1, 2).tolist(), "model_name": get_model_name(0),
        "robot_pose": {"source_field": "official_sdk.get_current_robottarget(0)",
                       "order": "x,y,z,qx,qy,qz,qw", "raw": current.tolist(),
                       "translation_unit": "m"},
        "stationary_max_translation_mm": max_mm,
        "stationary_max_rotation_deg": max_deg})
    temporary = manifest_path.with_suffix(".json.tmp")
    temporary.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    os.replace(temporary, manifest_path)
    print(json.dumps({"ok": True, "sample_id": sample_id, "image": image_name,
                      "model_name": get_model_name(0), "pose": current.tolist(),
                      "stationary_mm": max_mm, "stationary_deg": max_deg}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
