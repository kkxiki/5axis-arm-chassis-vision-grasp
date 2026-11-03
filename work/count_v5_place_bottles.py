#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path
from urllib.request import urlopen

import cv2
import numpy as np
from ultralytics import YOLO

ROOT = Path(__file__).resolve().parents[1]


def main() -> int:
    parser = argparse.ArgumentParser(description="Count bottles in the V5 place-photo ROI.")
    parser.add_argument("--config", default=str(ROOT / "config" / "bottle_demo_v5.json"))
    parser.add_argument("--camera-url", default="http://127.0.0.1:8092")
    parser.add_argument("--model", default="/home/nvidia/camera_1/yolov8_ros2_ws/yolov8s.pt")
    parser.add_argument("--conf", type=float, default=0.15)
    parser.add_argument("--output", default=str(ROOT / "runtime" / "place_photo_roi_count_latest.jpg"))
    args = parser.parse_args()

    cfg = json.loads(Path(args.config).read_text(encoding="utf-8"))
    roi = cfg["poses"]["place_photo"].get("yolo_roi_px")
    if not isinstance(roi, list) or len(roi) != 4:
        raise RuntimeError("place_photo.yolo_roi_px is missing")
    x0, y0, x1, y1 = [int(value) for value in roi]

    image_bytes = urlopen(args.camera_url.rstrip("/") + "/frame.jpg", timeout=5).read()
    image = cv2.imdecode(np.frombuffer(image_bytes, dtype=np.uint8), cv2.IMREAD_COLOR)
    if image is None or image.size == 0:
        raise RuntimeError("failed to decode camera frame")

    model = YOLO(args.model)
    result = model.predict(image, imgsz=640, conf=args.conf, verbose=False)[0]
    detections = []
    for box in result.boxes:
        class_id = int(box.cls[0].item())
        confidence = float(box.conf[0].item())
        if class_id != int(cfg["detection"]["coco_class_id"]):
            continue
        bx0, by0, bx1, by1 = [float(value) for value in box.xyxy[0].detach().cpu().numpy().tolist()]
        cx, cy = (bx0 + bx1) / 2.0, (by0 + by1) / 2.0
        in_roi = x0 <= cx <= x1 and y0 <= cy <= y1
        detections.append({
            "confidence": confidence,
            "bbox_xyxy_px": [round(bx0), round(by0), round(bx1), round(by1)],
            "center_px": [round(cx), round(cy)],
            "in_roi": in_roi,
        })
        color = (0, 255, 0) if in_roi else (0, 0, 255)
        cv2.rectangle(image, (round(bx0), round(by0)), (round(bx1), round(by1)), color, 2)
        cv2.putText(image, f"bottle {confidence:.2f} {'IN' if in_roi else 'OUT'}",
                    (round(bx0), max(25, round(by0) - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)

    count = sum(1 for item in detections if item["in_roi"])
    cv2.rectangle(image, (x0, y0), (x1, y1), (255, 0, 0), 3)
    cv2.putText(image, f"place ROI count={count}", (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 2)
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(output), image)

    print(json.dumps({
        "ok": True,
        "place_photo_roi_px": [x0, y0, x1, y1],
        "count_in_roi": count,
        "detections": detections,
        "annotated_image": str(output),
        "motion_commands_sent": False,
    }, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
