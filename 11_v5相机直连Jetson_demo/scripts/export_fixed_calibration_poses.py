#!/usr/bin/env python3
"""Export captured hand-eye samples as reusable fixed TCP calibration poses."""
from __future__ import annotations

import argparse
import csv
import json
from datetime import datetime
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--samples", required=True)
    parser.add_argument("--result", required=True)
    parser.add_argument("--output-dir", required=True)
    args = parser.parse_args()

    samples_path = Path(args.samples)
    result_path = Path(args.result)
    output = Path(args.output_dir)
    output.mkdir(parents=True, exist_ok=True)
    samples = json.loads(samples_path.read_text(encoding="utf-8"))
    result = json.loads(result_path.read_text(encoding="utf-8"))

    poses = []
    for sample in samples["samples"]:
        raw = sample["robot_pose"]["raw"]
        poses.append({
            "name": f"v4_calib_pose_{int(sample['id']):03d}",
            "sample_id": int(sample["id"]),
            "image": sample["image"],
            "corner_image": sample["debug_image"],
            "pose_order": "x,y,z,qx,qy,qz,qw",
            "translation_unit": "m",
            "raw": raw,
            "x_m": raw[0], "y_m": raw[1], "z_m": raw[2],
            "qx": raw[3], "qy": raw[4], "qz": raw[5], "qw": raw[6],
            "used_in_calibration": True,
            "note": "V4 wrist camera, eye-in-hand, 1280x720, 5x7 corners, 30.5mm square",
        })

    selected = result["selected_method"]
    quality = result["methods"][selected]["consistency"]
    document = {
        "schema_version": 2,
        "created_at": datetime.now().astimezone().isoformat(timespec="seconds"),
        "created_from": str(samples_path),
        "calibration_result": str(result_path),
        "calibration_status": "preliminary_only",
        "controller_ip": "192.168.30.11",
        "robot_model": samples["samples"][0].get("model_name", "taiKeU058T_0"),
        "camera_subsystem": "RsCameraSensor#0#0",
        "image_size": result["image_size"],
        "board": samples["board"],
        "pose_order": "x,y,z,qx,qy,qz,qw",
        "translation_unit": "m",
        "count": len(poses),
        "used_sample_ids": [p["sample_id"] for p in poses],
        "selected_handeye_method": selected,
        "quality": quality,
        "warning": "Replay only after workcell safety and unchanged board/camera installation are confirmed.",
        "poses": poses,
    }
    (output / "fixed_calibration_poses_v4.json").write_text(
        json.dumps(document, ensure_ascii=False, indent=2), encoding="utf-8")

    with (output / "fixed_calibration_poses_v4.csv").open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.writer(stream)
        writer.writerow(["name", "sample_id", "used_in_calibration", "x_m", "y_m", "z_m",
                         "qx", "qy", "qz", "qw", "image"])
        for pose in poses:
            writer.writerow([pose["name"], pose["sample_id"], pose["used_in_calibration"],
                             *pose["raw"], pose["image"]])
    print(json.dumps({"ok": True, "count": len(poses), "output_dir": str(output)}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
