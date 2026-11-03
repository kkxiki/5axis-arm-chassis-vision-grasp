#!/usr/bin/env python3
"""Solve eye-in-hand calibration for the right wrist D405.

Input : samples.json written by capture_handeye_samples.py
        (each sample has robot_pose.base_T_gripper_m and camera_T_target_m)
Solve : cv2.calibrateHandEye  (AX = XB)
Output: right_eye_in_hand_result.json and right_wrist_d405_to_tcp.yaml

All text in this file is ASCII on purpose (Windows host encoding issue).
"""
import argparse
import json
import math
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from handeye_common import invert_transform, transform_error


def solve(samples, method_flags):
    R_g2b, t_g2b, R_t2c, t_t2c = [], [], [], []
    for s in samples:
        bTg = np.asarray(s["robot_pose"]["base_T_gripper_m"]).reshape(4, 4)
        cTt = np.asarray(s["camera_T_target_m"]).reshape(4, 4)
        R_g2b.append(bTg[:3, :3])
        t_g2b.append(bTg[:3, 3])
        R_t2c.append(cTt[:3, :3])
        t_t2c.append(cTt[:3, 3])
    best = None
    for name, flag in method_flags:
        try:
            R_c2g, t_c2g = cv2.calibrateHandEye(
                np.asarray(R_g2b), np.asarray(t_g2b),
                np.asarray(R_t2c), np.asarray(t_t2c), method=flag)
        except Exception as exc:
            print("method", name, "failed:", exc)
            continue
        gTcam = np.eye(4)
        gTcam[:3, :3] = np.asarray(R_c2g)
        gTcam[:3, 3] = np.asarray(t_c2g).reshape(3)
        t_rms, r_rms = validate(samples, gTcam)
        entry = {"method": name, "gripper_T_camera": gTcam,
                 "translation_rms_m": t_rms, "rotation_rms_deg": r_rms}
        if best is None or (t_rms + r_rms * 0.001) < (best["translation_rms_m"] + best["rotation_rms_deg"] * 0.001):
            best = entry
        print("method=%s translation_rms_m=%.6f rotation_rms_deg=%.4f" % (name, t_rms, r_rms))
    return best


def validate(samples, gTcam):
    """Board is fixed in the base frame; check consistency of base_T_board."""
    boards = []
    for s in samples:
        bTg = np.asarray(s["robot_pose"]["base_T_gripper_m"]).reshape(4, 4)
        cTt = np.asarray(s["camera_T_target_m"]).reshape(4, 4)
        boards.append(bTg @ gTcam @ cTt)
    mean = np.mean(np.asarray(boards), axis=0)
    t_errs, r_errs = [], []
    for b in boards:
        tm, rm = transform_error(mean, b)
        t_errs.append(tm / 1000.0)   # mm -> m
        r_errs.append(rm)
    t_rms = float(np.sqrt(np.mean(np.square(t_errs))))
    r_rms = float(np.sqrt(np.mean(np.square(r_errs))))
    return t_rms, r_rms


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--samples", default="/root/XuK/calibration/recalibration/right_eye_in_hand/samples.json")
    parser.add_argument("--out-dir", default="/root/XuK/calibration/recalibration/right_eye_in_hand")
    args = parser.parse_args()

    with open(args.samples, encoding="utf-8") as stream:
        manifest = json.load(stream)
    samples = manifest["samples"]
    if len(samples) < 9:
        raise SystemExit("need >= 9 samples, got %d" % len(samples))

    methods = [
        ("tsai", cv2.CALIB_HAND_EYE_TSAI),
        ("park", cv2.CALIB_HAND_EYE_PARK),
        ("horaud", cv2.CALIB_HAND_EYE_HORAUD),
        ("andreff", cv2.CALIB_HAND_EYE_ANDREFF),
        ("daniilidis", cv2.CALIB_HAND_EYE_DANIILIDIS),
    ]
    best = solve(samples, methods)
    if best is None:
        raise SystemExit("calibrateHandEye failed for every method")

    gTcam = best["gripper_T_camera"]
    quat = Rotation.from_matrix(gTcam[:3, :3]).as_quat()  # xyzw
    t = gTcam[:3, 3]
    result = {
        "schema_version": 1,
        "camera_id": "right_wrist_d405",
        "parent_frame": "right_tcp",
        "child_frame": "right_d405_link",
        "method": best["method"],
        "translation_m": [float(v) for v in t],
        "quaternion_xyzw": [float(v) for v in quat],
        "translation_rms_m": best["translation_rms_m"],
        "rotation_rms_deg": best["rotation_rms_deg"],
        "sample_count": len(samples),
        "valid": True,
        "samples_used": [s["id"] for s in samples],
    }
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    json_path = out_dir / "right_eye_in_hand_result.json"
    json_path.write_text(json.dumps(result, indent=2), encoding="utf-8")

    yaml_lines = [
        "schema_version: 1",
        "camera_id: right_wrist_d405",
        "parent_frame: right_tcp",
        "child_frame: right_d405_link",
        "translation_m: [%.9f, %.9f, %.9f]" % tuple(t),
        "quaternion_xyzw: [%.9f, %.9f, %.9f, %.9f]" % tuple(quat),
        "translation_rms_m: %.9f" % best["translation_rms_m"],
        "rotation_rms_deg: %.6f" % best["rotation_rms_deg"],
        "sample_count: %d" % len(samples),
        "valid: true",
        "",
    ]
    yaml_path = out_dir / "right_wrist_d405_to_tcp.yaml"
    yaml_path.write_text("\n".join(yaml_lines), encoding="utf-8")

    print("=== RESULT ===")
    print(json.dumps(result, indent=2))
    print("=== YAML ===")
    print("\n".join(yaml_lines))


if __name__ == "__main__":
    main()
