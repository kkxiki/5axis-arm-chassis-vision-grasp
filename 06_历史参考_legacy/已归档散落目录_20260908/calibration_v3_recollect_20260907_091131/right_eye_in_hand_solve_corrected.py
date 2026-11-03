#!/usr/bin/env python3
"""Robust eye-in-hand solver for an unmarked symmetric chessboard.

The 5x7 inner-corner pattern has a 180 degree indexing ambiguity.  OpenCV may
return the opposite corner as point zero in some views.  This solver searches
that ambiguity, evaluates board consistency with a proper SO(3) mean, and may
drop clear geometric outliers while retaining at least nine samples.
"""

from __future__ import annotations

import argparse
import itertools
import json
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation


METHODS = [
    ("tsai", cv2.CALIB_HAND_EYE_TSAI),
    ("park", cv2.CALIB_HAND_EYE_PARK),
    ("horaud", cv2.CALIB_HAND_EYE_HORAUD),
    ("andreff", cv2.CALIB_HAND_EYE_ANDREFF),
    ("daniilidis", cv2.CALIB_HAND_EYE_DANIILIDIS),
]


def board_flip(cols: int, rows: int, square_m: float) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_euler("z", 180, degrees=True).as_matrix()
    transform[:3, 3] = [(cols - 1) * square_m, (rows - 1) * square_m, 0.0]
    return transform


def consistency(boards: list[np.ndarray]):
    translations = np.asarray([pose[:3, 3] for pose in boards])
    translation_mean = translations.mean(axis=0)
    rotations = Rotation.from_matrix(np.asarray([pose[:3, :3] for pose in boards]))
    rotation_mean = rotations.mean()
    translation_errors = np.linalg.norm(translations - translation_mean, axis=1)
    rotation_errors = np.degrees(
        np.linalg.norm((rotation_mean.inv() * rotations).as_rotvec(), axis=1)
    )
    return {
        "translation_rms_m": float(np.sqrt(np.mean(translation_errors**2))),
        "rotation_rms_deg": float(np.sqrt(np.mean(rotation_errors**2))),
        "translation_errors_m": translation_errors,
        "rotation_errors_deg": rotation_errors,
        "base_T_board_translation_m": translation_mean,
        "base_T_board_quaternion_xyzw": rotation_mean.as_quat(),
    }


def solve_candidate(base_t_gripper, camera_t_target, ids, method_name, method_flag):
    rotation, translation = cv2.calibrateHandEye(
        np.asarray([base_t_gripper[i][:3, :3] for i in ids]),
        np.asarray([base_t_gripper[i][:3, 3] for i in ids]),
        np.asarray([camera_t_target[i][:3, :3] for i in ids]),
        np.asarray([camera_t_target[i][:3, 3] for i in ids]),
        method=method_flag,
    )
    gripper_t_camera = np.eye(4)
    gripper_t_camera[:3, :3] = rotation
    gripper_t_camera[:3, 3] = np.asarray(translation).reshape(3)
    stats = consistency(
        [base_t_gripper[i] @ gripper_t_camera @ camera_t_target[i] for i in ids]
    )
    # One degree of angular board inconsistency is weighted like 2 mm.
    score = stats["translation_rms_m"] + 0.002 * stats["rotation_rms_deg"]
    return {
        "score": float(score),
        "method": method_name,
        "gripper_T_camera": gripper_t_camera,
        "ids": list(ids),
        **stats,
    }


def best_method(base_t_gripper, camera_t_target, ids):
    candidates = []
    for method_name, method_flag in METHODS:
        try:
            candidates.append(
                solve_candidate(
                    base_t_gripper, camera_t_target, ids, method_name, method_flag
                )
            )
        except Exception as exc:
            print(f"method={method_name} failed: {exc}")
    if not candidates:
        raise RuntimeError("all hand-eye methods failed")
    return min(candidates, key=lambda item: item["score"])


def find_orientation(base_t_gripper, original_camera_t_target, flip):
    count = len(original_camera_t_target)
    if count > 16:
        raise RuntimeError("exhaustive chessboard disambiguation supports at most 16 samples")
    best = None
    # Fix sample zero as the global board-frame convention; its complement is
    # physically equivalent and would only rename the board frame.
    for tail in itertools.product((False, True), repeat=count - 1):
        bits = (False,) + tail
        camera_t_target = [
            pose @ flip if bits[index] else pose
            for index, pose in enumerate(original_camera_t_target)
        ]
        candidate = best_method(base_t_gripper, camera_t_target, range(count))
        if best is None or candidate["score"] < best["solution"]["score"]:
            best = {
                "bits": bits,
                "camera_t_target": camera_t_target,
                "solution": candidate,
            }
    return best


def robust_trim(base_t_gripper, camera_t_target, initial, minimum=9):
    current = initial
    excluded = []
    while len(current["ids"]) > minimum:
        trials = []
        for sample_id in current["ids"]:
            ids = [value for value in current["ids"] if value != sample_id]
            candidate = best_method(base_t_gripper, camera_t_target, ids)
            trials.append((candidate["score"], sample_id, candidate))
        _, sample_id, candidate = min(trials, key=lambda item: item[0])
        improvement = (current["score"] - candidate["score"]) / current["score"]
        # Require a material improvement to avoid overfitting a nine-pose subset.
        if improvement < 0.15:
            break
        excluded.append(sample_id)
        current = candidate
    return current, excluded


def plain(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.floating, np.integer)):
        return value.item()
    raise TypeError(type(value).__name__)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--samples", required=True)
    parser.add_argument("--out-dir", required=True)
    args = parser.parse_args()

    manifest = json.loads(Path(args.samples).read_text(encoding="utf-8"))
    samples = manifest["samples"]
    if len(samples) < 9:
        raise SystemExit(f"need >=9 samples, got {len(samples)}")

    board = manifest["board"]
    cols = int(board["inner_corners_cols"])
    rows = int(board["inner_corners_rows"])
    square_m = float(board["square_size_mm"]) / 1000.0
    flip = board_flip(cols, rows, square_m)
    base_t_gripper = [
        np.asarray(sample["robot_pose"]["base_T_gripper_m"], dtype=float)
        for sample in samples
    ]
    original_camera_t_target = [
        np.asarray(sample["camera_T_target_m"], dtype=float) for sample in samples
    ]

    orientation = find_orientation(base_t_gripper, original_camera_t_target, flip)
    corrected_camera_t_target = orientation["camera_t_target"]
    solution, excluded = robust_trim(
        base_t_gripper, corrected_camera_t_target, orientation["solution"]
    )
    flipped = [index for index, value in enumerate(orientation["bits"]) if value]
    used = solution["ids"]
    transform = solution["gripper_T_camera"]
    translation = transform[:3, 3]
    quaternion = Rotation.from_matrix(transform[:3, :3]).as_quat()
    valid = (
        solution["translation_rms_m"] <= 0.010
        and solution["rotation_rms_deg"] <= 2.0
        and len(used) >= 9
    )

    result = {
        "schema_version": 2,
        "calibration_mode": "eye_in_hand",
        "camera_id": "wrist_d435i",
        "parent_frame": "tcp",
        "child_frame": "d435i_color_optical_frame",
        "method": solution["method"],
        "translation_m": translation,
        "quaternion_xyzw": quaternion,
        "translation_rms_m": solution["translation_rms_m"],
        "rotation_rms_deg": solution["rotation_rms_deg"],
        "sample_count_total": len(samples),
        "sample_count_used": len(used),
        "samples_used": used,
        "samples_excluded": excluded,
        "checkerboard_180_flipped_samples": flipped,
        "per_used_sample_translation_error_m": {
            str(sample_id): float(solution["translation_errors_m"][index])
            for index, sample_id in enumerate(used)
        },
        "per_used_sample_rotation_error_deg": {
            str(sample_id): float(solution["rotation_errors_deg"][index])
            for index, sample_id in enumerate(used)
        },
        "base_T_board_translation_m": solution["base_T_board_translation_m"],
        "base_T_board_quaternion_xyzw": solution["base_T_board_quaternion_xyzw"],
        "valid": valid,
        "validity_limits": {"translation_rms_m": 0.010, "rotation_rms_deg": 2.0},
    }

    output = Path(args.out_dir)
    output.mkdir(parents=True, exist_ok=True)
    json_path = output / "right_eye_in_hand_result_corrected.json"
    json_path.write_text(json.dumps(result, indent=2, default=plain), encoding="utf-8")
    yaml_path = output / "wrist_d435i_to_tcp_corrected.yaml"
    yaml_path.write_text(
        "\n".join(
            [
                "schema_version: 2",
                "calibration_mode: eye_in_hand",
                "camera_id: wrist_d435i",
                "parent_frame: tcp",
                "child_frame: d435i_color_optical_frame",
                f"translation_m: [{translation[0]:.9f}, {translation[1]:.9f}, {translation[2]:.9f}]",
                "quaternion_xyzw: "
                f"[{quaternion[0]:.9f}, {quaternion[1]:.9f}, {quaternion[2]:.9f}, {quaternion[3]:.9f}]",
                f"translation_rms_m: {solution['translation_rms_m']:.9f}",
                f"rotation_rms_deg: {solution['rotation_rms_deg']:.6f}",
                f"sample_count_total: {len(samples)}",
                f"sample_count_used: {len(used)}",
                f"samples_used: {used}",
                f"samples_excluded: {excluded}",
                f"checkerboard_180_flipped_samples: {flipped}",
                f"valid: {str(valid).lower()}",
                "",
            ]
        ),
        encoding="utf-8",
    )
    print(json.dumps(result, indent=2, default=plain))
    print(f"JSON={json_path}")
    print(f"YAML={yaml_path}")
    if not valid:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
