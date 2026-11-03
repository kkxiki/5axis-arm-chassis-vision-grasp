#!/usr/bin/env python3
"""Solve base_T_camera for a fixed camera and a board on the gripper."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from handeye_common import (
    find_board,
    invert_transform,
    make_transform,
    params_to_transform,
    solve_board_pose,
    transform_error,
    transform_to_params,
)


def robot_world_handeye_initialization(
    camera_t_targets: list[np.ndarray],
    base_t_grippers: list[np.ndarray],
) -> tuple[np.ndarray, np.ndarray, str]:
    candidates = []
    methods = [
        ("SHAH", cv2.CALIB_ROBOT_WORLD_HAND_EYE_SHAH),
        ("LI", cv2.CALIB_ROBOT_WORLD_HAND_EYE_LI),
    ]
    for name, method in methods:
        try:
            result = cv2.calibrateRobotWorldHandEye(
                [item[:3, :3] for item in camera_t_targets],
                [item[:3, 3].reshape(3, 1) for item in camera_t_targets],
                [item[:3, :3] for item in base_t_grippers],
                [item[:3, 3].reshape(3, 1) for item in base_t_grippers],
                method=method,
            )
            r_target_to_gripper, t_target_to_gripper, r_camera_to_base, t_camera_to_base = result
            target_t_gripper = make_transform(
                r_target_to_gripper, np.asarray(t_target_to_gripper).reshape(3)
            )
            camera_t_base = make_transform(
                r_camera_to_base, np.asarray(t_camera_to_base).reshape(3)
            )
            gripper_t_target = invert_transform(target_t_gripper)
            base_t_camera = invert_transform(camera_t_base)
            errors = [
                transform_error(
                    base_t_gripper @ gripper_t_target,
                    base_t_camera @ camera_t_target,
                )
                for base_t_gripper, camera_t_target in zip(
                    base_t_grippers, camera_t_targets
                )
            ]
            score = np.mean([e[0] for e in errors]) + 5.0 * np.mean(
                [e[1] for e in errors]
            )
            if np.isfinite(score):
                candidates.append(
                    (float(score), base_t_camera, gripper_t_target, name)
                )
        except cv2.error:
            continue
    if not candidates:
        raise RuntimeError("OpenCV robot-world/hand-eye initialization failed")
    _, base_t_camera, gripper_t_target, name = min(candidates, key=lambda x: x[0])
    return base_t_camera, gripper_t_target, name


def refine_solution(
    base_t_camera_initial: np.ndarray,
    gripper_t_target_initial: np.ndarray,
    base_t_grippers: list[np.ndarray],
    camera_t_targets: list[np.ndarray],
) -> tuple[np.ndarray, np.ndarray, object]:
    initial = np.concatenate(
        [
            transform_to_params(base_t_camera_initial),
            transform_to_params(gripper_t_target_initial),
        ]
    )

    def residuals(parameters):
        base_t_camera = params_to_transform(parameters[:6])
        gripper_t_target = params_to_transform(parameters[6:])
        output = []
        for base_t_gripper, camera_t_target in zip(
            base_t_grippers, camera_t_targets
        ):
            lhs = base_t_gripper @ gripper_t_target
            rhs = base_t_camera @ camera_t_target
            error = invert_transform(lhs) @ rhs
            # 10 mm and 1 degree have comparable influence.
            output.extend(error[:3, 3] / 0.010)
            output.extend(
                Rotation.from_matrix(error[:3, :3]).as_rotvec()
                / np.deg2rad(1.0)
            )
        return np.asarray(output, dtype=np.float64)

    optimization = least_squares(
        residuals,
        initial,
        method="trf",
        loss="soft_l1",
        f_scale=1.0,
        max_nfev=5000,
    )
    return (
        params_to_transform(optimization.x[:6]),
        params_to_transform(optimization.x[6:]),
        optimization,
    )


def load_observations(manifest_path: Path):
    with manifest_path.open("r", encoding="utf-8") as stream:
        manifest = json.load(stream)
    board = manifest["board"]
    cols = int(board["inner_corners_cols"])
    rows = int(board["inner_corners_rows"])
    square_size_m = float(board["square_size_mm"]) / 1000.0
    camera = manifest["camera"]
    camera_matrix = np.asarray(camera["k"], dtype=np.float64).reshape(3, 3)
    distortion = np.asarray(camera["d"], dtype=np.float64)

    base_t_grippers = []
    camera_t_targets = []
    reprojection_errors = []
    sample_ids = []
    for sample in manifest["samples"]:
        image_path = manifest_path.parent / sample["image"]
        image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if image is None:
            raise RuntimeError(f"cannot read {image_path}")
        found, corners = find_board(image, cols, rows, accurate=True)
        if not found:
            raise RuntimeError(f"board no longer detected in {image_path.name}")
        camera_t_target, reprojection_rms = solve_board_pose(
            corners,
            cols,
            rows,
            square_size_m,
            camera_matrix,
            distortion,
        )
        base_t_gripper = np.asarray(
            sample["robot_pose"]["base_T_gripper_m"], dtype=np.float64
        ).reshape(4, 4)
        base_t_grippers.append(base_t_gripper)
        camera_t_targets.append(camera_t_target)
        reprojection_errors.append(reprojection_rms)
        sample_ids.append(int(sample["id"]))
    return (
        manifest,
        camera_matrix,
        distortion,
        base_t_grippers,
        camera_t_targets,
        reprojection_errors,
        sample_ids,
    )


def solve(args: argparse.Namespace) -> dict:
    manifest_path = Path(args.manifest)
    (
        manifest,
        camera_matrix,
        distortion,
        base_t_grippers,
        camera_t_targets,
        reprojection_errors,
        sample_ids,
    ) = load_observations(manifest_path)
    if len(base_t_grippers) < args.min_samples:
        raise RuntimeError(
            f"need at least {args.min_samples} samples, got {len(base_t_grippers)}"
        )

    initial_camera, initial_target, method = robot_world_handeye_initialization(
        camera_t_targets, base_t_grippers
    )
    base_t_camera, gripper_t_target, optimization = refine_solution(
        initial_camera,
        initial_target,
        base_t_grippers,
        camera_t_targets,
    )

    pose_errors = [
        transform_error(
            base_t_gripper @ gripper_t_target,
            base_t_camera @ camera_t_target,
        )
        for base_t_gripper, camera_t_target in zip(
            base_t_grippers, camera_t_targets
        )
    ]
    translation_errors = np.asarray([item[0] for item in pose_errors])
    rotation_errors = np.asarray([item[1] for item in pose_errors])
    reprojection_errors_array = np.asarray(reprojection_errors)
    passed = bool(
        np.sqrt(np.mean(translation_errors**2)) <= args.max_translation_rms_mm
        and np.sqrt(np.mean(rotation_errors**2)) <= args.max_rotation_rms_deg
        and np.sqrt(np.mean(reprojection_errors_array**2))
        <= args.max_reprojection_rms_px
    )

    per_sample = []
    for index, sample_id in enumerate(sample_ids):
        per_sample.append(
            {
                "id": sample_id,
                "translation_error_mm": float(translation_errors[index]),
                "rotation_error_deg": float(rotation_errors[index]),
                "reprojection_rms_px": float(reprojection_errors[index]),
            }
        )
    report = {
        "schema_version": 1,
        "calibration_mode": "eye_to_hand",
        "equation": "base_T_gripper * gripper_T_target = base_T_camera * camera_T_target",
        "source_manifest": str(manifest_path),
        "sample_count": len(base_t_grippers),
        "opencv_initializer": method,
        "nonlinear_optimization_success": bool(optimization.success),
        "nonlinear_optimization_message": str(optimization.message),
        "base_T_camera_m": base_t_camera.tolist(),
        "gripper_T_target_m": gripper_t_target.tolist(),
        "translation_rms_mm": float(np.sqrt(np.mean(translation_errors**2))),
        "translation_max_mm": float(np.max(translation_errors)),
        "rotation_rms_deg": float(np.sqrt(np.mean(rotation_errors**2))),
        "rotation_max_deg": float(np.max(rotation_errors)),
        "reprojection_rms_px": float(
            np.sqrt(np.mean(reprojection_errors_array**2))
        ),
        "quality_thresholds": {
            "max_translation_rms_mm": args.max_translation_rms_mm,
            "max_rotation_rms_deg": args.max_rotation_rms_deg,
            "max_reprojection_rms_px": args.max_reprojection_rms_px,
        },
        "quality_passed": passed,
        "samples": per_sample,
        "camera": manifest["camera"],
        "board": manifest["board"],
    }

    output_json = Path(args.output_json)
    output_npz = Path(args.output_npz)
    for path in (output_json, output_npz):
        if path.exists() and not args.force:
            raise RuntimeError(f"refusing to overwrite {path}; pass --force")
        path.parent.mkdir(parents=True, exist_ok=True)
    with output_json.open("w", encoding="utf-8") as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2)
        stream.write("\n")

    # The legacy transform server multiplies X by camera points expressed in
    # millimetres. Therefore X stores the same rotation with translation in mm.
    legacy_x_mm = base_t_camera.copy()
    legacy_x_mm[:3, 3] *= 1000.0
    np.savez(
        output_npz,
        X=legacy_x_mm,
        mtx=camera_matrix,
        dist=distortion,
        base_T_camera_m=base_t_camera,
        gripper_T_target_m=gripper_t_target,
        calibration_mode=np.asarray("eye_to_hand"),
        sample_count=np.asarray(len(base_t_grippers), dtype=np.int32),
        quality_passed=np.asarray(passed),
    )
    return report


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--manifest",
        default="/root/XuK/calibration/recalibration/right/samples.json",
    )
    parser.add_argument(
        "--output-json",
        default="/root/XuK/calibration/recalibration/right/"
        "right_handeye_result_new.json",
    )
    parser.add_argument(
        "--output-npz",
        default="/root/XuK/calibration/recalibration/right/"
        "righthandeye_matrix_new.npz",
    )
    parser.add_argument("--min-samples", type=int, default=10)
    parser.add_argument("--max-translation-rms-mm", type=float, default=10.0)
    parser.add_argument("--max-rotation-rms-deg", type=float, default=2.0)
    parser.add_argument("--max-reprojection-rms-px", type=float, default=1.5)
    parser.add_argument("--force", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    report = solve(args)
    print(json.dumps(report, ensure_ascii=False, indent=2))
    if not report["quality_passed"]:
        raise SystemExit(2)


if __name__ == "__main__":
    main()

