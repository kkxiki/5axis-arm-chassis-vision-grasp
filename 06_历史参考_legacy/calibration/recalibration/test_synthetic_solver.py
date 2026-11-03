#!/usr/bin/env python3
"""Noise-free synthetic regression test for the eye-to-hand equation."""

import numpy as np
from scipy.spatial.transform import Rotation

from handeye_common import invert_transform, make_transform, transform_error
from solve_eye_to_hand import refine_solution, robot_world_handeye_initialization


def random_transform(rng, translation_range):
    return make_transform(
        Rotation.random(random_state=rng).as_matrix(),
        rng.uniform(-translation_range, translation_range, size=3),
    )


def main():
    rng = np.random.default_rng(335)
    expected_base_t_camera = make_transform(
        Rotation.from_euler("xyz", [12.0, -18.0, 95.0], degrees=True).as_matrix(),
        [0.42, -0.31, 1.15],
    )
    expected_gripper_t_target = make_transform(
        Rotation.from_euler("xyz", [90.0, 0.0, 15.0], degrees=True).as_matrix(),
        [0.03, -0.01, 0.12],
    )
    base_t_grippers = []
    camera_t_targets = []
    for _ in range(18):
        base_t_gripper = random_transform(rng, 0.7)
        camera_t_target = (
            invert_transform(expected_base_t_camera)
            @ base_t_gripper
            @ expected_gripper_t_target
        )
        base_t_grippers.append(base_t_gripper)
        camera_t_targets.append(camera_t_target)

    initial_camera, initial_target, method = robot_world_handeye_initialization(
        camera_t_targets, base_t_grippers
    )
    solved_camera, solved_target, result = refine_solution(
        initial_camera,
        initial_target,
        base_t_grippers,
        camera_t_targets,
    )
    camera_error = transform_error(expected_base_t_camera, solved_camera)
    target_error = transform_error(expected_gripper_t_target, solved_target)
    print(
        f"initializer={method}, optimizer={result.success}, "
        f"camera_error={camera_error}, target_error={target_error}"
    )
    if camera_error[0] > 1e-4 or camera_error[1] > 1e-5:
        raise SystemExit("base_T_camera synthetic recovery failed")
    if target_error[0] > 1e-4 or target_error[1] > 1e-5:
        raise SystemExit("gripper_T_target synthetic recovery failed")


if __name__ == "__main__":
    main()

