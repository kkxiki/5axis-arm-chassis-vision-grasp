#!/usr/bin/env python3
"""Shared geometry helpers for eye-to-hand calibration."""

from __future__ import annotations

import math
from typing import Iterable, Tuple

import cv2
import numpy as np
from scipy.spatial.transform import Rotation


def make_transform(rotation: np.ndarray, translation: Iterable[float]) -> np.ndarray:
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    transform[:3, 3] = np.asarray(translation, dtype=np.float64).reshape(3)
    return transform


def invert_transform(transform: np.ndarray) -> np.ndarray:
    transform = np.asarray(transform, dtype=np.float64).reshape(4, 4)
    inverse = np.eye(4, dtype=np.float64)
    inverse[:3, :3] = transform[:3, :3].T
    inverse[:3, 3] = -inverse[:3, :3] @ transform[:3, 3]
    return inverse


def pose_xyzw_to_transform(pose: Iterable[float], translation_scale: float) -> np.ndarray:
    values = np.asarray(list(pose), dtype=np.float64)
    if values.shape != (7,) or not np.isfinite(values).all():
        raise ValueError("pose must contain seven finite values: x,y,z,qx,qy,qz,qw")
    quaternion = values[3:7]
    norm = np.linalg.norm(quaternion)
    if norm < 1e-9:
        raise ValueError("pose quaternion has zero length")
    rotation = Rotation.from_quat(quaternion / norm).as_matrix()
    return make_transform(rotation, values[:3] * float(translation_scale))


def transform_to_params(transform: np.ndarray) -> np.ndarray:
    transform = np.asarray(transform, dtype=np.float64).reshape(4, 4)
    return np.concatenate(
        [Rotation.from_matrix(transform[:3, :3]).as_rotvec(), transform[:3, 3]]
    )


def params_to_transform(params: Iterable[float]) -> np.ndarray:
    values = np.asarray(list(params), dtype=np.float64).reshape(6)
    return make_transform(Rotation.from_rotvec(values[:3]).as_matrix(), values[3:])


def board_object_points(cols: int, rows: int, square_size_m: float) -> np.ndarray:
    if cols < 2 or rows < 2 or square_size_m <= 0:
        raise ValueError("invalid chessboard geometry")
    points = np.zeros((cols * rows, 3), dtype=np.float32)
    points[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    points *= float(square_size_m)
    return points


def find_board(
    image_bgr: np.ndarray,
    cols: int,
    rows: int,
    *,
    accurate: bool = True,
) -> Tuple[bool, np.ndarray | None]:
    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    pattern = (int(cols), int(rows))

    if accurate and hasattr(cv2, "findChessboardCornersSB"):
        flags = cv2.CALIB_CB_NORMALIZE_IMAGE
        flags |= getattr(cv2, "CALIB_CB_EXHAUSTIVE", 0)
        flags |= getattr(cv2, "CALIB_CB_ACCURACY", 0)
        found, corners = cv2.findChessboardCornersSB(gray, pattern, flags=flags)
        if found:
            return True, np.asarray(corners, dtype=np.float32)

    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    found, corners = cv2.findChessboardCorners(gray, pattern, flags)
    if not found:
        return False, None
    criteria = (
        cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
        40,
        0.001,
    )
    corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
    return True, corners


def solve_board_pose(
    corners: np.ndarray,
    cols: int,
    rows: int,
    square_size_m: float,
    camera_matrix: np.ndarray,
    distortion: np.ndarray,
) -> Tuple[np.ndarray, float]:
    object_points = board_object_points(cols, rows, square_size_m)
    camera_matrix = np.asarray(camera_matrix, dtype=np.float64).reshape(3, 3)
    distortion = np.asarray(distortion, dtype=np.float64).reshape(-1, 1)
    success, rvec, tvec = cv2.solvePnP(
        object_points,
        np.asarray(corners, dtype=np.float32),
        camera_matrix,
        distortion,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    if not success:
        raise RuntimeError("cv2.solvePnP failed")
    rotation, _ = cv2.Rodrigues(rvec)
    camera_t_target = make_transform(rotation, tvec.reshape(3))
    projected, _ = cv2.projectPoints(
        object_points, rvec, tvec, camera_matrix, distortion
    )
    delta = projected.reshape(-1, 2) - np.asarray(corners).reshape(-1, 2)
    reprojection_rms = float(np.sqrt(np.mean(np.sum(delta * delta, axis=1))))
    return camera_t_target, reprojection_rms


def transform_error(lhs: np.ndarray, rhs: np.ndarray) -> Tuple[float, float]:
    error = invert_transform(lhs) @ rhs
    translation_mm = float(np.linalg.norm(error[:3, 3]) * 1000.0)
    rotation_deg = float(
        np.linalg.norm(Rotation.from_matrix(error[:3, :3]).as_rotvec())
        * 180.0
        / math.pi
    )
    return translation_mm, rotation_deg

