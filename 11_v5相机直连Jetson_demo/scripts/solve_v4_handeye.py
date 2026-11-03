#!/usr/bin/env python3
from __future__ import annotations

import argparse
import itertools
import json
from pathlib import Path

import cv2
import numpy as np


def quat_to_rot(q):
    x, y, z, w = np.asarray(q, dtype=np.float64)
    n = np.linalg.norm([x, y, z, w])
    x, y, z, w = np.asarray([x, y, z, w]) / n
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ], dtype=np.float64)


def transform(r, t):
    out = np.eye(4, dtype=np.float64)
    out[:3, :3] = r
    out[:3, 3] = np.asarray(t).reshape(3)
    return out


def rotation_angle_deg(r):
    return float(np.degrees(np.arccos(np.clip((np.trace(r)-1.0)/2.0, -1.0, 1.0))))


def consistency(base_t_targets):
    translations = np.array([t[:3, 3] for t in base_t_targets])
    center = np.mean(translations, axis=0)
    terr = np.linalg.norm(translations-center, axis=1) * 1000.0
    rerr = np.array([rotation_angle_deg(base_t_targets[i][:3, :3].T @ base_t_targets[j][:3, :3])
                     for i in range(len(base_t_targets)) for j in range(i)])
    return {
        "translation_rms_mm": float(np.sqrt(np.mean(terr**2))),
        "translation_max_mm": float(np.max(terr)),
        "rotation_rms_deg": float(np.sqrt(np.mean(rerr**2))),
        "rotation_max_deg": float(np.max(rerr)),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--input-dir", default="/home/nvidia/5axis_v4/calibration/wrist_v4")
    ap.add_argument("--output", default="/home/nvidia/5axis_v4/calibration/wrist_v4/result.json")
    args = ap.parse_args()
    root = Path(args.input_dir)
    manifest = json.loads((root / "samples.json").read_text())
    board = manifest["board"]
    cols, rows = board["inner_corners_cols"], board["inner_corners_rows"]
    square = board["square_size_mm"] / 1000.0
    object_points = np.zeros((cols*rows, 3), np.float32)
    object_points[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square

    obj, img, used = [], [], []
    image_size = None
    for sample in manifest["samples"]:
        frame = cv2.imread(str(root / sample["image"]))
        if frame is None:
            continue
        image_size = (frame.shape[1], frame.shape[0])
        corners = np.asarray(sample["corners_px"], np.float32).reshape(-1, 1, 2)
        if len(corners) != cols*rows:
            continue
        obj.append(object_points.copy())
        img.append(corners)
        used.append(sample)
    if len(used) < 8:
        raise RuntimeError("at least 8 valid samples are required")

    rms, k, dist, rvecs, tvecs = cv2.calibrateCamera(obj, img, image_size, None, None)
    per_view = []
    for op, ip, rv, tv in zip(obj, img, rvecs, tvecs):
        projected, _ = cv2.projectPoints(op, rv, tv, k, dist)
        per_view.append(float(np.sqrt(np.mean(np.sum((ip-projected)**2, axis=2)))))

    rg2b, tg2b, pose_candidates = [], [], []
    for sample, corners in zip(used, img):
        pose = np.asarray(sample["robot_pose"]["raw"], np.float64)
        rg2b.append(quat_to_rot(pose[3:]))
        tg2b.append(pose[:3].reshape(3, 1))
        pair = []
        for ordered in (corners, corners[::-1].copy()):
            ok, rv, tv = cv2.solvePnP(object_points, ordered, k, dist,
                                      flags=cv2.SOLVEPNP_ITERATIVE)
            if not ok:
                raise RuntimeError("solvePnP failed")
            pair.append((cv2.Rodrigues(rv)[0], np.asarray(tv).reshape(3, 1)))
        pose_candidates.append(pair)

    # A 5x7 checkerboard is invariant under 180-degree rotation.  Resolve the
    # per-image corner-order ambiguity by minimizing fixed-target consistency.
    best_flip = None
    for tail in itertools.product((0, 1), repeat=len(used)-1):
        flips = (0,) + tail
        rt = [pose_candidates[i][flips[i]][0] for i in range(len(used))]
        tt = [pose_candidates[i][flips[i]][1] for i in range(len(used))]
        try:
            rc2g, tc2g = cv2.calibrateHandEye(
                rg2b, tg2b, rt, tt, method=cv2.CALIB_HAND_EYE_TSAI)
        except cv2.error:
            continue
        gtc = transform(rc2g, tc2g)
        bt = [transform(a, b) @ gtc @ transform(c, d)
              for a, b, c, d in zip(rg2b, tg2b, rt, tt)]
        metric = consistency(bt)
        score = metric["translation_rms_mm"] + 2.0 * metric["rotation_rms_deg"]
        if best_flip is None or score < best_flip[0]:
            best_flip = (score, flips)
    if best_flip is None:
        raise RuntimeError("checkerboard orientation resolution failed")
    flips = best_flip[1]
    rt2c = [pose_candidates[i][flips[i]][0] for i in range(len(used))]
    tt2c = [pose_candidates[i][flips[i]][1] for i in range(len(used))]

    methods = {
        "TSAI": cv2.CALIB_HAND_EYE_TSAI,
        "PARK": cv2.CALIB_HAND_EYE_PARK,
        "HORAUD": cv2.CALIB_HAND_EYE_HORAUD,
        "ANDREFF": cv2.CALIB_HAND_EYE_ANDREFF,
        "DANIILIDIS": cv2.CALIB_HAND_EYE_DANIILIDIS,
    }
    solutions = {}
    for name, method in methods.items():
        try:
            rc2g, tc2g = cv2.calibrateHandEye(rg2b, tg2b, rt2c, tt2c, method=method)
            gtc = transform(rc2g, tc2g)
            bt = [transform(a, b) @ gtc @ transform(c, d)
                  for a, b, c, d in zip(rg2b, tg2b, rt2c, tt2c)]
            metric = consistency(bt)
            metric["score"] = metric["translation_rms_mm"] + 2.0*metric["rotation_rms_deg"]
            solutions[name] = {"camera_to_tcp_matrix": gtc.tolist(), "consistency": metric}
        except cv2.error as exc:
            solutions[name] = {"error": str(exc)}
    valid = {n: v for n, v in solutions.items() if "consistency" in v and np.isfinite(v["consistency"]["score"])}
    if not valid:
        raise RuntimeError("all hand-eye methods failed")
    best = min(valid, key=lambda n: valid[n]["consistency"]["score"])
    result = {
        "schema_version": 1,
        "mode": "eye_in_hand",
        "sample_count": len(used),
        "image_size": list(image_size),
        "board_square_size_m": square,
        "camera_matrix": k.tolist(),
        "distortion_coefficients": dist.reshape(-1).tolist(),
        "intrinsic_rms_px": float(rms),
        "per_view_reprojection_rms_px": per_view,
        "reversed_corner_sample_ids": [int(used[i]["id"]) for i, value in enumerate(flips) if value],
        "selected_method": best,
        "camera_to_tcp_matrix": valid[best]["camera_to_tcp_matrix"],
        "methods": solutions,
        "pose_convention": "base_T_tcp from x,y,z,qx,qy,qz,qw; result is tcp_T_camera",
    }
    Path(args.output).write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
