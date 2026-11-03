#!/usr/bin/env python3
"""Reverse selected checkerboard corner lists and redraw their debug images."""
from __future__ import annotations

import argparse
import json
import shutil
from pathlib import Path

import cv2
import numpy as np


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input-dir", required=True)
    parser.add_argument("--sample-ids", required=True, nargs="+", type=int)
    parser.add_argument("--backup-name", default="direction_backup_before_fix")
    args = parser.parse_args()

    root = Path(args.input_dir)
    manifest_path = root / "samples.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    selected = set(args.sample_ids)
    backup = root / args.backup_name
    backup.mkdir(parents=True, exist_ok=True)
    shutil.copy2(manifest_path, backup / "samples.json")
    result_path = root / "result.json"
    if result_path.exists():
        shutil.copy2(result_path, backup / "result.json")

    cols = int(manifest["board"]["inner_corners_cols"])
    rows = int(manifest["board"]["inner_corners_rows"])
    changed = []
    for sample in manifest["samples"]:
        sample_id = int(sample["id"])
        if sample_id not in selected:
            continue
        corners = np.asarray(sample["corners_px"], dtype=np.float32).reshape(-1, 2)
        if len(corners) != cols * rows:
            raise RuntimeError(f"sample {sample_id}: unexpected corner count {len(corners)}")
        sample["corners_px"] = corners[::-1].tolist()
        image_path = root / sample["image"]
        debug_path = root / sample["debug_image"]
        shutil.copy2(debug_path, backup / debug_path.name)
        image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if image is None:
            raise RuntimeError(f"sample {sample_id}: cannot read {image_path}")
        corrected = np.asarray(sample["corners_px"], dtype=np.float32).reshape(-1, 1, 2)
        cv2.drawChessboardCorners(image, (cols, rows), corrected, True)
        if not cv2.imwrite(str(debug_path), image):
            raise RuntimeError(f"sample {sample_id}: cannot write {debug_path}")
        changed.append(sample_id)

    missing = sorted(selected - set(changed))
    if missing:
        raise RuntimeError(f"sample IDs not found: {missing}")
    temporary = manifest_path.with_suffix(".json.tmp")
    temporary.write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8")
    temporary.replace(manifest_path)
    print(json.dumps({"ok": True, "reversed_sample_ids": changed,
                      "backup": str(backup)}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
