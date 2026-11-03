from __future__ import annotations

import math
import statistics
from collections.abc import Iterable


def normalized_bbox_to_pixels(
    center_x: float,
    center_y: float,
    size_x: float,
    size_y: float,
    image_width: int,
    image_height: int,
) -> tuple[int, int, int, int, int, int]:
    """Convert a normalized vision_msgs box to a clamped pixel box and centre."""
    if image_width <= 0 or image_height <= 0:
        raise ValueError("image dimensions must be positive")
    values = (center_x, center_y, size_x, size_y)
    if any(not math.isfinite(float(value)) for value in values):
        raise ValueError("bounding box contains NaN/Inf")

    # The existing YOLO node publishes normalized values. Accept pixel-valued
    # boxes as well so the bridge remains compatible with standard ROS nodes.
    normalized = max(abs(center_x), abs(center_y), abs(size_x), abs(size_y)) <= 2.0
    if normalized:
        cx = float(center_x) * image_width
        cy = float(center_y) * image_height
        width = float(size_x) * image_width
        height = float(size_y) * image_height
    else:
        cx, cy, width, height = map(float, values)

    x1 = max(0, min(image_width - 1, int(round(cx - width / 2.0))))
    y1 = max(0, min(image_height - 1, int(round(cy - height / 2.0))))
    x2 = max(x1 + 1, min(image_width, int(round(cx + width / 2.0))))
    y2 = max(y1 + 1, min(image_height, int(round(cy + height / 2.0))))
    center_px = max(0, min(image_width - 1, int(round(cx))))
    center_py = max(0, min(image_height - 1, int(round(cy))))
    return x1, y1, x2, y2, center_px, center_py


def bearing_from_pixel(pixel_x: float, focal_x: float, principal_x: float) -> float:
    if not all(math.isfinite(float(v)) for v in (pixel_x, focal_x, principal_x)):
        raise ValueError("camera projection values contain NaN/Inf")
    if focal_x <= 0:
        raise ValueError("focal_x must be positive")
    return math.atan2(float(pixel_x) - float(principal_x), float(focal_x))


def robust_depth_m(
    raw_values: Iterable[float],
    unit_scale_m: float,
    minimum_m: float = 0.08,
    maximum_m: float = 8.0,
) -> float:
    """Return a median depth after rejecting invalid and out-of-range pixels."""
    if unit_scale_m <= 0:
        raise ValueError("unit_scale_m must be positive")
    values = []
    for raw in raw_values:
        value_m = float(raw) * unit_scale_m
        if math.isfinite(value_m) and minimum_m <= value_m <= maximum_m:
            values.append(value_m)
    if not values:
        raise ValueError("depth ROI contains no valid pixels")
    return float(statistics.median(values))
