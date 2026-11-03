"""前视相机几何换算模块。

提供三个纯数学工具函数（不做网络/文件 IO），供前视视觉服务复用：
1. normalized_bbox_to_pixels：把 YOLO 风格的归一化检测框转成像素框与中心点；
2. bearing_from_pixel：由像素横向偏移算出目标相对相机光轴的方位角；
3. robust_depth_m：对深度 ROI 做稳健取值（剔除无效值、超范围值后取中位数）。

坐标系与单位约定：像素原点在图像左上角，x 向右、y 向下；
深度与距离统一使用米（m），角度统一使用弧度（rad）。
"""
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
    """把视觉检测框换算成像素坐标下的边界框和中心点。

    参数：
        center_x/center_y：框中心的 x、y；
        size_x/size_y：框的宽、高；
        image_width/image_height：图像宽高（像素），必须为正整数。
        前四个值既可能是归一化值（0~1 比例），也可能是像素值，判定方式见下方注释。

    返回：
        (x1, y1, x2, y2, center_px, center_py)，均为整数像素坐标。
        其中 (x1, y1) 为左上角、(x2, y2) 为右下角（不包含端点）。

    边界处理：
        实际计算出的框会被截断到图像内，x2/y2 保证至少比 x1/y1 大 1 像素，
        因此即使框贴在图像边缘也不会出现零面积或越界坐标。

    异常：图像尺寸非正、或框的四个数含 NaN/Inf 时抛 ValueError。
    """
    if image_width <= 0 or image_height <= 0:
        raise ValueError("image dimensions must be positive")
    values = (center_x, center_y, size_x, size_y)
    if any(not math.isfinite(float(value)) for value in values):
        raise ValueError("bounding box contains NaN/Inf")

    # 现有 YOLO 节点发布的是归一化数值；同时兼容标准的像素值框，
    # 以免与其它 ROS 节点的输出不兼容。
    # 判定规则：四个数绝对值都不超过 2.0 视作归一化值（乘图像宽高），否则按像素值处理。
    normalized = max(abs(center_x), abs(center_y), abs(size_x), abs(size_y)) <= 2.0
    if normalized:
        cx = float(center_x) * image_width
        cy = float(center_y) * image_height
        width = float(size_x) * image_width
        height = float(size_y) * image_height
    else:
        cx, cy, width, height = map(float, values)

    # 由「中心 + 宽高」还原左上/右下角，四舍五入到整数像素并截断到图像内
    x1 = max(0, min(image_width - 1, int(round(cx - width / 2.0))))
    y1 = max(0, min(image_height - 1, int(round(cy - height / 2.0))))
    # x2/y2 下界取 x1+1/y1+1，保证框至少 1 像素宽高（避免后续 ROI 取空）
    x2 = max(x1 + 1, min(image_width, int(round(cx + width / 2.0))))
    y2 = max(y1 + 1, min(image_height, int(round(cy + height / 2.0))))
    center_px = max(0, min(image_width - 1, int(round(cx))))
    center_py = max(0, min(image_height - 1, int(round(cy))))
    return x1, y1, x2, y2, center_px, center_py


def bearing_from_pixel(pixel_x: float, focal_x: float, principal_x: float) -> float:
    """由像素横坐标算出目标相对相机光轴的方位角（即 bearing）。

    公式（针孔相机模型）：
        bearing = atan2(pixel_x - principal_x, focal_x)

    参数：
        pixel_x：目标中心的横坐标（像素）；
        focal_x：x 方向焦距 fx（像素），必须为正；
        principal_x：主点横坐标 cx（像素），即光轴落点。

    返回：方位角，单位 rad。以光轴为零点，像素在中心右侧（pixel_x > principal_x）
        时为正、在左侧时为负。纵坐标不参与计算，所以只反映水平方向的偏角。

    异常：入参含 NaN/Inf 或 focal_x <= 0 时抛 ValueError。
    """
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
    """先剔除无效像素、再取中位数，得到稳健的深度值（米）。

    参数：
        raw_values：深度 ROI 内的原始深度值（可迭代，通常是像素值）；
        unit_scale_m：原始值的单位换算系数，乘以它得到米（如 0.001 表示原始值单位是毫米）；
        minimum_m/maximum_m：有效深度范围（米），默认 0.08~8.0。

    处理流程：
        1. 每个原始值乘 unit_scale_m 换算成米；
        2. 丢弃 NaN/Inf，以及范围外的值（0 通常表示无回波，过大值通常是噪声）；
        3. 对剩下的有效值取中位数——中位数对个别离群点不敏感，比均值更稳。

    返回：有效像素深度的中位数，单位 m。

    异常：unit_scale_m <= 0、或过滤后没有任何有效像素时抛 ValueError。
    """
    if unit_scale_m <= 0:
        raise ValueError("unit_scale_m must be positive")
    values = []
    for raw in raw_values:
        value_m = float(raw) * unit_scale_m
        # 只保留有限且在 [minimum_m, maximum_m] 内的深度
        if math.isfinite(value_m) and minimum_m <= value_m <= maximum_m:
            values.append(value_m)
    if not values:
        raise ValueError("depth ROI contains no valid pixels")
    return float(statistics.median(values))
