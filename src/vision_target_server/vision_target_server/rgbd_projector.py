"""RGB-D depth validation and pinhole projection helpers."""

from dataclasses import dataclass

import numpy as np
import cv2


@dataclass(frozen=True)
class DepthSample:
    """Validated depth sample around one image pixel."""

    depth_m: float
    mad_m: float
    valid_ratio: float


def depth_to_meters(depth: np.ndarray, uint16_scale_m: float = 0.001) -> np.ndarray:
    """Convert an Orbbec depth image to meters."""
    if depth.dtype == np.uint16:
        return depth.astype(np.float32) * float(uint16_scale_m)
    if depth.dtype in (np.float32, np.float64):
        return depth.astype(np.float32)
    raise ValueError(f'Unsupported depth dtype: {depth.dtype}')


def sample_depth(
    depth_m: np.ndarray,
    u: int,
    v: int,
    radius_px: int,
    min_depth_m: float,
    max_depth_m: float,
    max_mad_m: float,
    min_valid_samples: int = 5,
    min_valid_ratio: float = 0.0,
) -> DepthSample | None:
    """Sample robust median depth in a square ROI around (u, v)."""
    if depth_m.ndim != 2:
        raise ValueError('Depth image must be single-channel')

    height, width = depth_m.shape
    if u < 0 or u >= width or v < 0 or v >= height:
        return None

    radius = max(0, int(radius_px))
    x0, x1 = max(0, u - radius), min(width, u + radius + 1)
    y0, y1 = max(0, v - radius), min(height, v + radius + 1)
    roi = depth_m[y0:y1, x0:x1]
    if roi.size == 0:
        return None

    mask = np.isfinite(roi) & (roi >= min_depth_m) & (roi <= max_depth_m)
    valid = roi[mask]
    valid_ratio = float(valid.size) / float(roi.size)
    if valid.size < int(min_valid_samples) or valid_ratio < float(min_valid_ratio):
        return None

    depth = float(np.median(valid))
    mad = float(np.median(np.abs(valid - depth)))
    if mad > float(max_mad_m):
        return None

    return DepthSample(depth_m=depth, mad_m=mad, valid_ratio=valid_ratio)


def project_pixel(u: int, v: int, depth_m: float, camera_info) -> tuple[float, float, float]:
    """Project one pixel/depth sample into the camera optical frame."""
    matrix = np.asarray(camera_info.k, dtype=np.float64).reshape(3, 3)
    distortion = np.asarray(camera_info.d, dtype=np.float64)
    if (not np.isfinite(matrix).all() or not np.isfinite(distortion).all()
            or matrix[0, 0] <= 0 or matrix[1, 1] <= 0
            or not np.isfinite(depth_m) or depth_m <= 0):
        raise ValueError('Invalid camera intrinsics or depth')
    pixel = np.array([[[float(u), float(v)]]], dtype=np.float64)
    if distortion.size and np.any(distortion):
        if camera_info.distortion_model in ('plumb_bob', 'rational_polynomial'):
            ray = cv2.undistortPoints(pixel, matrix, distortion).reshape(2)
        elif camera_info.distortion_model == 'equidistant' and distortion.size == 4:
            ray = cv2.fisheye.undistortPoints(pixel, matrix, distortion).reshape(2)
        else:
            raise ValueError('Unsupported camera distortion model')
    else:
        ray = np.array([(u - matrix[0, 2]) / matrix[0, 0],
                        (v - matrix[1, 2]) / matrix[1, 1]])
    if not np.isfinite(ray).all():
        raise ValueError('Invalid projected camera ray')
    return float(ray[0] * depth_m), float(ray[1] * depth_m), float(depth_m)
