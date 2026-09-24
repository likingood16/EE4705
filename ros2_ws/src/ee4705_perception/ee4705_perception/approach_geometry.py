"""Image geometry calculations for the Task 4 approach controller."""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class TargetMeasurement:
    """Position and apparent size of a grounded object in an image."""

    bbox_pixels: tuple[float, float, float, float]
    center_x_pixels: float
    center_y_pixels: float
    horizontal_error: float
    width_fraction: float
    height_fraction: float


def normalized_bbox_to_pixels(
    bbox: tuple[float, float, float, float],
    image_width: int,
    image_height: int,
) -> tuple[float, float, float, float]:
    """Convert a 0-1000 bounding box into camera-image pixels."""

    if image_width <= 0 or image_height <= 0:
        raise ValueError("Image width and height must be positive.")

    x1, y1, x2, y2 = bbox

    return (
        x1 / 1000.0 * image_width,
        y1 / 1000.0 * image_height,
        x2 / 1000.0 * image_width,
        y2 / 1000.0 * image_height,
    )


def measure_target(
    bbox: tuple[float, float, float, float],
    image_width: int,
    image_height: int,
) -> TargetMeasurement:
    """Calculate target centre, steering error, and apparent size."""

    x1, y1, x2, y2 = normalized_bbox_to_pixels(
        bbox,
        image_width,
        image_height,
    )

    center_x = (x1 + x2) / 2.0
    center_y = (y1 + y2) / 2.0

    image_center_x = image_width / 2.0
    horizontal_error = (center_x - image_center_x) / image_center_x

    width_fraction = (x2 - x1) / image_width
    height_fraction = (y2 - y1) / image_height

    return TargetMeasurement(
        bbox_pixels=(x1, y1, x2, y2),
        center_x_pixels=center_x,
        center_y_pixels=center_y,
        horizontal_error=horizontal_error,
        width_fraction=width_fraction,
        height_fraction=height_fraction,
    )

# TurtleBot3 Waffle Pi mounting, measured from base_footprint in the running
# simulation (TF and the Gazebo model SDF agree). Camera pitch is zero.
CAMERA_HFOV_RAD = 1.085595
CAMERA_X_M = 0.076
CAMERA_HEIGHT_M = 0.103
SCAN_X_M = -0.064
ROBOT_FRONT_X_M = 0.0685


def focal_length_pixels(image_width: int, hfov_rad: float = CAMERA_HFOV_RAD) -> float:
    """Pinhole focal length in pixels for a camera with square pixels."""

    return (image_width / 2.0) / math.tan(hfov_rad / 2.0)


def pixel_bearing(
    x_pixels: float,
    image_width: int,
    hfov_rad: float = CAMERA_HFOV_RAD,
) -> float:
    """Horizontal angle to an image column; positive is to the robot's left."""

    focal = focal_length_pixels(image_width, hfov_rad)
    return math.atan((image_width / 2.0 - x_pixels) / focal)


def ground_distance(
    y_pixels: float,
    image_width: int,
    image_height: int,
    camera_height_m: float = CAMERA_HEIGHT_M,
    hfov_rad: float = CAMERA_HFOV_RAD,
) -> float | None:
    """Forward distance from the camera to a floor point seen at image row y.

    The bottom edge of a bounding box is where the object meets the floor, so
    this estimates the object's distance even when the laser passes over it.
    Rows at or above the horizon give None (the floor point is at infinity).
    """

    below_horizon = y_pixels - image_height / 2.0
    if below_horizon <= 2.0:
        return None
    return camera_height_m * focal_length_pixels(image_width, hfov_rad) / below_horizon


def camera_to_scan(distance_m: float, bearing_rad: float) -> tuple[float, float]:
    """Convert a camera-relative (distance, bearing) into laser (range, bearing)."""

    x = distance_m * math.cos(bearing_rad) + (CAMERA_X_M - SCAN_X_M)
    y = distance_m * math.sin(bearing_rad)
    return math.hypot(x, y), math.atan2(y, x)
