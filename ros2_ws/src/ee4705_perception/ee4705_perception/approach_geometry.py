"""Image geometry calculations for the Task 4 approach controller."""

from __future__ import annotations

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