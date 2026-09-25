"""High-level approach decisions for Task 4."""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum


class ApproachAction(str, Enum):
    """Possible actions selected by the approach policy."""

    SEARCH = "search"
    TURN_LEFT = "turn_left"
    TURN_RIGHT = "turn_right"
    MOVE_FORWARD = "move_forward"
    STOP_TARGET_REACHED = "stop_target_reached"
    STOP_OBSTACLE = "stop_obstacle"
    STOP_SENSOR_UNAVAILABLE = "stop_sensor_unavailable"


@dataclass(frozen=True)
class ApproachConfig:
    """Thresholds used by the object-approach policy."""

    center_tolerance: float = 0.15

    # Keep this for visual scale checking,
    # but final success also requires LiDAR confirmation.
    close_height_fraction: float = 0.55

    # Hard safety stop.
    minimum_front_distance_m: float = 0.35

    # Desired stopping distance for a reached target.
    target_reached_distance_m: float = 0.80

    def __post_init__(self):

        if not math.isfinite(self.center_tolerance):
            raise ValueError(
                "Center tolerance must be finite."
            )

        if not 0 < self.center_tolerance < 1:
            raise ValueError(
                "Center tolerance must be between 0 and 1."
            )

        if not math.isfinite(
            self.close_height_fraction
        ):
            raise ValueError(
                "Close height fraction must be finite."
            )

        if not (
            0
            < self.close_height_fraction
            <= 1
        ):
            raise ValueError(
                "Close height fraction must be in (0, 1]."
            )

        if not math.isfinite(
            self.minimum_front_distance_m
        ):
            raise ValueError(
                "Minimum front distance must be finite."
            )

        if (
            self.minimum_front_distance_m
            <= 0
        ):
            raise ValueError(
                "Minimum front distance must be positive."
            )

        if not math.isfinite(
            self.target_reached_distance_m
        ):
            raise ValueError(
                "Target reached distance must be finite."
            )

        if (
            self.target_reached_distance_m
            <= self.minimum_front_distance_m
        ):
            raise ValueError(
                "Target reached distance must be greater "
                "than the minimum safety distance."
            )


def decide_approach_action(
    *,
    target_found: bool,
    horizontal_error: float | None,
    height_fraction: float | None,
    front_distance_m: float | None,
    config: ApproachConfig = ApproachConfig(),
) -> ApproachAction:
    """
    Choose the next approach action.

    Logic:
    - no fresh LiDAR -> stop
    - obstacle too close -> stop
    - target not visible -> search
    - target off-centre -> rotate
    - target centred and within target distance -> reached
    - target centred but still far -> move forward
    """

    # ========================================================
    # LIDAR SAFETY
    # ========================================================

    if front_distance_m is None:

        return (
            ApproachAction
            .STOP_SENSOR_UNAVAILABLE
        )

    if (
        not math.isfinite(
            front_distance_m
        )
        or front_distance_m < 0
    ):

        return (
            ApproachAction
            .STOP_SENSOR_UNAVAILABLE
        )

    # Hard safety stop.
    if (
        front_distance_m
        <= config.minimum_front_distance_m
    ):

        return (
            ApproachAction
            .STOP_OBSTACLE
        )

    # ========================================================
    # TARGET NOT VISIBLE
    # ========================================================

    if not target_found:

        return (
            ApproachAction
            .SEARCH
        )

    # ========================================================
    # VALIDATE CAMERA MEASUREMENTS
    # ========================================================

    if (
        horizontal_error is None
        or height_fraction is None
    ):

        raise ValueError(
            "A visible target requires horizontal "
            "error and height fraction."
        )

    if not (
        -1.0
        <= horizontal_error
        <= 1.0
    ):

        raise ValueError(
            "Horizontal error must be between -1 and 1."
        )

    if not (
        0.0
        < height_fraction
        <= 1.0
    ):

        raise ValueError(
            "Height fraction must be in (0, 1]."
        )

    # ========================================================
    # ALIGN TARGET
    # ========================================================

    if (
        horizontal_error
        < -config.center_tolerance
    ):

        return (
            ApproachAction
            .TURN_LEFT
        )

    if (
        horizontal_error
        > config.center_tolerance
    ):

        return (
            ApproachAction
            .TURN_RIGHT
        )

    # ========================================================
    # TARGET IS CENTRED
    # ========================================================

    # Use LiDAR as the final distance criterion.
    # This prevents tall/large objects such as humanoids
    # from being declared "reached" while still far away.
    if (
        front_distance_m
        <= config.target_reached_distance_m
    ):

        return (
            ApproachAction
            .STOP_TARGET_REACHED
        )

    # Target is centred but still too far away.
    return (
        ApproachAction
        .MOVE_FORWARD
    )
