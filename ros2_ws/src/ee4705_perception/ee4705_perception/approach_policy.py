"""Safe high-level decisions for the Task 4 object-approach controller."""

from __future__ import annotations

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


@dataclass(frozen=True)
class ApproachConfig:
    """Tunable thresholds for selecting an approach action."""

    center_tolerance: float = 0.15
    close_height_fraction: float = 0.55
    minimum_front_distance_m: float = 0.35


def decide_approach_action(
    *,
    target_found: bool,
    horizontal_error: float | None,
    height_fraction: float | None,
    front_distance_m: float | None,
    config: ApproachConfig = ApproachConfig(),
) -> ApproachAction:
    """Select one safe high-level action from perception measurements."""

    if front_distance_m is not None:
        if front_distance_m < 0:
            raise ValueError("Front distance cannot be negative.")

        if front_distance_m <= config.minimum_front_distance_m:
            return ApproachAction.STOP_OBSTACLE

    if not target_found:
        return ApproachAction.SEARCH

    if horizontal_error is None or height_fraction is None:
        raise ValueError(
            "A visible target requires horizontal error and height fraction."
        )

    if not -1.0 <= horizontal_error <= 1.0:
        raise ValueError("Horizontal error must be between -1 and 1.")

    if not 0.0 <= height_fraction <= 1.0:
        raise ValueError("Height fraction must be between 0 and 1.")

    if height_fraction >= config.close_height_fraction:
        return ApproachAction.STOP_TARGET_REACHED

    if horizontal_error < -config.center_tolerance:
        return ApproachAction.TURN_LEFT

    if horizontal_error > config.center_tolerance:
        return ApproachAction.TURN_RIGHT

    return ApproachAction.MOVE_FORWARD