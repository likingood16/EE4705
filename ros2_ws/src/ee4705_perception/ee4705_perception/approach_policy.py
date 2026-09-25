# EE4705 Project 1.2 | Task 4 - object approach
# Contributors (from git history): Marie (2 commits), Alexander Likin (1 commit), Charansagar Ramanujam (LiDAR arrival rule)
"""High-level approach decisions; ROS safety monitoring is added separately."""

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
    """Thresholds for the reactive policy used by approach_controller."""

    center_tolerance: float = 0.15
    # Kept for visual scale checks; arrival is decided by the laser.
    close_height_fraction: float = 0.55
    # Hard safety stop.
    minimum_front_distance_m: float = 0.35
    # A centred target is reached once the laser range drops to this.
    target_reached_distance_m: float = 0.80

    def __post_init__(self):
        if not math.isfinite(self.center_tolerance):
            raise ValueError("Center tolerance must be finite.")
        if not 0 < self.center_tolerance < 1:
            raise ValueError("Center tolerance must be between 0 and 1.")

        if not math.isfinite(self.close_height_fraction):
            raise ValueError("Close height fraction must be finite.")
        if not 0 < self.close_height_fraction <= 1:
            raise ValueError("Close height fraction must be in (0, 1].")

        if not math.isfinite(self.minimum_front_distance_m):
            raise ValueError("Minimum front distance must be finite.")
        if self.minimum_front_distance_m <= 0:
            raise ValueError("Minimum front distance must be positive.")

        if not math.isfinite(self.target_reached_distance_m):
            raise ValueError("Target reached distance must be finite.")
        if self.target_reached_distance_m <= self.minimum_front_distance_m:
            raise ValueError(
                "Target reached distance must exceed the minimum safety distance."
            )


def decide_approach_action(
    *,
    target_found: bool,
    horizontal_error: float | None,
    height_fraction: float | None,
    front_distance_m: float | None,
    config: ApproachConfig = ApproachConfig(),
) -> ApproachAction:
    """Choose an action from validated, fresh sensor measurements."""

    # Missing, invalid, or stale scan data must never permit movement.
    # The future ROS adapter must pass None when a scan is stale.
    if front_distance_m is None:
        return ApproachAction.STOP_SENSOR_UNAVAILABLE

    if not math.isfinite(front_distance_m) or front_distance_m < 0:
        return ApproachAction.STOP_SENSOR_UNAVAILABLE

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

    if not 0.0 < height_fraction <= 1.0:
        raise ValueError("Height fraction must be in (0, 1].")

    if horizontal_error < -config.center_tolerance:
        return ApproachAction.TURN_LEFT

    if horizontal_error > config.center_tolerance:
        return ApproachAction.TURN_RIGHT

    # The laser, not the box size, decides arrival: tall objects such as the
    # humanoid fill the frame while still far away.
    if front_distance_m <= config.target_reached_distance_m:
        return ApproachAction.STOP_TARGET_REACHED

    return ApproachAction.MOVE_FORWARD

class StepKind(str, Enum):
    """What the controller does after one grounding query."""

    ARRIVED = "arrived"
    ADVANCE = "advance"
    SEARCH = "search"


@dataclass(frozen=True)
class StepConfig:
    """Bearing-and-range approach settings for the Waffle Pi in Gazebo.

    Ranges are laser ranges; the bumper is 0.13 m ahead of the laser, so
    arrival_range_m = 0.55 leaves roughly 0.4 m between bumper and object.
    """

    arrival_range_m: float = 0.55
    arrival_bearing_rad: float = math.radians(10)
    align_tolerance_rad: float = math.radians(3)
    max_step_m: float = 1.2
    unknown_range_step_m: float = 0.6
    search_step_rad: float = math.radians(45)

    def __post_init__(self):
        values = (self.arrival_range_m, self.arrival_bearing_rad,
                  self.align_tolerance_rad, self.max_step_m,
                  self.unknown_range_step_m, self.search_step_rad)
        if not all(math.isfinite(v) and v > 0 for v in values):
            raise ValueError("Step settings must be positive and finite.")


@dataclass(frozen=True)
class ApproachStep:
    """One planned move: turn by turn_rad, then drive forward_m."""

    kind: StepKind
    turn_rad: float = 0.0
    forward_m: float = 0.0


def plan_step(
    *,
    target_found: bool,
    bearing_rad: float | None,
    target_range_m: float | None,
    config: StepConfig = StepConfig(),
) -> ApproachStep:
    """Choose the next move from one grounding result.

    target_range_m is the laser-frame range to the target, or None when it
    could not be measured (then the robot only takes a short step).
    """

    if not target_found:
        return ApproachStep(StepKind.SEARCH, turn_rad=config.search_step_rad)

    if bearing_rad is None or not math.isfinite(bearing_rad):
        raise ValueError("A visible target requires a finite bearing.")

    turn = bearing_rad if abs(bearing_rad) > config.align_tolerance_rad else 0.0

    if target_range_m is not None and target_range_m <= config.arrival_range_m + 0.05:
        if abs(bearing_rad) <= config.arrival_bearing_rad:
            return ApproachStep(StepKind.ARRIVED, turn_rad=turn)
        # Close but off-centre: turn to face it, then look again.
        return ApproachStep(StepKind.ADVANCE, turn_rad=turn)

    if target_range_m is None:
        forward = config.unknown_range_step_m
    else:
        forward = min(config.max_step_m, target_range_m - config.arrival_range_m)
    return ApproachStep(StepKind.ADVANCE, turn_rad=turn, forward_m=max(0.0, forward))
