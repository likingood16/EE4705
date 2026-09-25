# EE4705 Project 1.2 | Task 4 - object approach
# Contributors (from git history): Marie (1 commit)
"""Convert approach decisions into bounded speed requests."""

from __future__ import annotations

import math
from dataclasses import dataclass

from .approach_policy import ApproachAction


@dataclass(frozen=True)
class VelocityRequest:
    """Forward speed in m/s and turning speed in rad/s."""

    linear_x: float = 0.0
    angular_z: float = 0.0


@dataclass(frozen=True)
class VelocityConfig:
    """Conservative initial speeds to verify in simulation."""

    forward_speed_mps: float = 0.08
    turn_speed_radps: float = 0.25
    search_speed_radps: float = 0.20

    def __post_init__(self):
        # Hard ceilings for this initial project controller.
        limits = (
            ("forward_speed_mps", self.forward_speed_mps, 0.12),
            ("turn_speed_radps", self.turn_speed_radps, 0.40),
            ("search_speed_radps", self.search_speed_radps, 0.40),
        )

        for name, value, maximum in limits:
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError(f"{name} must be numeric.")
            if not math.isfinite(value) or not 0 < value <= maximum:
                raise ValueError(
                    f"{name} must be positive and no greater than {maximum}."
                )


def velocity_for_action(
    action: ApproachAction,
    *,
    motion_permitted: bool = False,
    config: VelocityConfig = VelocityConfig(),
) -> VelocityRequest:
    """Return zero speed unless motion is explicitly permitted."""

    if not isinstance(action, ApproachAction):
        raise ValueError("action must be an ApproachAction.")

    if not isinstance(motion_permitted, bool):
        raise ValueError("motion_permitted must be a boolean.")

    if not motion_permitted:
        return VelocityRequest()

    if action == ApproachAction.MOVE_FORWARD:
        return VelocityRequest(linear_x=config.forward_speed_mps)

    if action == ApproachAction.TURN_LEFT:
        return VelocityRequest(angular_z=config.turn_speed_radps)

    if action == ApproachAction.TURN_RIGHT:
        return VelocityRequest(angular_z=-config.turn_speed_radps)

    if action == ApproachAction.SEARCH:
        return VelocityRequest(angular_z=config.search_speed_radps)

    # Every STOP action produces zero linear and angular speed.
    return VelocityRequest()