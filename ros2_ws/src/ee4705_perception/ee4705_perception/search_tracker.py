"""Track rotation and time limits for a Task 4 object search."""

from __future__ import annotations

import math
from enum import Enum


class SearchStatus(str, Enum):
    SEARCHING = "searching"
    FOUND = "found"
    EXHAUSTED = "exhausted"
    TIMED_OUT = "timed_out"


class SearchTracker:
    """Track a single search using measured yaw and elapsed time."""

    def __init__(
        self,
        *,
        initial_yaw_rad: float,
        started_at_s: float,
        max_rotation_rad: float = 2 * math.pi,
        timeout_s: float = 60.0,
    ):
        values = (
            initial_yaw_rad,
            started_at_s,
            max_rotation_rad,
            timeout_s,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("Search settings must be finite.")

        if max_rotation_rad <= 0 or timeout_s <= 0:
            raise ValueError("Search limits must be positive.")

        self.previous_yaw_rad = initial_yaw_rad
        self.previous_time_s = started_at_s
        self.started_at_s = started_at_s
        self.max_rotation_rad = max_rotation_rad
        self.timeout_s = timeout_s

        self.rotation_rad = 0.0
        self.status = SearchStatus.SEARCHING

    def update(
        self,
        *,
        yaw_rad: float,
        now_s: float,
        target_found: bool,
    ) -> SearchStatus:
        """Update from fresh odometry and a current detection result."""

        if self.status != SearchStatus.SEARCHING:
            return self.status

        if not math.isfinite(yaw_rad) or not math.isfinite(now_s):
            raise ValueError("Search measurements must be finite.")

        if now_s < self.previous_time_s:
            raise ValueError("Search time cannot move backwards.")

        if not isinstance(target_found, bool):
            raise ValueError("target_found must be a boolean.")

        # Convert angle changes across +pi/-pi into small signed changes.
        delta = math.atan2(
            math.sin(yaw_rad - self.previous_yaw_rad),
            math.cos(yaw_rad - self.previous_yaw_rad),
        )

        # Count total angular travel, including direction reversals.
        self.rotation_rad += abs(delta)
        self.previous_yaw_rad = yaw_rad
        self.previous_time_s = now_s

        if now_s - self.started_at_s >= self.timeout_s:
            self.status = SearchStatus.TIMED_OUT
        elif target_found:
            self.status = SearchStatus.FOUND
        elif self.rotation_rad >= self.max_rotation_rad - 1e-9:
            self.status = SearchStatus.EXHAUSTED

        return self.status