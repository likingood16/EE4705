"""ROS-independent safety helpers for the simulated Task 4 controller.

Prepared with AI assistance for Student C; student review and attribution required.
"""

from __future__ import annotations

import math
import queue
import threading
from dataclasses import dataclass


class ApproachAbort(RuntimeError):
    """A controlled stop with a reportable reason."""


@dataclass(frozen=True)
class ApproachOutcome:
    reason: str
    api_calls: int
    elapsed_s: float
    search_rotation_deg: float
    visual_candidate: bool = False
    target: str = "object"
    initially_visible: bool | None = None
    final_range_m: float | None = None
    evidence_dir: str = ""
    input_tokens: int = 0
    output_tokens: int = 0

    @property
    def reply(self):
        if self.visual_candidate:
            return f"I am now next to the {self.target}."
        if self.reason == "observation_only":
            return f"I checked the camera for the {self.target} without moving."
        if self.reason == "object_not_found_search_limit":
            return (f"Sorry, I could not find the {self.target} "
                    "after turning a full circle.")
        if self.reason in ("obstacle", "path_blocked", "obstacle_during_turn"):
            return (f"Sorry, I stopped because something is blocking "
                    f"the way to the {self.target}.")
        return (f"Sorry, I could not reach the {self.target} "
                f"({self.reason.replace('_', ' ')}).")


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def fresh(received_at, now, max_age):
    return received_at is not None and 0 <= now - received_at <= max_age


def yaw_from_quaternion(x, y, z, w):
    values = (x, y, z, w)
    if not all(math.isfinite(v) for v in values):
        raise ValueError("Non-finite orientation")
    norm = math.sqrt(sum(v * v for v in values))
    if norm < 1e-9:
        raise ValueError("Zero quaternion")
    x, y, z, w = (v / norm for v in values)
    return math.atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z))


def scan_clearance(ranges, angle_min, angle_increment, range_min, range_max,
                   *, all_around=False, inf_is_clear=False):
    """Conservative clearance; angle zero must be robot-forward.

    Unknown rays in the required sector invalidate it. Positive infinity is
    accepted as no return ONLY after the operator verifies the simulator's
    convention and explicitly enables inf_is_clear.
    """
    if not ranges or not all(math.isfinite(v) for v in (
        angle_min, angle_increment, range_min, range_max
    )) or angle_increment == 0 or not 0 <= range_min < range_max:
        return None
    if all_around and len(ranges) * abs(angle_increment) < 2*math.pi - 0.05:
        return None
    selected = []
    angles = []
    for i, value in enumerate(ranges):
        angle = math.atan2(math.sin(angle_min + i*angle_increment),
                           math.cos(angle_min + i*angle_increment))
        if not all_around and abs(angle) > math.radians(30):
            continue
        angles.append(angle)
        if value == math.inf and inf_is_clear:
            selected.append(range_max)
        elif not math.isfinite(value) or value < 0 or value > range_max:
            return None
        elif value < range_min:
            selected.append(0.0)  # Treat a too-near return as blocked.
        else:
            selected.append(value)
    if not selected:
        return None
    if not all_around and (min(angles) > -math.radians(25)
                          or max(angles) < math.radians(25)):
        return None
    return min(selected)


def _scan_rays(ranges, angle_min, angle_increment, range_min, range_max, inf_is_clear):
    """Yield (angle, range) per ray; None ranges mark invalid rays.

    With inf_is_clear, +inf (no return within range_max) becomes math.inf so
    callers can tell "nothing there" from a real return.
    """
    for i, value in enumerate(ranges):
        angle = math.atan2(math.sin(angle_min + i*angle_increment),
                           math.cos(angle_min + i*angle_increment))
        if value == math.inf and inf_is_clear:
            yield angle, math.inf
        elif not math.isfinite(value) or value < 0 or value > range_max:
            yield angle, None
        elif value < range_min:
            yield angle, 0.0  # A too-near return counts as blocked.
        else:
            yield angle, value


def _scan_valid(ranges, angle_min, angle_increment, range_min, range_max):
    return bool(ranges) and all(math.isfinite(v) for v in (
        angle_min, angle_increment, range_min, range_max
    )) and angle_increment != 0 and 0 <= range_min < range_max


def range_at_bearing(ranges, angle_min, angle_increment, range_min, range_max,
                     *, bearing, half_window, inf_is_clear=False):
    """Nearest return within +/- half_window of a bearing (laser frame).

    Returns math.inf when every ray in the window is clear, and None when the
    window contains an invalid ray or no ray at all.
    """
    if not _scan_valid(ranges, angle_min, angle_increment, range_min, range_max):
        return None
    nearest = None
    for angle, value in _scan_rays(ranges, angle_min, angle_increment,
                                   range_min, range_max, inf_is_clear):
        offset = math.atan2(math.sin(angle - bearing), math.cos(angle - bearing))
        if abs(offset) > half_window:
            continue
        if value is None:
            return None
        nearest = value if nearest is None else min(nearest, value)
    return nearest


def corridor_clearance(ranges, angle_min, angle_increment, range_min, range_max,
                       *, half_width, inf_is_clear=False):
    """Forward distance (laser frame) to the nearest return in the robot's path.

    The path is the strip |y| <= half_width ahead of the laser. Unlike a fixed
    angular sector, this ignores side walls the robot will pass, and still
    catches a narrow object straight ahead. Returns math.inf when the strip is
    clear and None when an invalid ray could hide an obstacle in it.
    """
    if not _scan_valid(ranges, angle_min, angle_increment, range_min, range_max):
        return None
    nearest = math.inf
    for angle, value in _scan_rays(ranges, angle_min, angle_increment,
                                   range_min, range_max, inf_is_clear):
        if abs(angle) >= math.pi / 2:
            continue
        if value is None:
            return None
        if value == math.inf:
            continue
        if abs(value * math.sin(angle)) <= half_width:
            nearest = min(nearest, value * math.cos(angle))
    return nearest


def sweep_clearance(ranges, angle_min, angle_increment, range_min, range_max,
                    *, scan_x, inf_is_clear=False):
    """Nearest return measured from the robot's centre, for turning in place.

    The laser sits scan_x metres ahead of the rotation centre (negative when
    behind it), so a fixed range threshold around the laser would be too
    strict on one side and too loose on the other. Requires full 360 degree
    coverage; returns None when a ray is invalid.
    """
    if not _scan_valid(ranges, angle_min, angle_increment, range_min, range_max):
        return None
    if len(ranges) * abs(angle_increment) < 2*math.pi - 0.05:
        return None
    nearest = math.inf
    for angle, value in _scan_rays(ranges, angle_min, angle_increment,
                                   range_min, range_max, inf_is_clear):
        if value is None:
            return None
        if value == math.inf:
            continue
        nearest = min(nearest, math.hypot(scan_x + value*math.cos(angle),
                                          value*math.sin(angle)))
    return nearest


def wait_for_grounding(call, *, poll, stop, now, timeout_s):
    """Keep ROS responsive while one daemon worker performs a network call.

    The worker never accesses ROS or velocities. A cancelled/timed-out result
    cannot be consumed by a later call. Provider-side work may still complete
    and be billed; cancellation does not cancel the HTTP request.
    """
    results = queue.Queue(maxsize=1)

    def worker():
        try:
            results.put((True, call()))
        except Exception as error:
            results.put((False, error))

    stop()
    deadline = now() + timeout_s
    threading.Thread(target=worker, daemon=True).start()
    while True:
        stop()
        poll()  # Must check cancellation and total timeout.
        if now() >= deadline:
            raise ApproachAbort("vlm_timeout")
        try:
            ok, value = results.get_nowait()
        except queue.Empty:
            continue
        if not ok:
            raise ApproachAbort("grounding_failed") from value
        return value
