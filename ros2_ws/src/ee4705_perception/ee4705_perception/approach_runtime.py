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

    @property
    def reply(self):
        if self.visual_candidate:
            return "I stopped near the visible target. Final distance still needs verification."
        return f"Approach stopped: {self.reason.replace('_', ' ')}."


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
