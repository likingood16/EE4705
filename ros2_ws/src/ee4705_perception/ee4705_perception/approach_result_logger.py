"""Save Task 4 trial results without overwriting earlier trials."""

from __future__ import annotations

import csv
import math
from dataclasses import asdict, dataclass, fields
from pathlib import Path


@dataclass(frozen=True)
class ApproachTrial:
    """One trial; leave measurements as None until actually evaluated."""

    trial_id: str
    target: str
    mode: str = "mock"
    start_pose: str = ""
    model: str = ""
    image_path: str = ""
    initially_visible: bool | None = None
    grounding_correct: bool | None = None
    approach_success: bool | None = None
    final_distance_m: float | None = None
    distance_method: str = ""
    elapsed_s: float | None = None
    search_rotation_deg: float | None = None
    api_calls: int = 0
    cost_usd: float | None = None
    failure_reason: str = ""
    notes: str = ""

    def __post_init__(self):
        if not self.trial_id.strip() or not self.target.strip():
            raise ValueError("Trial ID and target cannot be empty.")

        if self.mode not in ("mock", "simulation", "real_robot"):
            raise ValueError("Invalid trial mode.")

        for name in (
            "initially_visible",
            "grounding_correct",
            "approach_success",
        ):
            value = getattr(self, name)
            if value is not None and not isinstance(value, bool):
                raise ValueError(f"{name} must be a boolean or None.")

        for name in (
            "final_distance_m",
            "elapsed_s",
            "search_rotation_deg",
            "cost_usd",
        ):
            value = getattr(self, name)
            if value is None:
                continue
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError(f"{name} must be numeric.")
            if not math.isfinite(value) or value < 0:
                raise ValueError(f"{name} must be finite and non-negative.")

        if (
            isinstance(self.api_calls, bool)
            or not isinstance(self.api_calls, int)
            or self.api_calls < 0
        ):
            raise ValueError("api_calls must be a non-negative integer.")

        if self.final_distance_m is not None and not self.distance_method.strip():
            raise ValueError("Specify how final distance was measured.")


FIELDNAMES = [field.name for field in fields(ApproachTrial)]


def append_approach_trial(
    csv_path: str | Path,
    trial: ApproachTrial,
) -> None:
    """Append one result, rejecting incompatible files and duplicate IDs.

    Intended for one writer at a time.
    """

    path = Path(csv_path)
    has_content = path.exists() and path.stat().st_size > 0

    if has_content:
        with path.open(newline="", encoding="utf-8") as source:
            reader = csv.DictReader(source)

            if reader.fieldnames != FIELDNAMES:
                raise ValueError("Existing CSV has an incompatible header.")

            for row in reader:
                if row["trial_id"] == trial.trial_id:
                    raise ValueError("Trial ID already exists in this CSV.")

    row = {}
    for name, value in asdict(trial).items():
        if value is None:
            row[name] = ""
        elif isinstance(value, bool):
            row[name] = "true" if value else "false"
        else:
            row[name] = value

    path.parent.mkdir(parents=True, exist_ok=True)

    with path.open("a", newline="", encoding="utf-8") as destination:
        writer = csv.DictWriter(destination, fieldnames=FIELDNAMES)

        if not has_content:
            writer.writeheader()

        writer.writerow(row)