# EE4705 Project 1.2 | Task 4 - object approach
# Contributors (from git history): Marie (1 commit)
"""Combine grounding, image geometry, and approach-policy decisions."""

from __future__ import annotations

from dataclasses import dataclass

from .approach_geometry import TargetMeasurement, measure_target
from .approach_policy import (
    ApproachAction,
    ApproachConfig,
    decide_approach_action,
)
from .object_grounder import GroundingResult


@dataclass(frozen=True)
class ApproachDecision:
    """One complete decision based on a grounding result."""

    action: ApproachAction
    measurement: TargetMeasurement | None
    target_found: bool
    label: str
    model: str
    latency_s: float


def evaluate_grounding(
    grounding: GroundingResult,
    *,
    image_width: int,
    image_height: int,
    front_distance_m: float | None,
    config: ApproachConfig = ApproachConfig(),
) -> ApproachDecision:
    """Turn one validated grounding result into an approach action."""

    if grounding.found:
        if grounding.bbox is None:
            raise ValueError(
                "A found grounding result requires a bounding box."
            )

        measurement = measure_target(
            grounding.bbox,
            image_width=image_width,
            image_height=image_height,
        )

        action = decide_approach_action(
            target_found=True,
            horizontal_error=measurement.horizontal_error,
            height_fraction=measurement.height_fraction,
            front_distance_m=front_distance_m,
            config=config,
        )

    else:
        measurement = None

        action = decide_approach_action(
            target_found=False,
            horizontal_error=None,
            height_fraction=None,
            front_distance_m=front_distance_m,
            config=config,
        )

    return ApproachDecision(
        action=action,
        measurement=measurement,
        target_found=grounding.found,
        label=grounding.label,
        model=grounding.model,
        latency_s=grounding.latency_s,
    )