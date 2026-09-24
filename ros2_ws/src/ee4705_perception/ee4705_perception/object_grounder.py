"""Visual grounding for Task 4 language-directed object approach."""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

from .vlm_client import VLMClient


# Tells the model what this robot's view looks like. The first version
# ("Locate the target object ... Do not guess") missed the white mannequin
# ("person"), which the camera sees only up to its knees when close, and the
# small dark cinder block, and boxed a brick pillar as "cinder block": 6/12
# correct on saved trial frames with known answers versus 12/12 for this one
# (evaluation/grounding_prompt_test.py).
GROUNDING_PROMPT = """You are the visual grounding system of a small indoor robot. Its camera is
only 10 cm above the floor, and the house is a simulation built from simple
3D models.

Locate the target object in the supplied image.

Target object: {target}

Things to know about this robot's view:
- Objects are simplified 3D models. For example, a person may be a plain
  white or grey mannequin without a face or clothes.
- Because the camera is low, a nearby tall object is often cut off at the top
  of the image, e.g. only a person's legs and feet are visible. Report the
  target if a clearly identifiable part of it is visible.
- The target may be small and far away; look carefully along the floor.
- Walls, wall corners, pillars, doorways and other parts of the building are
  never the target. Do not box a different object that merely has a similar
  colour or texture.

Return ONLY one JSON object in this exact format:
{{"found": true, "label": "object name", "bbox": [x1, y1, x2, y2]}}

Use coordinates normalized from 0 to 1000:
- (0, 0) is the top-left corner.
- (1000, 1000) is the bottom-right corner.
- x1 and y1 are the top-left corner of the object.
- x2 and y2 are the bottom-right corner of the object.

If the target is not visible, return:
{{"found": false, "label": "object name", "bbox": null}}

Do not include Markdown or explanatory text."""


@dataclass(frozen=True)
class GroundingResult:
    """Validated result of one visual-grounding request."""

    found: bool
    target: str
    label: str
    bbox: tuple[float, float, float, float] | None
    model: str
    latency_s: float
    cost_usd: float | None
    raw_text: str
    input_tokens: int | None = None
    output_tokens: int | None = None
    retries: int = 0


def extract_json_object(text: str) -> dict:
    """Extract one JSON object from a VLM response."""

    start = text.find("{")
    end = text.rfind("}")

    if start == -1 or end == -1 or end < start:
        raise ValueError("The VLM response does not contain a JSON object.")

    try:
        data = json.loads(text[start : end + 1])
    except json.JSONDecodeError as error:
        raise ValueError("The VLM returned malformed JSON.") from error

    if not isinstance(data, dict):
        raise ValueError("The grounding response must be a JSON object.")

    return data


def validate_bbox(value: object) -> tuple[float, float, float, float]:
    """Validate a normalized [x1, y1, x2, y2] bounding box."""

    if not isinstance(value, list) or len(value) != 4:
        raise ValueError("bbox must contain exactly four coordinates.")

    if any(isinstance(item, bool) or not isinstance(item, (int, float)) for item in value):
        raise ValueError("Every bbox coordinate must be a number.")

    x1, y1, x2, y2 = (float(item) for item in value)

    if not all(0.0 <= coordinate <= 1000.0 for coordinate in (x1, y1, x2, y2)):
        raise ValueError("bbox coordinates must be between 0 and 1000.")

    if x1 >= x2 or y1 >= y2:
        raise ValueError("bbox must satisfy x1 < x2 and y1 < y2.")

    return x1, y1, x2, y2


class ObjectGrounder:
    """Locate a language-specified object in one saved image."""

    def __init__(self, client: VLMClient, prompt: str = GROUNDING_PROMPT) -> None:
        self.client = client
        self.prompt = prompt

    def locate(
        self,
        image_path: str | Path,
        target: str,
    ) -> GroundingResult:
        """Ask the VLM to locate a target and validate its response."""

        cleaned_target = target.strip()

        if not cleaned_target:
            raise ValueError("The target object cannot be empty.")

        response = self.client.ask(
            image_path,
            self.prompt.format(target=cleaned_target),
        )
        data = extract_json_object(response.text)

        found = data.get("found")
        if not isinstance(found, bool):
            raise ValueError("The grounding response requires a boolean found field.")

        label = data.get("label", cleaned_target)
        if not isinstance(label, str) or not label.strip():
            raise ValueError("The grounding response requires a non-empty label.")

        if found:
            bbox = validate_bbox(data.get("bbox"))
        else:
            bbox = None
            if data.get("bbox") is not None:
                raise ValueError("bbox must be null when found is false.")

        return GroundingResult(
            found=found,
            target=cleaned_target,
            label=label.strip(),
            bbox=bbox,
            model=response.model,
            latency_s=response.latency_s,
            cost_usd=response.cost_usd,
            raw_text=response.text,
            input_tokens=response.input_tokens,
            output_tokens=response.output_tokens,
            retries=getattr(response, "retries", 0),
        )


