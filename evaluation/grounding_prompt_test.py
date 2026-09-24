"""Compare grounding prompts on saved camera frames with known answers.

The frames come from the first approach-trial run (evaluation/task4_evidence)
and the Task 3 scenes. For each, whether the target is really in the image was
checked by eye. Both prompts are sent to Qwen3-VL-Plus; results go to
evaluation/grounding_prompt_test.csv.

    python evaluation/grounding_prompt_test.py
"""

from __future__ import annotations

import csv
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "ros2_ws/src/ee4705_perception"))

from ee4705_perception.object_grounder import GROUNDING_PROMPT, ObjectGrounder  # noqa: E402
from ee4705_perception.vlm_client import QwenVLMClient  # noqa: E402

RESULTS = PROJECT_ROOT / "evaluation" / "grounding_prompt_test.csv"
TRIALS = "evaluation/task4_evidence/trials"
SCENES = "evaluation/scenes/comparison"

# The prompt used in the first approach-trial run.
ORIGINAL_PROMPT = """You are the visual grounding system of an indoor robot.

Locate the target object in the supplied image.

Target object: {target}

Return ONLY one JSON object in this exact format:
{{"found": true, "label": "object name", "bbox": [x1, y1, x2, y2]}}

Use coordinates normalized from 0 to 1000:
- (0, 0) is the top-left corner.
- (1000, 1000) is the bottom-right corner.
- x1 and y1 are the top-left corner of the object.
- x2 and y2 are the bottom-right corner of the object.

If the target is not clearly visible, return:
{{"found": false, "label": "object name", "bbox": null}}

Do not guess. Do not include Markdown or explanatory text."""

# (image, target, really visible, what the image shows)
CASES = [
    (f"{TRIALS}/attempt-wvircayg/frame-001.jpg", "person", True, "mannequin legs, 2 m"),
    (f"{TRIALS}/attempt-8bxk51c6/frame-000.jpg", "person", True, "mannequin legs, 2.7 m"),
    (f"{SCENES}/scene-005.jpg", "person", True, "mannequin legs, close"),
    (f"{SCENES}/scene-001.jpg", "person", True, "whole mannequin, far down a corridor"),
    (f"{TRIALS}/attempt-d6c1ffyw/frame-000.jpg", "cinder block", True, "small dark block 3.5 m away"),
    (f"{TRIALS}/attempt-wvircayg/frame-005.jpg", "fire hydrant", True, "hydrant through a doorway"),
    (f"{TRIALS}/attempt-d6c1ffyw/frame-004.jpg", "cinder block", False, "table and a brick pillar"),
    (f"{TRIALS}/attempt-d6c1ffyw/frame-002.jpg", "cinder block", False, "brick wall"),
    (f"{TRIALS}/attempt-d6c1ffyw/frame-006.jpg", "cinder block", False, "table, brick walls"),
    (f"{TRIALS}/attempt-8bxk51c6/frame-002.jpg", "person", False, "wooden wall"),
    (f"{TRIALS}/attempt-tbx5j1eg/frame-001.jpg", "fire hydrant", False, "wooden wall"),
    (f"{TRIALS}/attempt-tbx5j1eg/frame-003.jpg", "fire hydrant", False, "trash can and walls"),
]


def main() -> None:
    client = QwenVLMClient()
    rows = []
    for name, prompt in (("original", ORIGINAL_PROMPT), ("current", GROUNDING_PROMPT)):
        grounder = ObjectGrounder(client, prompt=prompt)
        correct = 0
        for image, target, visible, content in CASES:
            try:
                result = grounder.locate(PROJECT_ROOT / image, target)
                found, bbox, error = result.found, result.bbox, ""
            except Exception as exception:  # a malformed answer counts as wrong
                found, bbox, error = None, None, str(exception)[:200]
            correct += found == visible
            rows.append({"prompt": name, "image": image, "target": target,
                         "visible": visible, "image_shows": content, "found": found,
                         "bbox": "" if bbox is None else list(bbox),
                         "correct": found == visible, "error": error})
            print(f"{name:8s} {'OK ' if found == visible else 'BAD'} {target:12s} "
                  f"visible={visible!s:5s} found={found!s:5s} {content}")
        print(f"{name}: {correct}/{len(CASES)} correct\n")
    with RESULTS.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    print(f"Saved {RESULTS.relative_to(PROJECT_ROOT)}. Check the boxes of found targets by eye.")


if __name__ == "__main__":
    main()
