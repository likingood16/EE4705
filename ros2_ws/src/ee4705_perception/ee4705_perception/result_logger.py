"""Append VLM trial results using the project's evaluation CSV format."""

from __future__ import annotations

import csv
from pathlib import Path

from .vlm_client import VLMResponse


FIELDNAMES = [
    "trial_id",
    "scene_id",
    "image_path",
    "model",
    "question",
    "expected_objects",
    "reported_objects",
    "correct_objects",
    "missed_objects",
    "hallucinated_objects",
    "latency_s",
    "cost_usd",
    "notes",
]


def append_trial(
    csv_path: str | Path,
    *,
    trial_id: str,
    scene_id: str,
    image_path: str | Path,
    question: str,
    response: VLMResponse,
    notes: str = "Needs manual scoring",
) -> None:
    """Append one unscored model result without deleting earlier trials."""

    path = Path(csv_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    write_header = not path.exists() or path.stat().st_size == 0

    row = {
        "trial_id": trial_id,
        "scene_id": scene_id,
        "image_path": str(image_path),
        "model": response.model,
        "question": question,
        "expected_objects": "",
        "reported_objects": response.text,
        "correct_objects": "",
        "missed_objects": "",
        "hallucinated_objects": "",
        "latency_s": f"{response.latency_s:.4f}",
        "cost_usd": "" if response.cost_usd is None else response.cost_usd,
        "notes": notes,
    }

    with path.open("a", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=FIELDNAMES)
        if write_header:
            writer.writeheader()
        writer.writerow(row)
