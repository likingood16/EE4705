"""Offline pipeline checks, not real robot evaluation trials."""

import csv
import json
import tempfile
import unittest
from pathlib import Path

from ee4705_perception.approach_policy import ApproachAction
from ee4705_perception.approach_result_logger import (
    ApproachTrial,
    append_approach_trial,
)
from ee4705_perception.approach_session import evaluate_grounding
from ee4705_perception.approach_velocity import (
    VelocityRequest,
    velocity_for_action,
)
from ee4705_perception.object_grounder import ObjectGrounder
from ee4705_perception.vlm_client import VLMResponse


class ScriptedVLMClient:
    """Return predetermined JSON without inspecting an image."""

    def __init__(self, responses):
        self.responses = iter(responses)
        self.calls = 0

    def ask(self, image_path, prompt):
        self.calls += 1
        return VLMResponse(
            text=json.dumps(next(self.responses)),
            model="scripted-mock",
            latency_s=0.0,
            cost_usd=0.0,
        )


class ApproachPipelineTests(unittest.TestCase):

    def test_search_align_advance_and_stop_sequence(self):
        client = ScriptedVLMClient([
            {"found": False, "label": "cup", "bbox": None},
            {"found": True, "label": "cup",
             "bbox": [0, 200, 200, 500]},
            {"found": True, "label": "cup",
             "bbox": [400, 200, 600, 500]},
            {"found": True, "label": "cup",
             "bbox": [350, 100, 650, 800]},
        ])
        grounder = ObjectGrounder(client)

        expected_actions = [
            ApproachAction.SEARCH,
            ApproachAction.TURN_LEFT,
            ApproachAction.MOVE_FORWARD,
            ApproachAction.STOP_TARGET_REACHED,
        ]
        requests = []

        for expected in expected_actions:
            grounding = grounder.locate("unused.jpg", "cup")
            decision = evaluate_grounding(
                grounding,
                image_width=640,
                image_height=480,
                front_distance_m=2.0,
            )

            self.assertEqual(decision.action, expected)

            requests.append(
                velocity_for_action(
                    decision.action,
                    motion_permitted=True,
                )
            )

        self.assertGreater(requests[0].angular_z, 0.0)
        self.assertEqual(requests[0].linear_x, 0.0)
        self.assertGreater(requests[1].angular_z, 0.0)
        self.assertGreater(requests[2].linear_x, 0.0)
        self.assertEqual(requests[3], VelocityRequest())
        self.assertEqual(client.calls, 4)

        # Log the software check without inventing robot measurements.
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "mock_trials.csv"
            append_approach_trial(
                output,
                ApproachTrial(
                    trial_id="mock-sequence-001",
                    target="cup",
                    mode="mock",
                    model="scripted-mock",
                    api_calls=0,
                    cost_usd=0.0,
                    notes="Four scripted responses; no robot movement.",
                ),
            )

            with output.open(newline="", encoding="utf-8") as source:
                rows = list(csv.DictReader(source))

            self.assertEqual(len(rows), 1)
            self.assertEqual(rows[0]["mode"], "mock")
            self.assertEqual(rows[0]["approach_success"], "")
            self.assertEqual(rows[0]["final_distance_m"], "")
            self.assertEqual(rows[0]["api_calls"], "0")

    def test_missing_scan_produces_zero_velocity(self):
        client = ScriptedVLMClient([
            {"found": True, "label": "cup",
             "bbox": [400, 200, 600, 500]},
        ])
        grounding = ObjectGrounder(client).locate("unused.jpg", "cup")

        decision = evaluate_grounding(
            grounding,
            image_width=640,
            image_height=480,
            front_distance_m=None,
        )
        request = velocity_for_action(
            decision.action,
            motion_permitted=True,
        )

        self.assertEqual(
            decision.action,
            ApproachAction.STOP_SENSOR_UNAVAILABLE,
        )
        self.assertEqual(request, VelocityRequest())

    def test_close_obstacle_prevents_forward_motion(self):
        client = ScriptedVLMClient([
            {"found": True, "label": "cup",
             "bbox": [400, 200, 600, 500]},
        ])
        grounding = ObjectGrounder(client).locate("unused.jpg", "cup")

        decision = evaluate_grounding(
            grounding,
            image_width=640,
            image_height=480,
            front_distance_m=0.2,
        )
        request = velocity_for_action(
            decision.action,
            motion_permitted=True,
        )

        self.assertEqual(
            decision.action,
            ApproachAction.STOP_OBSTACLE,
        )
        self.assertEqual(request, VelocityRequest())


if __name__ == "__main__":
    unittest.main()