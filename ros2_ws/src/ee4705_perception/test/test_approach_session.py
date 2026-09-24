import unittest

from ee4705_perception.approach_policy import ApproachAction
from ee4705_perception.approach_session import evaluate_grounding
from ee4705_perception.object_grounder import GroundingResult


def make_grounding(
    *,
    found=True,
    bbox=(250.0, 200.0, 750.0, 500.0),
):
    return GroundingResult(
        found=found,
        target="red cup",
        label="red cup",
        bbox=bbox if found else None,
        model="test-vlm",
        latency_s=0.5,
        cost_usd=None,
        raw_text="test response",
    )


class ApproachSessionTests(unittest.TestCase):

    def test_centred_distant_target_moves_forward(self):
        decision = evaluate_grounding(
            make_grounding(),
            image_width=640,
            image_height=480,
            front_distance_m=2.0,
        )

        self.assertEqual(
            decision.action,
            ApproachAction.MOVE_FORWARD,
        )
        self.assertIsNotNone(decision.measurement)
        self.assertEqual(decision.label, "red cup")

    def test_left_target_turns_left(self):
        decision = evaluate_grounding(
            make_grounding(
                bbox=(0.0, 200.0, 200.0, 500.0),
            ),
            image_width=640,
            image_height=480,
            front_distance_m=2.0,
        )

        self.assertEqual(
            decision.action,
            ApproachAction.TURN_LEFT,
        )

    def test_missing_target_starts_search(self):
        decision = evaluate_grounding(
            make_grounding(found=False, bbox=None),
            image_width=640,
            image_height=480,
            front_distance_m=2.0,
        )

        self.assertEqual(
            decision.action,
            ApproachAction.SEARCH,
        )
        self.assertIsNone(decision.measurement)

    def test_close_obstacle_overrides_movement(self):
        decision = evaluate_grounding(
            make_grounding(),
            image_width=640,
            image_height=480,
            front_distance_m=0.2,
        )

        self.assertEqual(
            decision.action,
            ApproachAction.STOP_OBSTACLE,
        )

    def test_grounding_metadata_is_preserved(self):
        decision = evaluate_grounding(
            make_grounding(),
            image_width=640,
            image_height=480,
            front_distance_m=2.0,
        )

        self.assertEqual(decision.model, "test-vlm")
        self.assertAlmostEqual(decision.latency_s, 0.5)


if __name__ == "__main__":
    unittest.main()