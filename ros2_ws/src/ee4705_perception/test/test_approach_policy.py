import unittest

from ee4705_perception.approach_policy import (
    ApproachAction,
    ApproachConfig,
    decide_approach_action,
)


class ApproachPolicyTests(unittest.TestCase):

    def test_missing_target_starts_search(self):
        action = decide_approach_action(
            target_found=False,
            horizontal_error=None,
            height_fraction=None,
            front_distance_m=2.0,
        )

        self.assertEqual(action, ApproachAction.SEARCH)

    def test_left_target_causes_left_turn(self):
        action = decide_approach_action(
            target_found=True,
            horizontal_error=-0.5,
            height_fraction=0.2,
            front_distance_m=2.0,
        )

        self.assertEqual(action, ApproachAction.TURN_LEFT)

    def test_right_target_causes_right_turn(self):
        action = decide_approach_action(
            target_found=True,
            horizontal_error=0.5,
            height_fraction=0.2,
            front_distance_m=2.0,
        )

        self.assertEqual(action, ApproachAction.TURN_RIGHT)

    def test_centred_distant_target_moves_forward(self):
        action = decide_approach_action(
            target_found=True,
            horizontal_error=0.05,
            height_fraction=0.2,
            front_distance_m=2.0,
        )

        self.assertEqual(action, ApproachAction.MOVE_FORWARD)

    def test_large_target_stops_as_reached(self):
        action = decide_approach_action(
            target_found=True,
            horizontal_error=0.0,
            height_fraction=0.6,
            front_distance_m=1.0,
        )

        self.assertEqual(action, ApproachAction.STOP_TARGET_REACHED)

    def test_close_obstacle_has_highest_priority(self):
        action = decide_approach_action(
            target_found=True,
            horizontal_error=0.5,
            height_fraction=0.2,
            front_distance_m=0.2,
        )

        self.assertEqual(action, ApproachAction.STOP_OBSTACLE)

    def test_thresholds_can_be_configured(self):
        config = ApproachConfig(
            center_tolerance=0.1,
            close_height_fraction=0.7,
            minimum_front_distance_m=0.4,
        )

        action = decide_approach_action(
            target_found=True,
            horizontal_error=0.12,
            height_fraction=0.3,
            front_distance_m=1.0,
            config=config,
        )

        self.assertEqual(action, ApproachAction.TURN_RIGHT)

    def test_visible_target_requires_measurements(self):
        with self.assertRaisesRegex(ValueError, "requires"):
            decide_approach_action(
                target_found=True,
                horizontal_error=None,
                height_fraction=None,
                front_distance_m=1.0,
            )


if __name__ == "__main__":
    unittest.main()