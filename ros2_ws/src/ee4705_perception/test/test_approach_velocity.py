# EE4705 Project 1.2 | Task 4 - object approach
# Contributors (from git history): Marie (1 commit)
import unittest

from ee4705_perception.approach_policy import ApproachAction
from ee4705_perception.approach_velocity import (
    VelocityConfig,
    VelocityRequest,
    velocity_for_action,
)


class ApproachVelocityTests(unittest.TestCase):

    def test_motion_is_disabled_by_default(self):
        for action in ApproachAction:
            with self.subTest(action=action):
                self.assertEqual(
                    velocity_for_action(action),
                    VelocityRequest(),
                )

    def test_forward_has_no_turning(self):
        request = velocity_for_action(
            ApproachAction.MOVE_FORWARD,
            motion_permitted=True,
        )

        self.assertAlmostEqual(request.linear_x, 0.1)
        self.assertEqual(request.angular_z, 0.0)

    def test_turns_have_opposite_signs_and_no_forward_motion(self):
        for action, expected in (
            (ApproachAction.TURN_LEFT, 0.35),
            (ApproachAction.TURN_RIGHT, -0.35),
        ):
            with self.subTest(action=action):
                request = velocity_for_action(
                    action,
                    motion_permitted=True,
                )
                self.assertEqual(request.linear_x, 0.0)
                self.assertAlmostEqual(request.angular_z, expected)

    def test_search_rotates_without_forward_motion(self):
        request = velocity_for_action(
            ApproachAction.SEARCH,
            motion_permitted=True,
        )

        self.assertEqual(request.linear_x, 0.0)
        self.assertAlmostEqual(request.angular_z, 0.30)

    def test_all_stop_actions_produce_zero_velocity(self):
        for action in (
            ApproachAction.STOP_TARGET_REACHED,
            ApproachAction.STOP_OBSTACLE,
            ApproachAction.STOP_SENSOR_UNAVAILABLE,
        ):
            with self.subTest(action=action):
                self.assertEqual(
                    velocity_for_action(action, motion_permitted=True),
                    VelocityRequest(),
                )

    def test_invalid_speeds_are_rejected(self):
        for settings in (
            {"forward_speed_mps": 0.5},
            {"turn_speed_radps": -0.1},
            {"search_speed_radps": float("nan")},
            {"forward_speed_mps": float("inf")},
        ):
            with self.subTest(settings=settings):
                with self.assertRaises(ValueError):
                    VelocityConfig(**settings)

    def test_unknown_action_is_rejected(self):
        with self.assertRaises(ValueError):
            velocity_for_action("move_forward", motion_permitted=True)


if __name__ == "__main__":
    unittest.main()