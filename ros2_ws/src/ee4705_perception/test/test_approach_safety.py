# EE4705 Project 1.2 | Task 4 - object approach
# Contributors (from git history): Marie (1 commit)
import unittest

from ee4705_perception.approach_policy import (
    ApproachAction,
    ApproachConfig,
    decide_approach_action,
)


class ApproachSafetyTests(unittest.TestCase):

    def decide(self, distance, *, found=True, error=0.0, height=0.2):
        return decide_approach_action(
            target_found=found,
            horizontal_error=error if found else None,
            height_fraction=height if found else None,
            front_distance_m=distance,
        )

    def test_missing_scan_stops_forward_motion(self):
        self.assertEqual(
            self.decide(None),
            ApproachAction.STOP_SENSOR_UNAVAILABLE,
        )

    def test_missing_scan_stops_search(self):
        self.assertEqual(
            self.decide(None, found=False),
            ApproachAction.STOP_SENSOR_UNAVAILABLE,
        )

    def test_invalid_distances_stop_motion(self):
        for distance in (float("nan"), float("inf"), -1.0):
            with self.subTest(distance=distance):
                self.assertEqual(
                    self.decide(distance),
                    ApproachAction.STOP_SENSOR_UNAVAILABLE,
                )

    def test_exact_safety_threshold_stops(self):
        self.assertEqual(
            self.decide(0.35),
            ApproachAction.STOP_OBSTACLE,
        )

    def test_close_off_centre_target_does_not_claim_arrival(self):
        self.assertEqual(
            self.decide(0.7, error=0.7, height=0.6),
            ApproachAction.TURN_RIGHT,
        )

    def test_invalid_configuration_is_rejected(self):
        for options in (
            {"center_tolerance": -0.1},
            {"close_height_fraction": 1.5},
            {"minimum_front_distance_m": 0.0},
            {"target_reached_distance_m": 0.3},
        ):
            with self.subTest(options=options):
                with self.assertRaises(ValueError):
                    ApproachConfig(**options)


if __name__ == "__main__":
    unittest.main()