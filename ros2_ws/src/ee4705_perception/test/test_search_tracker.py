# EE4705 Project 1.2 | Task 4 - object search
# Contributors (from git history): Marie (1 commit)
import math
import unittest

from ee4705_perception.search_tracker import (
    SearchStatus,
    SearchTracker,
)


class SearchTrackerTests(unittest.TestCase):

    def make_tracker(self):
        return SearchTracker(
            initial_yaw_rad=0.0,
            started_at_s=0.0,
        )

    def test_small_rotation_continues_search(self):
        tracker = self.make_tracker()

        status = tracker.update(
            yaw_rad=0.2,
            now_s=1.0,
            target_found=False,
        )

        self.assertEqual(status, SearchStatus.SEARCHING)
        self.assertAlmostEqual(tracker.rotation_rad, 0.2)

    def test_visible_target_ends_search(self):
        tracker = self.make_tracker()

        status = tracker.update(
            yaw_rad=0.0,
            now_s=1.0,
            target_found=True,
        )

        self.assertEqual(status, SearchStatus.FOUND)

    def test_full_turn_exhausts_search(self):
        tracker = self.make_tracker()

        for step in range(1, 13):
            status = tracker.update(
                yaw_rad=step * math.pi / 6,
                now_s=float(step),
                target_found=False,
            )

        self.assertEqual(status, SearchStatus.EXHAUSTED)

    def test_angle_wraparound_is_handled(self):
        tracker = SearchTracker(
            initial_yaw_rad=math.radians(170),
            started_at_s=0.0,
        )

        tracker.update(
            yaw_rad=math.radians(-170),
            now_s=1.0,
            target_found=False,
        )

        self.assertAlmostEqual(
            tracker.rotation_rad,
            math.radians(20),
        )

    def test_timeout_stops_search(self):
        tracker = self.make_tracker()

        status = tracker.update(
            yaw_rad=0.0,
            now_s=60.0,
            target_found=False,
        )

        self.assertEqual(status, SearchStatus.TIMED_OUT)

    def test_finished_search_stays_finished(self):
        tracker = self.make_tracker()
        tracker.update(
            yaw_rad=0.0,
            now_s=60.0,
            target_found=False,
        )

        status = tracker.update(
            yaw_rad=0.1,
            now_s=61.0,
            target_found=True,
        )

        self.assertEqual(status, SearchStatus.TIMED_OUT)

    def test_backwards_time_is_rejected(self):
        tracker = self.make_tracker()

        with self.assertRaisesRegex(ValueError, "backwards"):
            tracker.update(
                yaw_rad=0.0,
                now_s=-1.0,
                target_found=False,
            )

    def test_invalid_limit_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "positive"):
            SearchTracker(
                initial_yaw_rad=0.0,
                started_at_s=0.0,
                timeout_s=0.0,
            )


if __name__ == "__main__":
    unittest.main()