# EE4705 Project 1.2 | Task 4 - object approach
# Contributors (from git history): Marie (1 commit)
import math
import threading
import time
import unittest

from ee4705_perception.approach_runtime import (
    ApproachAbort, fresh, scan_clearance, wait_for_grounding, yaw_from_quaternion,
)


class RuntimeTests(unittest.TestCase):
    def scan(self, ranges=None, **kwargs):
        return scan_clearance([2.0]*360 if ranges is None else ranges,
                              0.0, math.pi/180, 0.1, 5.0, **kwargs)

    def test_front_sector_wraps_zero(self):
        rays = [2.0]*360
        rays[359] = 0.2
        self.assertEqual(self.scan(rays), 0.2)

    def test_rear_obstacle_only_affects_rotation(self):
        rays = [2.0]*360
        rays[180] = 0.2
        self.assertEqual(self.scan(rays), 2.0)
        self.assertEqual(self.scan(rays, all_around=True), 0.2)

    def test_nan_front_invalidates(self):
        rays = [2.0]*360
        rays[0] = math.nan
        self.assertIsNone(self.scan(rays))

    def test_infinity_requires_explicit_convention(self):
        self.assertIsNone(self.scan([math.inf]*360))
        self.assertEqual(self.scan([math.inf]*360, inf_is_clear=True), 5.0)

    def test_negative_infinity_is_never_clear(self):
        self.assertIsNone(self.scan([-math.inf]*360, inf_is_clear=True))

    def test_below_range_min_is_blocked(self):
        self.assertEqual(self.scan([0.02]*360), 0.0)

    def test_above_range_max_invalidates(self):
        self.assertIsNone(self.scan([6.0]*360))

    def test_partial_scan_cannot_authorize_rotation(self):
        self.assertIsNone(self.scan([2.0]*180, all_around=True))

    def test_empty_scan_invalid(self):
        self.assertIsNone(self.scan([]))

    def test_freshness(self):
        self.assertTrue(fresh(1.0, 1.5, 0.75))
        self.assertFalse(fresh(1.0, 2.0, 0.75))
        self.assertFalse(fresh(None, 2.0, 0.75))
        self.assertFalse(fresh(3.0, 2.0, 0.75))

    def test_yaw_normalizes_quaternion(self):
        self.assertAlmostEqual(yaw_from_quaternion(0, 0, 2, 2), math.pi/2)

    def test_bad_quaternion_rejected(self):
        with self.assertRaises(ValueError):
            yaw_from_quaternion(0, 0, 0, 0)
        with self.assertRaises(ValueError):
            yaw_from_quaternion(0, 0, math.nan, 1)

    def test_network_worker_success_polls_and_stops(self):
        stops = []
        polls = []
        result = wait_for_grounding(lambda: 42, poll=lambda: polls.append(1),
                                    stop=lambda: stops.append(1), now=time.monotonic,
                                    timeout_s=1)
        self.assertEqual(result, 42)
        self.assertTrue(stops and polls)

    def test_network_worker_exception(self):
        def fail():
            raise ValueError("bad JSON")
        with self.assertRaisesRegex(ApproachAbort, "grounding_failed"):
            wait_for_grounding(fail, poll=lambda: time.sleep(0.001),
                               stop=lambda: None, now=time.monotonic, timeout_s=1)

    def test_network_timeout_and_late_result(self):
        release = threading.Event()
        try:
            with self.assertRaisesRegex(ApproachAbort, "vlm_timeout"):
                wait_for_grounding(lambda: release.wait(2),
                                   poll=lambda: time.sleep(0.001), stop=lambda: None,
                                   now=time.monotonic, timeout_s=0.01)
        finally:
            release.set()
        self.assertEqual(wait_for_grounding(lambda: "new", poll=lambda: time.sleep(0.001),
                                           stop=lambda: None, now=time.monotonic,
                                           timeout_s=1), "new")

    def test_cancellation_precedes_available_result(self):
        def cancel():
            raise ApproachAbort("cancelled")
        with self.assertRaisesRegex(ApproachAbort, "cancelled"):
            wait_for_grounding(lambda: 42, poll=cancel, stop=lambda: None,
                               now=time.monotonic, timeout_s=1)
