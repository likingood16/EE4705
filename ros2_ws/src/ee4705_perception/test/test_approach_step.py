# EE4705 Project 1.2 | Task 4 - object approach
# Contributors (from git history): Alexander Likin (2 commits)
"""Bearing/range geometry, laser helpers and the step planner."""

import math
import unittest

from ee4705_perception.approach_geometry import (
    camera_to_scan, ground_distance, pixel_bearing,
)
from ee4705_perception.approach_policy import StepConfig, StepKind, plan_step
from ee4705_perception.approach_robot import estimate_target_range
from ee4705_perception.approach_runtime import corridor_clearance, range_at_bearing


def scan(ranges):
    return (ranges, 0.0, math.pi / 180, 0.12, 3.5)


class GeometryTests(unittest.TestCase):
    def test_bearing_sign_and_edges(self):
        self.assertAlmostEqual(pixel_bearing(320, 640), 0.0)
        self.assertGreater(pixel_bearing(100, 640), 0)  # left of centre is +yaw
        self.assertAlmostEqual(pixel_bearing(0, 640), 1.085595 / 2, places=6)

    def test_ground_distance(self):
        self.assertIsNone(ground_distance(240, 640, 480))  # horizon
        focal = 320 / math.tan(1.085595 / 2)
        self.assertAlmostEqual(ground_distance(340, 640, 480), 0.103 * focal / 100)

    def test_camera_to_scan_adds_mount_offset(self):
        distance, bearing = camera_to_scan(1.0, 0.0)
        self.assertAlmostEqual(distance, 1.14)
        self.assertAlmostEqual(bearing, 0.0)


class LaserTests(unittest.TestCase):
    def test_range_at_bearing_uses_window(self):
        ranges = [3.0] * 360
        ranges[20] = 1.2
        self.assertAlmostEqual(range_at_bearing(
            *scan(ranges), bearing=math.radians(20), half_window=math.radians(3)), 1.2)
        self.assertAlmostEqual(range_at_bearing(
            *scan(ranges), bearing=0.0, half_window=math.radians(3)), 3.0)

    def test_inf_only_counts_as_clear_when_allowed(self):
        ranges = [math.inf] * 360
        self.assertIsNone(range_at_bearing(*scan(ranges), bearing=0.0, half_window=0.1))
        self.assertEqual(range_at_bearing(*scan(ranges), bearing=0.0, half_window=0.1,
                                          inf_is_clear=True), math.inf)

    def test_corridor_ignores_side_walls(self):
        ranges = [3.0] * 360
        for i in range(60, 121):
            ranges[i] = 0.3  # wall 0.3 m to the left
        ranges[300] = 0.3
        self.assertGreater(corridor_clearance(*scan(ranges), half_width=0.19), 1.0)

    def test_corridor_sees_narrow_obstacle_ahead(self):
        ranges = [3.0] * 360
        ranges[5] = 0.8  # 0.07 m off the centre line
        self.assertAlmostEqual(corridor_clearance(*scan(ranges), half_width=0.19),
                               0.8 * math.cos(math.radians(5)))


class PlannerTests(unittest.TestCase):
    def test_not_found_searches(self):
        step = plan_step(target_found=False, bearing_rad=None, target_range_m=None)
        self.assertEqual(step.kind, StepKind.SEARCH)
        self.assertAlmostEqual(step.turn_rad, math.radians(45))

    def test_far_target_turns_then_steps(self):
        step = plan_step(target_found=True, bearing_rad=0.3, target_range_m=4.0)
        self.assertEqual(step.kind, StepKind.ADVANCE)
        self.assertAlmostEqual(step.turn_rad, 0.3)
        self.assertAlmostEqual(step.forward_m, StepConfig().max_step_m)

    def test_near_target_steps_to_arrival_range(self):
        step = plan_step(target_found=True, bearing_rad=0.0, target_range_m=1.0)
        self.assertAlmostEqual(step.forward_m, 1.0 - StepConfig().arrival_range_m)

    def test_arrives_only_when_centred(self):
        self.assertEqual(plan_step(target_found=True, bearing_rad=0.05,
                                   target_range_m=0.55).kind, StepKind.ARRIVED)
        off_centre = plan_step(target_found=True, bearing_rad=0.5, target_range_m=0.55)
        self.assertEqual(off_centre.kind, StepKind.ADVANCE)
        self.assertEqual(off_centre.forward_m, 0.0)

    def test_unknown_range_takes_short_step(self):
        step = plan_step(target_found=True, bearing_rad=0.0, target_range_m=None)
        self.assertAlmostEqual(step.forward_m, StepConfig().unknown_range_step_m)


class RangeFusionTests(unittest.TestCase):
    def test_laser_preferred_when_consistent(self):
        self.assertEqual(estimate_target_range(1.0, 1.1), (1.0, "laser"))

    def test_floor_used_when_laser_passes_over_low_object(self):
        self.assertEqual(estimate_target_range(3.0, 0.8), (0.8, "floor"))

    def test_unknown_without_either(self):
        self.assertEqual(estimate_target_range(math.inf, None), (None, "unknown"))


if __name__ == "__main__":
    unittest.main()


class SweepAndStandoffTests(unittest.TestCase):
    def test_sweep_measured_from_robot_centre(self):
        from ee4705_perception.approach_runtime import sweep_clearance
        ranges = [3.0] * 360
        ranges[0] = 0.33  # 0.33 m ahead of the laser = 0.266 m from the centre
        ranges[180] = 0.33  # behind the laser = 0.394 m from the centre
        value = sweep_clearance(*scan(ranges), scan_x=-0.064)
        self.assertAlmostEqual(value, 0.33 - 0.064, places=3)

    def test_standoff_pose_is_short_of_target_and_faces_it(self):
        from ee4705_perception.approach_robot import NAV2_STANDOFF_M, standoff_pose
        x, y, yaw = standoff_pose((1.0, 2.0), math.pi / 2, 3.0, 0.0)
        self.assertAlmostEqual(yaw, math.pi / 2)
        self.assertAlmostEqual(x, 1.0)
        self.assertAlmostEqual(y, 2.0 - 0.064 + 3.0 - NAV2_STANDOFF_M)
