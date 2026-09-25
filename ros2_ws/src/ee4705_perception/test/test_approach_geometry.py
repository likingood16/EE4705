# EE4705 Project 1.2 | Task 4 - object approach
# Contributors (from git history): Marie (1 commit)
import unittest

from ee4705_perception.approach_geometry import (
    measure_target,
    normalized_bbox_to_pixels,
)


class ApproachGeometryTests(unittest.TestCase):

    def test_normalized_bbox_is_converted_to_pixels(self):
        pixels = normalized_bbox_to_pixels(
            (250.0, 100.0, 750.0, 900.0),
            image_width=640,
            image_height=480,
        )

        self.assertEqual(pixels, (160.0, 48.0, 480.0, 432.0))

    def test_centred_target_has_zero_error(self):
        measurement = measure_target(
            (250.0, 100.0, 750.0, 900.0),
            image_width=640,
            image_height=480,
        )

        self.assertAlmostEqual(measurement.center_x_pixels, 320.0)
        self.assertAlmostEqual(measurement.horizontal_error, 0.0)
        self.assertAlmostEqual(measurement.width_fraction, 0.5)
        self.assertAlmostEqual(measurement.height_fraction, 0.8)

    def test_left_target_has_negative_error(self):
        measurement = measure_target(
            (0.0, 100.0, 200.0, 400.0),
            image_width=640,
            image_height=480,
        )

        self.assertLess(measurement.horizontal_error, 0.0)
        self.assertAlmostEqual(measurement.horizontal_error, -0.8)

    def test_right_target_has_positive_error(self):
        measurement = measure_target(
            (800.0, 100.0, 1000.0, 400.0),
            image_width=640,
            image_height=480,
        )

        self.assertGreater(measurement.horizontal_error, 0.0)
        self.assertAlmostEqual(measurement.horizontal_error, 0.8)

    def test_invalid_image_size_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "must be positive"):
            normalized_bbox_to_pixels(
                (100.0, 100.0, 500.0, 500.0),
                image_width=0,
                image_height=480,
            )


if __name__ == "__main__":
    unittest.main()