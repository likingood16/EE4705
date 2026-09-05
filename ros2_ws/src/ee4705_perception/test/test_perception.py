import csv
import tempfile
import unittest
from pathlib import Path

from ee4705_perception.result_logger import append_trial
from ee4705_perception.scene_describer import SceneDescriber
from ee4705_perception.vlm_client import MockVLMClient, image_to_data_url


class PerceptionTests(unittest.TestCase):
    def setUp(self):
        self.temp_directory = tempfile.TemporaryDirectory()
        self.temp_path = Path(self.temp_directory.name)

    def tearDown(self):
        self.temp_directory.cleanup()

    def make_test_image(self):
        image = self.temp_path / "room.jpg"
        image.write_bytes(b"offline-test-image")
        return image

    def test_image_to_data_url(self):
        image = self.make_test_image()
        self.assertTrue(image_to_data_url(image).startswith("data:image/jpeg;base64,"))

    def test_invalid_image_extension_is_rejected(self):
        text_file = self.temp_path / "room.txt"
        text_file.write_text("not an image", encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "JPEG, PNG, or WebP"):
            image_to_data_url(text_file)

    def test_mock_scene_description_and_csv_logging(self):
        image = self.make_test_image()
        response = SceneDescriber(MockVLMClient()).describe(image)
        self.assertEqual(response.model, "mock-vlm")
        self.assertIn("MOCK RESPONSE", response.text)

        output = self.temp_path / "results.csv"
        append_trial(
            output,
            trial_id="1",
            scene_id="room-1",
            image_path=image,
            question="Describe the room",
            response=response,
        )

        with output.open(newline="", encoding="utf-8") as csv_file:
            rows = list(csv.DictReader(csv_file))
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]["model"], "mock-vlm")
        self.assertEqual(rows[0]["notes"], "Needs manual scoring")

    def test_empty_visual_question_is_rejected(self):
        image = self.make_test_image()
        service = SceneDescriber(MockVLMClient())
        with self.assertRaisesRegex(ValueError, "cannot be empty"):
            service.answer(image, "   ")


if __name__ == "__main__":
    unittest.main()
