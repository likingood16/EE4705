import unittest

from ee4705_perception.object_grounder import ObjectGrounder
from ee4705_perception.vlm_client import VLMResponse


class FixedVLMClient:
    """Fake VLM client that returns a response selected by the test."""

    def __init__(self, text: str) -> None:
        self.text = text
        self.last_prompt = ""

    def ask(self, image_path, prompt: str) -> VLMResponse:
        self.last_prompt = prompt
        return VLMResponse(
            text=self.text,
            model="test-vlm",
            latency_s=0.25,
            cost_usd=0.001,
        )


class ObjectGrounderTests(unittest.TestCase):

    def test_visible_object_is_parsed(self):
        client = FixedVLMClient(
            '{"found": true, "label": "red cup", '
            '"bbox": [200, 150, 500, 800]}'
        )
        grounder = ObjectGrounder(client)

        result = grounder.locate("unused-test-image.jpg", "red cup")

        self.assertTrue(result.found)
        self.assertEqual(result.target, "red cup")
        self.assertEqual(result.label, "red cup")
        self.assertEqual(result.bbox, (200.0, 150.0, 500.0, 800.0))
        self.assertEqual(result.model, "test-vlm")
        self.assertAlmostEqual(result.latency_s, 0.25)
        self.assertIn("Target object: red cup", client.last_prompt)

    def test_not_visible_object_is_parsed(self):
        client = FixedVLMClient(
            '{"found": false, "label": "blue ball", "bbox": null}'
        )
        grounder = ObjectGrounder(client)

        result = grounder.locate("unused-test-image.jpg", "blue ball")

        self.assertFalse(result.found)
        self.assertIsNone(result.bbox)

    def test_empty_target_is_rejected(self):
        client = FixedVLMClient(
            '{"found": false, "label": "unknown", "bbox": null}'
        )
        grounder = ObjectGrounder(client)

        with self.assertRaisesRegex(ValueError, "cannot be empty"):
            grounder.locate("unused-test-image.jpg", "   ")

    def test_malformed_json_is_rejected(self):
        client = FixedVLMClient(
            '{"found": true, "label": "cup", "bbox": [100, 200, 300, 400]'
        )
        grounder = ObjectGrounder(client)

        with self.assertRaisesRegex(ValueError, "JSON"):
            grounder.locate("unused-test-image.jpg", "cup")

    def test_reversed_bbox_is_rejected(self):
        client = FixedVLMClient(
            '{"found": true, "label": "cup", '
            '"bbox": [700, 200, 300, 600]}'
        )
        grounder = ObjectGrounder(client)

        with self.assertRaisesRegex(ValueError, "x1 < x2"):
            grounder.locate("unused-test-image.jpg", "cup")

    def test_out_of_range_bbox_is_rejected(self):
        client = FixedVLMClient(
            '{"found": true, "label": "cup", '
            '"bbox": [-10, 200, 300, 600]}'
        )
        grounder = ObjectGrounder(client)

        with self.assertRaisesRegex(ValueError, "between 0 and 1000"):
            grounder.locate("unused-test-image.jpg", "cup")


if __name__ == "__main__":
    unittest.main()