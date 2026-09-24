import csv
import tempfile
import unittest
from pathlib import Path

from ee4705_perception.approach_result_logger import (
    ApproachTrial,
    append_approach_trial,
)


class ApproachLoggerTests(unittest.TestCase):

    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.csv_path = Path(self.directory.name) / "trials.csv"

    def read_rows(self):
        with self.csv_path.open(newline="", encoding="utf-8") as source:
            return list(csv.DictReader(source))

    def test_unknown_results_remain_blank(self):
        append_approach_trial(
            self.csv_path,
            ApproachTrial(trial_id="mock-001", target="cup"),
        )

        row = self.read_rows()[0]
        self.assertEqual(row["mode"], "mock")
        self.assertEqual(row["approach_success"], "")
        self.assertEqual(row["final_distance_m"], "")
        self.assertEqual(row["cost_usd"], "")

    def test_appending_preserves_previous_trial(self):
        for trial_id in ("mock-001", "mock-002"):
            append_approach_trial(
                self.csv_path,
                ApproachTrial(
                    trial_id=trial_id,
                    target="cup",
                    grounding_correct=False,
                ),
            )

        rows = self.read_rows()
        self.assertEqual(len(rows), 2)
        self.assertEqual(rows[0]["trial_id"], "mock-001")
        self.assertEqual(rows[1]["grounding_correct"], "false")

    def test_duplicate_trial_is_rejected(self):
        trial = ApproachTrial(trial_id="mock-001", target="cup")
        append_approach_trial(self.csv_path, trial)

        with self.assertRaisesRegex(ValueError, "already exists"):
            append_approach_trial(self.csv_path, trial)

        self.assertEqual(len(self.read_rows()), 1)

    def test_incompatible_header_is_not_modified(self):
        original = "unrelated,column\n1,2\n"
        self.csv_path.write_text(original, encoding="utf-8")

        with self.assertRaisesRegex(ValueError, "header"):
            append_approach_trial(
                self.csv_path,
                ApproachTrial(trial_id="mock-001", target="cup"),
            )

        self.assertEqual(
            self.csv_path.read_text(encoding="utf-8"),
            original,
        )

    def test_invalid_measurements_are_rejected(self):
        for options in (
            {"elapsed_s": -1.0},
            {"cost_usd": float("nan")},
            {"api_calls": -1},
            {"approach_success": "yes"},
            {"final_distance_m": 0.4},  # Measurement method missing.
        ):
            with self.subTest(options=options):
                with self.assertRaises(ValueError):
                    ApproachTrial(
                        trial_id="mock-001",
                        target="cup",
                        **options,
                    )


if __name__ == "__main__":
    unittest.main()