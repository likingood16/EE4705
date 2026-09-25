# EE4705 Project 1.2 | Task 3 - VLM client
# Contributors (from git history): Alexander Likin (1 commit)
"""Transient provider errors are retried; other errors are raised at once."""

import unittest
from unittest.mock import patch

from ee4705_perception import vlm_client
from ee4705_perception.vlm_client import call_with_retries, is_transient_error


class RetryTests(unittest.TestCase):
    def setUp(self):
        patcher = patch.object(vlm_client.time, "sleep", lambda seconds: None)
        patcher.start()
        self.addCleanup(patcher.stop)

    def test_overload_is_transient(self):
        self.assertTrue(is_transient_error(RuntimeError(
            "503 UNAVAILABLE. This model is currently experiencing high demand.")))
        self.assertTrue(is_transient_error(RuntimeError("504 DEADLINE_EXCEEDED")))
        self.assertFalse(is_transient_error(ValueError("bbox must contain four numbers")))

    def test_retries_then_succeeds(self):
        attempts = []

        def call():
            attempts.append(1)
            if len(attempts) < 3:
                raise RuntimeError("503 UNAVAILABLE")
            return "ok"

        self.assertEqual(call_with_retries(call), ("ok", 2))

    def test_gives_up_after_limit(self):
        def call():
            raise RuntimeError("429 RESOURCE_EXHAUSTED")

        with self.assertRaises(RuntimeError):
            call_with_retries(call, attempts=2)

    def test_permanent_error_is_not_retried(self):
        attempts = []

        def call():
            attempts.append(1)
            raise ValueError("404 NOT_FOUND model")

        with self.assertRaises(ValueError):
            call_with_retries(call)
        self.assertEqual(len(attempts), 1)


if __name__ == "__main__":
    unittest.main()
