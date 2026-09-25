# EE4705 Project 1.2 | Task 2 - terminal chat
"""An explicit approach request must reach the approach even if the parser chats."""

import json
import os
import unittest
from unittest.mock import patch

# The module builds a Gemini client at import time; no real call is made here.
os.environ.setdefault("GEMINI_API_KEY", "test-key")

try:
    from ee4705_perception import terminal_chat
except ImportError as error:  # terminal_chat needs ROS 2 (rclpy) to import
    raise unittest.SkipTest(f"terminal_chat unavailable: {error}")


CHAT_REPLY = {"action": "chat", "reply": "I cannot do that."}


class ParserOverrideTest(unittest.TestCase):

    def setUp(self):
        terminal_chat.history.clear()
        patches = [
            patch.object(terminal_chat, "log_interaction"),
            patch.object(terminal_chat, "log_turn"),
        ]
        for p in patches:
            p.start()
            self.addCleanup(p.stop)

    def run_turn(self, user_text, parsed):
        def fake_parse(text):
            terminal_chat.history.append({"role": "user", "content": text})
            terminal_chat.history.append(
                {"role": "assistant", "content": json.dumps(parsed)})
            return dict(parsed), 0.1, {"calls": 1}

        with patch.object(terminal_chat, "parse_command", side_effect=fake_parse), \
                patch.object(terminal_chat, "run_approach_command",
                             return_value=("done", {"success": True})) as approach:
            turn = terminal_chat.handle_turn(user_text)
        return turn, approach

    def test_chat_reply_to_move_request_becomes_approach(self):
        turn, approach = self.run_turn("move to the fire hydrant", CHAT_REPLY)

        approach.assert_called_once_with("fire hydrant")
        self.assertEqual(turn["command"],
                         {"action": "approach", "object": "fire hydrant"})
        self.assertTrue(turn["parser_override"])
        self.assertEqual(json.loads(terminal_chat.history[1]["content"]),
                         turn["command"])

    def test_room_request_is_not_overridden(self):
        turn, approach = self.run_turn("go to room 3", CHAT_REPLY)

        approach.assert_not_called()
        self.assertEqual(turn["command"], CHAT_REPLY)
        self.assertFalse(turn["parser_override"])
        self.assertEqual(turn["reply"], CHAT_REPLY["reply"])

    def test_unknown_object_still_reaches_the_approach(self):
        turn, approach = self.run_turn("move to the kitchen", CHAT_REPLY)

        approach.assert_called_once_with("kitchen")
        self.assertTrue(turn["parser_override"])

    def test_parsed_approach_is_not_marked_as_override(self):
        parsed = {"action": "approach", "object": "mailbox"}
        turn, approach = self.run_turn("go to the mailbox", parsed)

        approach.assert_called_once_with("mailbox")
        self.assertFalse(turn["parser_override"])

    def test_target_extraction(self):
        target = terminal_chat.explicit_approach_target
        self.assertEqual(target("Please head over towards the cafe table."),
                         "cafe table")
        self.assertEqual(target("search for the car wheel"), "car wheel")
        self.assertEqual(target("find cinder block"), "cinder block")
        self.assertIsNone(target("go to Room 6"))
        self.assertIsNone(target("what do you see?"))

    def test_pronoun_is_left_to_the_parser(self):
        target = terminal_chat.explicit_approach_target
        self.assertIsNone(target("Approach it"))
        self.assertIsNone(target("move to that one"))

        turn, approach = self.run_turn("Approach it", CHAT_REPLY)

        approach.assert_not_called()
        self.assertFalse(turn["parser_override"])


if __name__ == "__main__":
    unittest.main()
