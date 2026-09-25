# EE4705 Project 1.2 | Task 3 - VLM client
# Contributors (from git history): Alexander Likin (1 commit)
import os
import unittest
from unittest.mock import patch

from ee4705_perception.cli import (
    build_parser,
    make_client,
)
from ee4705_perception.vlm_client import (
    OpenAICompatibleVLMClient,
    QwenVLMClient,
)


class QwenVLMTests(unittest.TestCase):
    @patch.object(
        OpenAICompatibleVLMClient,
        "__init__",
        return_value=None,
    )
    def test_qwen_uses_safe_defaults(
        self,
        parent_initialiser,
    ):
        with patch.dict(os.environ, {}, clear=True):
            QwenVLMClient()

        positional, keywords = (
            parent_initialiser.call_args
        )

        self.assertEqual(
            positional,
            ("qwen3-vl-plus",),
        )
        self.assertEqual(
            keywords["api_key_env"],
            "QWEN_API_KEY",
        )
        self.assertEqual(
            keywords["base_url"],
            (
                "https://dashscope-intl.aliyuncs.com/"
                "compatible-mode/v1"
            ),
        )
        self.assertIsNone(
            keywords["image_detail"]
        )

    @patch.object(
        OpenAICompatibleVLMClient,
        "__init__",
        return_value=None,
    )
    def test_qwen_accepts_environment_overrides(
        self,
        parent_initialiser,
    ):
        environment = {
            "QWEN_MODEL": "custom-qwen-model",
            "QWEN_BASE_URL": (
                "https://example.test/v1"
            ),
        }

        with patch.dict(
            os.environ,
            environment,
            clear=True,
        ):
            QwenVLMClient()

        positional, keywords = (
            parent_initialiser.call_args
        )

        self.assertEqual(
            positional,
            ("custom-qwen-model",),
        )
        self.assertEqual(
            keywords["base_url"],
            "https://example.test/v1",
        )

    @patch(
        "ee4705_perception.cli.QwenVLMClient"
    )
    def test_cli_selects_qwen(
        self,
        qwen_client,
    ):
        arguments = build_parser().parse_args(
            [
                "--image",
                "room.jpg",
                "--provider",
                "qwen",
            ]
        )

        selected_client = make_client(arguments)

        qwen_client.assert_called_once_with(
            None,
            base_url=None,
        )
        self.assertIs(
            selected_client,
            qwen_client.return_value,
        )


if __name__ == "__main__":
    unittest.main()