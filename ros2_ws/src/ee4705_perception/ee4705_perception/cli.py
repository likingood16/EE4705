"""Command-line demonstration for one saved image."""

from __future__ import annotations

import argparse
from pathlib import Path

from .result_logger import append_trial
from .scene_describer import DESCRIPTION_PROMPT, SceneDescriber
from .vlm_client import MockVLMClient, OpenAICompatibleVLMClient


def build_parser() -> argparse.ArgumentParser:
    """Create the command-line argument parser."""

    parser = argparse.ArgumentParser(
        description="Describe an image or answer a visual question."
    )
    parser.add_argument("--image", required=True, help="Path to a JPEG, PNG, or WebP file")
    parser.add_argument("--question", help="Optional visual question about the image")
    parser.add_argument(
        "--provider",
        choices=["mock", "openai-compatible"],
        default="mock",
        help="Use mock first; use openai-compatible for a real API call",
    )
    parser.add_argument("--model", help="Required model name for a real API call")
    parser.add_argument("--base-url", help="Optional compatible provider endpoint")
    parser.add_argument("--api-key-env", default="OPENAI_API_KEY")
    parser.add_argument("--image-detail", default="low")
    parser.add_argument("--log", help="Optional output CSV path")
    parser.add_argument("--trial-id", default="prototype-001")
    parser.add_argument("--scene-id", default="prototype-scene")
    return parser


def make_client(arguments: argparse.Namespace):
    """Construct the requested provider client."""

    if arguments.provider == "mock":
        return MockVLMClient()
    if not arguments.model:
        raise ValueError("--model is required with --provider openai-compatible")
    return OpenAICompatibleVLMClient(
        arguments.model,
        api_key_env=arguments.api_key_env,
        base_url=arguments.base_url,
        image_detail=arguments.image_detail,
    )


def main(argv: list[str] | None = None) -> int:
    """Run one scene-description or visual-question trial."""

    arguments = build_parser().parse_args(argv)
    image_path = Path(arguments.image)
    service = SceneDescriber(make_client(arguments))

    if arguments.question:
        response = service.answer(image_path, arguments.question)
        logged_question = arguments.question
    else:
        response = service.describe(image_path)
        logged_question = DESCRIPTION_PROMPT

    print(f"Model: {response.model}")
    print(f"Latency: {response.latency_s:.4f} seconds")
    print(f"Answer: {response.text}")

    if arguments.log:
        append_trial(
            arguments.log,
            trial_id=arguments.trial_id,
            scene_id=arguments.scene_id,
            image_path=image_path,
            question=logged_question,
            response=response,
        )
        print(f"Saved trial to: {arguments.log}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
