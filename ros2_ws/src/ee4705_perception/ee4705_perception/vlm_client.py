# EE4705 Project 1.2 | Task 3 - VLM client
# Contributors (from git history): Alexander Likin (5 commits)
"""Small provider interface for sending a saved image to a VLM."""

from __future__ import annotations

import base64
import mimetypes
import os
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Protocol


TRANSIENT_ERROR_MARKERS = ("503", "UNAVAILABLE", "429", "RESOURCE_EXHAUSTED",
                           "overloaded", "high demand", "timed out", "Timeout",
                           "500 INTERNAL", "502", "504", "DEADLINE_EXCEEDED",
                           "Connection")


def is_transient_error(error: Exception) -> bool:
    """True for provider overload/rate-limit/network errors worth retrying."""

    text = f"{type(error).__name__}: {error}"
    return any(marker in text for marker in TRANSIENT_ERROR_MARKERS)


def call_with_retries(call, *, attempts: int = 3, first_delay_s: float = 2.0):
    """Run call(); retry transient provider errors with exponential backoff.

    Returns (result, retries). Latency reported by callers includes the
    retries, because that is what the user waits for.
    """

    for attempt in range(attempts):
        try:
            return call(), attempt
        except Exception as error:
            if attempt == attempts - 1 or not is_transient_error(error):
                error.attempts = attempt + 1  # for API-call accounting
                raise
            time.sleep(first_delay_s * 2**attempt)
    raise AssertionError("unreachable")


@dataclass(frozen=True)
class VLMResponse:
    """Normalized result returned by every supported VLM provider."""

    text: str
    model: str
    latency_s: float
    input_tokens: int | None = None
    output_tokens: int | None = None
    cost_usd: float | None = None
    retries: int = 0


class VLMClient(Protocol):
    """Interface used by the scene-description service."""

    def ask(self, image_path: str | Path, prompt: str) -> VLMResponse:
        """Ask one question about one saved image."""


def image_to_data_url(image_path: str | Path) -> str:
    """Read an image and return an API-ready base64 data URL."""

    path = Path(image_path)
    if not path.is_file():
        raise FileNotFoundError(f"Image does not exist: {path}")

    mime_type, _ = mimetypes.guess_type(path.name)
    if mime_type not in {"image/jpeg", "image/png", "image/webp"}:
        raise ValueError("Use a JPEG, PNG, or WebP image.")

    encoded = base64.b64encode(path.read_bytes()).decode("ascii")
    return f"data:{mime_type};base64,{encoded}"


class MockVLMClient:
    """Offline client for testing the pipeline without spending API credit."""

    model = "mock-vlm"

    def ask(self, image_path: str | Path, prompt: str) -> VLMResponse:
        """Return a clearly labelled demonstration response."""

        image_to_data_url(image_path)
        started = time.perf_counter()
        text = (
            "[MOCK RESPONSE] The image pipeline works. "
            "Connect a real VLM before evaluating scene accuracy."
        )
        return VLMResponse(
            text=text,
            model=self.model,
            latency_s=time.perf_counter() - started,
        )

class GeminiVLMClient:
    """Client for sending saved images to Google Gemini."""

    def __init__(
        self,
        model: str | None = None,
        *,
        api_key_env: str = "GEMINI_API_KEY",
    ) -> None:
        api_key = os.getenv(api_key_env)

        if not api_key:
            raise ValueError(
                f"Environment variable {api_key_env} is not set. "
                "Store the key in the ignored .env file."
            )

        try:
            from google import genai
            from google.genai import types
        except ImportError as error:
            raise RuntimeError(
                "The google-genai package is missing. "
                "Install the dependencies from requirements.txt."
            ) from error

        self._client = genai.Client(
            api_key=api_key,
            http_options=types.HttpOptions(timeout=90_000),  # milliseconds
        )
        self._types = types

        self.model = (
            model
            or os.getenv("GEMINI_VISION_MODEL")
            or os.getenv("GEMINI_MODEL")
            or "gemini-3.5-flash-lite"
        )


    def ask(
        self,
        image_path: str | Path,
        prompt: str,
    ) -> VLMResponse:
        """Send one image and prompt to Gemini."""

        path = Path(image_path)

        if not path.is_file():
            raise FileNotFoundError(
                f"Image does not exist: {path}"
            )

        mime_type, _ = mimetypes.guess_type(path.name)

        if mime_type not in {
            "image/jpeg",
            "image/png",
            "image/webp",
        }:
            raise ValueError(
                "Use a JPEG, PNG, or WebP image."
            )

        image_part = self._types.Part.from_bytes(
            data=path.read_bytes(),
            mime_type=mime_type,
        )

        started = time.perf_counter()

        response, retries = call_with_retries(
            lambda: self._client.models.generate_content(
                model=self.model,
                contents=[
                    prompt,
                    image_part,
                ],
                config=self._types.GenerateContentConfig(
                    temperature=0,
                ),
            )
        )

        latency_s = time.perf_counter() - started
        usage = getattr(response, "usage_metadata", None)

        return VLMResponse(
            text=(getattr(response, "text", "") or "").strip(),
            model=self.model,
            latency_s=latency_s,
            input_tokens=getattr(
                usage,
                "prompt_token_count",
                None,
            ),
            output_tokens=getattr(
                usage,
                "candidates_token_count",
                None,
            ),
            retries=retries,
        )

class OpenAICompatibleVLMClient:
    """Client for services exposing an OpenAI-compatible chat endpoint."""

    def __init__(
        self,
        model: str,
        *,
        api_key_env: str = "OPENAI_API_KEY",
        base_url: str | None = None,
        image_detail: str | None = "low",
    ) -> None:
        api_key = os.getenv(api_key_env)
        if not api_key:
            raise ValueError(
                f"Environment variable {api_key_env} is not set. "
                "Do not put the key inside source code."
            )

        try:
            from openai import OpenAI
        except ImportError as error:
            raise RuntimeError(
                "The openai package is missing. Install it with: pip install openai"
            ) from error

        # Bounded, so a stalled request cannot hang the robot loop; transient
        # failures are retried by call_with_retries instead of the SDK.
        client_options = {"api_key": api_key, "timeout": 90.0, "max_retries": 0}
        if base_url:
            client_options["base_url"] = base_url

        self._client = OpenAI(**client_options)
        self.model = model
        self.image_detail = image_detail

    def ask(self, image_path: str | Path, prompt: str) -> VLMResponse:
        """Send a multimodal chat request and normalize the response."""

        image_url = {"url": image_to_data_url(image_path)}
        if self.image_detail:
            image_url["detail"] = self.image_detail

        started = time.perf_counter()
        response, retries = call_with_retries(
            lambda: self._client.chat.completions.create(
                model=self.model,
                temperature=0,
                messages=[
                    {
                        "role": "user",
                        "content": [
                            {"type": "text", "text": prompt},
                            {"type": "image_url", "image_url": image_url},
                        ],
                    }
                ],
            )
        )
        latency_s = time.perf_counter() - started
        text = response.choices[0].message.content or ""
        usage = getattr(response, "usage", None)

        return VLMResponse(
            text=text.strip(),
            model=self.model,
            latency_s=latency_s,
            input_tokens=getattr(usage, "prompt_tokens", None),
            output_tokens=getattr(usage, "completion_tokens", None),
            retries=retries,
        )
class QwenVLMClient(OpenAICompatibleVLMClient):
    """Qwen vision client using Alibaba's OpenAI-compatible API."""

    DEFAULT_MODEL = "qwen3-vl-plus"
    DEFAULT_BASE_URL = (
        "https://dashscope-intl.aliyuncs.com/"
        "compatible-mode/v1"
    )

    def __init__(
        self,
        model: str | None = None,
        *,
        api_key_env: str = "QWEN_API_KEY",
        base_url: str | None = None,
    ) -> None:
        """Configure Qwen from environment variables or explicit values."""

        selected_model = (
            model
            or os.getenv("QWEN_MODEL")
            or self.DEFAULT_MODEL
        )

        selected_base_url = (
            base_url
            or os.getenv("QWEN_BASE_URL")
            or self.DEFAULT_BASE_URL
        )

        super().__init__(
            selected_model,
            api_key_env=api_key_env,
            base_url=selected_base_url,
            image_detail=None,
        )