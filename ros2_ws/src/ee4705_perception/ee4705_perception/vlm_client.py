"""Small provider interface for sending a saved image to a VLM."""

from __future__ import annotations

import base64
import mimetypes
import os
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Protocol


@dataclass(frozen=True)
class VLMResponse:
    """Normalized result returned by every supported VLM provider."""

    text: str
    model: str
    latency_s: float
    input_tokens: int | None = None
    output_tokens: int | None = None
    cost_usd: float | None = None


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

        client_options = {"api_key": api_key}
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
        response = self._client.chat.completions.create(
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
        latency_s = time.perf_counter() - started
        text = response.choices[0].message.content or ""
        usage = getattr(response, "usage", None)

        return VLMResponse(
            text=text.strip(),
            model=self.model,
            latency_s=latency_s,
            input_tokens=getattr(usage, "prompt_tokens", None),
            output_tokens=getattr(usage, "completion_tokens", None),
        )
