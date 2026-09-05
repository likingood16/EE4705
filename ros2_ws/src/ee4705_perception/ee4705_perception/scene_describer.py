"""Prompts and service methods for Task 3 scene understanding."""

from __future__ import annotations

from pathlib import Path

from .vlm_client import VLMClient, VLMResponse


DESCRIPTION_PROMPT = """You are the eyes of an indoor mobile robot.
Describe only objects that are clearly visible in the image.
Mention useful spatial relationships, colours, and approximate counts.
Do not guess or invent hidden objects. Answer in two or three concise sentences."""

QUESTION_PROMPT = """You are answering a question about the robot's current camera image.
Use only visible evidence. If the answer cannot be determined, say that clearly.
Question: {question}"""


class SceneDescriber:
    """Provide scene descriptions and visual question answering through one client."""

    def __init__(self, client: VLMClient) -> None:
        self.client = client

    def describe(self, image_path: str | Path) -> VLMResponse:
        """Describe one saved camera image."""

        return self.client.ask(image_path, DESCRIPTION_PROMPT)

    def answer(self, image_path: str | Path, question: str) -> VLMResponse:
        """Answer one follow-up question about a saved camera image."""

        cleaned_question = question.strip()
        if not cleaned_question:
            raise ValueError("The visual question cannot be empty.")
        return self.client.ask(
            image_path,
            QUESTION_PROMPT.format(question=cleaned_question),
        )
