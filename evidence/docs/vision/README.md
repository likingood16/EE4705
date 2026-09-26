# Task 3: VLM scene understanding

## Scope

Task 3 connects the TurtleBot3 camera to a vision-language model (VLM) so the robot can:

1. Describe its current camera view.
2. Answer follow-up questions about the same view.
3. Compare two real VLM services using identical images and prompts.
4. Record accuracy, hallucinations, latency, token usage, and estimated cost.

## Implemented providers

The perception package supports:

- Google Gemini
- Alibaba Qwen
- Generic OpenAI-compatible services
- An offline mock provider for testing without API usage

The default models are configured in the local `.env` file:

```dotenv
GEMINI_MODEL=gemini-3.5-flash-lite
QWEN_MODEL=qwen3-vl-plus
QWEN_BASE_URL=https://dashscope-intl.aliyuncs.com/compatible-mode/v1
VISION_PROVIDER=gemini