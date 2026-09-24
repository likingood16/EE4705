"""Multi-turn terminal chat: the robot's dialogue manager (Tasks 2 and 5).

Every user turn is parsed by an LLM into one JSON command, which is then
executed: navigate to a room (and describe it on arrival), describe the
view, answer a visual question, approach an object, stop, or just reply.

handle_turn() runs one complete turn and returns the reply together with
per-stage results, latencies and API usage, so the end-to-end evaluation
(evaluation/run_end_to_end_trials.py) drives exactly the same pipeline as
the interactive chat.
"""

import csv
import json
import os
import time
from datetime import datetime
from pathlib import Path

import rclpy

from google import genai
from google.genai import types

from ee4705_perception.back_away import back_away_if_blocked
from ee4705_perception.camera_snapshot import capture_one_frame
from ee4705_perception.goto_room import GotoRoom
from ee4705_perception.scene_describer import SceneDescriber
from ee4705_perception.vlm_client import (
    GeminiVLMClient,
    QwenVLMClient,
    call_with_retries,
)


# ============================================================
# PORTABLE PROJECT PATHS
# ============================================================

PROJECT_ROOT = Path(
    os.environ.get(
        "EE4705_ROOT",
        Path(__file__).resolve().parents[4],
    )
)

CURRENT_CAMERA_IMAGE = (
    PROJECT_ROOT
    / "evaluation"
    / "scenes"
    / "current_camera.jpg"
)

APPROACH_EVIDENCE_DIR = (
    PROJECT_ROOT
    / "evaluation"
    / "task4_evidence"
    / "chat"
)

LOG_DIR = PROJECT_ROOT / "logs"
LOG_FILE = LOG_DIR / "terminal_chat_log.csv"
# One JSON object per turn with per-stage latency and API usage.
TURN_LOG_FILE = LOG_DIR / "terminal_chat_turns.jsonl"

LOG_DIR.mkdir(
    parents=True,
    exist_ok=True,
)

CURRENT_CAMERA_IMAGE.parent.mkdir(
    parents=True,
    exist_ok=True,
)


# ============================================================
# COMMAND-PARSER CONFIGURATION
# ============================================================

# Both parsers scored 20/20 on the Task 2 utterances (Gemini 0.73 s, Qwen
# 0.74 s mean), but on the evaluation days Gemini took 20-40 s per call and
# returned 503/504 overload errors, so Qwen is the default (see
# docs/results_summary.md). When the primary parser fails, the same prompt is
# sent to the other provider so a turn is not lost.
GEMINI_MODEL = os.getenv(
    "GEMINI_MODEL",
    "gemini-3.5-flash-lite",
)

QWEN_PARSER_MODEL = os.getenv(
    "QWEN_PARSER_MODEL",
    "qwen-plus",
)

# Primary parser ("gemini" or "qwen"); the other one is the fallback.
PARSER_PROVIDER = os.getenv("PARSER_PROVIDER", "qwen").strip().lower()

PARSER_FALLBACK = os.getenv("PARSER_FALLBACK", "1") == "1"

client = genai.Client(
    api_key=os.getenv("GEMINI_API_KEY"),
    http_options=types.HttpOptions(timeout=90_000),  # milliseconds
)

qwen_text_client = None

# Keep the prompt bounded: the last N history entries (3 per turn).
MAX_HISTORY_ENTRIES = 36


# ============================================================
# VISION MODELS
# ============================================================

vision_describer = None
grounding_client = None


def make_vlm_client(provider):
    """Create a Gemini or Qwen vision client."""

    provider = provider.strip().lower()

    if provider == "gemini":
        return GeminiVLMClient()

    if provider == "qwen":
        return QwenVLMClient()

    raise ValueError(
        "The provider must be 'gemini' or 'qwen'."
    )


def get_vision_describer():
    """Create the selected scene-description client only when requested."""

    global vision_describer

    if vision_describer is None:
        # Qwen: 10x lower latency than Gemini in the Task 3 comparison, no
        # failed calls, same hallucination count (Gemini named more objects).
        vision_describer = SceneDescriber(
            make_vlm_client(os.getenv("VISION_PROVIDER", "qwen"))
        )

    return vision_describer


def get_grounding_client():
    """Qwen-VL is trained for grounding, so it is the default here."""

    global grounding_client

    if grounding_client is None:
        grounding_client = make_vlm_client(
            os.getenv("GROUNDING_PROVIDER", "qwen")
        )

    return grounding_client


def ask_current_view(question=None):
    """Capture the latest frame and send it to the selected vision client."""

    saved_image = capture_one_frame(
        CURRENT_CAMERA_IMAGE,
        timeout_s=10.0,
    )

    describer = get_vision_describer()

    if question:
        return describer.answer(
            saved_image,
            question,
        )

    return describer.describe(
        saved_image
    )


# ============================================================
# SYSTEM PROMPT
# ============================================================

SYSTEM_PROMPT = """
You are the dialogue manager of an indoor TurtleBot3 robot.

The house contains exactly six valid rooms:
Room 1, Room 2, Room 3, Room 4, Room 5, Room 6.

For EVERY user message, return ONLY one JSON object.

Allowed commands:

1. Navigate to a room:
{"action": "goto_room", "room": 1}

The room value must be an integer from 1 to 6.

Understand natural paraphrases such as:
- "Go to Room 3"
- "Could you check room three please?"
- "Head over to the third room"
- "Navigate to room number five"

2. Describe the current scene:
{"action": "describe"}

Use this when the user requests a general description, such as:
- "Describe what you can see"
- "What is around you?"
- "Look around and describe the room"

3. Answer a visual question:
{"action": "visual_question", "question": "Is there anything on the floor?"}

Use this when the user asks a specific question about the
current camera view, including follow-up questions such as:
- "What colour is the wall?"
- "Is there anything on the floor?"
- "Can you see a fire hydrant?"
- "What is on the left?"
- "What colour is it?"

Copy the user's visual question into the "question" field.

4. Approach an object:
{"action": "approach", "object": "cup"}

5. Stop:
{"action": "stop"}

6. General conversation or clarification:
{"action": "chat", "reply": "your response"}

If the user requests an invalid room such as Room 99,
return a chat action explaining that only Rooms 1 to 6 exist.

If the request is unclear,
ask for clarification using the chat action.

Unsupported locomotion rule:

The robot is a wheeled ground robot.

If the user requests unsupported locomotion such as:
- fly
- jump
- climb
- swim
- teleport

do NOT convert the request into goto_room,
even if the message contains a valid room number.

Use the "chat" action to explain that the robot
can only navigate on the ground using its wheels.

Example:

User:
"Fly to Room 2"

Correct output:
{"action": "chat", "reply": "I cannot fly. I can only navigate on the ground to Rooms 1 through 6."}

Do NOT output:
{"action": "goto_room", "room": 2}

Memory / reminder rule:

If the user says to remember, note, keep in mind,
or store some information, do NOT execute an action
mentioned inside that information unless the user
explicitly asks the robot to perform it now.

Use the "chat" action to acknowledge the information.

Example:

User:
"Remember that I want to visit Room 4."

Correct output:
{"action": "chat", "reply": "Okay, I'll remember that you want to visit Room 4."}

Do NOT output:
{"action": "goto_room", "room": 4}

The remembered information should remain available
through the conversation history so that later
follow-up questions can refer to it.

Use the conversation history to interpret follow-up requests.
"""


# ============================================================
# CHAT HISTORY
# ============================================================

history = []


def log_interaction(
    user_text,
    command,
    reply,
    latency,
    navigation_success=None
):

    file_exists = os.path.isfile(
        LOG_FILE
    )

    with open(
        LOG_FILE,
        "a",
        newline=""
    ) as file:

        writer = csv.writer(
            file
        )

        if not file_exists:

            writer.writerow([
                "timestamp",
                "user_input",
                "action",
                "room",
                "object",
                "robot_reply",
                "parser_latency_s",
                "navigation_success"
            ])

        writer.writerow([
            datetime.now().isoformat(
                timespec="seconds"
            ),
            user_text,
            command.get(
                "action",
                ""
            ),
            command.get(
                "room",
                ""
            ),
            command.get(
                "object",
                ""
            ),
            reply,
            round(
                latency,
                3
            ),
            navigation_success
        ])


def log_turn(turn):
    """Append the full per-stage record of one turn to the JSONL log."""

    with open(TURN_LOG_FILE, "a", encoding="utf-8") as file:
        file.write(json.dumps(turn) + "\n")


# ============================================================
# BUILD MULTI-TURN PROMPT
# ============================================================

def build_prompt(
    user_text
):

    conversation = ""

    for item in history[-MAX_HISTORY_ENTRIES:]:

        conversation += (
            f"{item['role'].upper()}: "
            f"{item['content']}\n"
        )

    conversation += (
        f"USER: {user_text}\n"
    )

    return conversation


# ============================================================
# COMMAND PARSER
# ============================================================

def _parse_with_gemini(prompt):
    """Return (JSON text, input tokens, output tokens, retries) from Gemini."""

    # With a fallback available, retry once only: Gemini overload errors
    # took 15-25 s each to arrive during testing.
    response, retries = call_with_retries(
        lambda: client.models.generate_content(
            model=GEMINI_MODEL,
            contents=prompt,
            config=types.GenerateContentConfig(
                system_instruction=SYSTEM_PROMPT,
                response_mime_type="application/json",
                temperature=0.1,
            ),
        ),
        attempts=2 if PARSER_FALLBACK else 3,
    )

    usage = getattr(response, "usage_metadata", None)

    return (
        response.text.strip(),
        getattr(usage, "prompt_token_count", None),
        getattr(usage, "candidates_token_count", None),
        retries,
    )


def _parse_with_qwen(prompt):
    """Return (JSON text, input tokens, output tokens, retries) from Qwen."""

    global qwen_text_client

    if qwen_text_client is None:
        from openai import OpenAI

        qwen_text_client = OpenAI(
            api_key=os.getenv("QWEN_API_KEY"),
            base_url=os.getenv(
                "QWEN_BASE_URL",
                "https://dashscope-intl.aliyuncs.com/compatible-mode/v1",
            ),
            timeout=60.0,
            max_retries=0,
        )

    response, retries = call_with_retries(
        lambda: qwen_text_client.chat.completions.create(
            model=QWEN_PARSER_MODEL,
            temperature=0.1,
            response_format={"type": "json_object"},
            messages=[
                {"role": "system", "content": SYSTEM_PROMPT},
                {"role": "user", "content": prompt},
            ],
        )
    )

    usage = getattr(response, "usage", None)

    return (
        (response.choices[0].message.content or "").strip(),
        getattr(usage, "prompt_tokens", None),
        getattr(usage, "completion_tokens", None),
        retries,
    )


def parse_command(
    user_text
):
    """Parse one utterance; returns (command, latency_s, stage record)."""

    prompt = build_prompt(
        user_text
    )

    parsers = [
        ("gemini", GEMINI_MODEL, _parse_with_gemini),
        ("qwen", QWEN_PARSER_MODEL, _parse_with_qwen),
    ]

    if PARSER_PROVIDER == "qwen":
        parsers.reverse()

    if not PARSER_FALLBACK:
        parsers = parsers[:1]

    stage = {"calls": 0, "input_tokens": 0, "output_tokens": 0, "errors": []}
    command = None
    start_time = time.time()

    for provider, model, parse in parsers:

        stage["calls"] += 1

        try:

            output, tokens_in, tokens_out, retries = parse(prompt)
            stage["calls"] += retries
            stage["input_tokens"] += tokens_in or 0
            stage["output_tokens"] += tokens_out or 0

            command = json.loads(
                output
            )

            if not isinstance(command, dict):
                raise ValueError("The parser did not return a JSON object.")

            stage["model"] = model
            break

        except Exception as error:

            stage["calls"] += getattr(error, "attempts", 1) - 1

            print(
                f"[Parser error ({provider}): {error}]"
            )

            stage["errors"].append(f"{provider}: {error}"[:300])

    latency = (
        time.time()
        - start_time
    )

    if command is None:

        command = {
            "action": "chat",
            "reply": (
                "Sorry, I could not understand "
                "that request. Please try again."
            )
        }

    stage["latency_s"] = round(latency, 3)

    # Save user turn
    history.append({
        "role": "user",
        "content": user_text
    })

    # Save the parsed command
    history.append({
        "role": "assistant",
        "content": json.dumps(
            command
        )
    })

    return command, latency, stage


# ============================================================
# NAVIGATION
# ============================================================

def navigate_to_room(
    room_number
):

    room_name = (
        f"room{room_number}"
    )

    print(
        f"Robot: Navigating to "
        f"Room {room_number}..."
    )

    # After an object approach the robot can be too close to the object
    # for Nav2 to plan; back away first (see back_away.py).
    back_away_if_blocked()

    node = GotoRoom(
        room_name
    )

    while (
        rclpy.ok()
        and not node.finished
    ):

        rclpy.spin_once(
            node,
            timeout_sec=0.1
        )

    success = (
        node.success
    )

    node.destroy_node()

    return success


# ============================================================
# ONE TURN: PARSE AND EXECUTE
# ============================================================

def _vision_stage(response, started):
    return {
        "model": response.model,
        "latency_s": round(time.time() - started, 3),
        "calls": 1 + response.retries,
        "input_tokens": response.input_tokens,
        "output_tokens": response.output_tokens,
        "text": response.text,
    }


def run_approach_command(obj):
    """Run the Task 4 approach controller; returns (reply, stage record)."""

    from ee4705_perception.approach_robot import run_approach
    from ee4705_perception.goto_room import navigate_to_pose

    enable_motion = os.getenv("TASK4_ENABLE_MOTION", "1") == "1"

    started = time.time()

    # Navigation is synchronous, so Nav2 has no active goal here and the
    # approach controller is the only node commanding /cmd_vel.
    result = run_approach(
        obj,
        get_grounding_client(),
        enable_motion=enable_motion,
        exclusive_control=enable_motion,
        # Verified in Gazebo: the simulated LDS reports +inf for no return.
        inf_is_clear=True,
        evidence_dir=APPROACH_EVIDENCE_DIR,
        # Lets Nav2 plan around an obstacle between the robot and the object.
        navigate=navigate_to_pose if enable_motion else None,
    )

    stage = {
        "success": result.visual_candidate,
        "reason": result.reason,
        "latency_s": round(time.time() - started, 3),
        "calls": result.api_calls,
        "input_tokens": result.input_tokens,
        "output_tokens": result.output_tokens,
        "initially_visible": result.initially_visible,
        "final_range_m": result.final_range_m,
        "search_rotation_deg": round(result.search_rotation_deg, 1),
        "evidence_dir": result.evidence_dir,
        "model": getattr(get_grounding_client(), "model", ""),
    }

    return result.reply, stage


def handle_turn(user_text):
    """Parse and execute one user turn; returns the full turn record."""

    command, latency, parse_stage = parse_command(
        user_text
    )

    action = command.get(
        "action"
    )

    turn = {
        "timestamp": datetime.now().isoformat(timespec="seconds"),
        "user": user_text,
        "command": command,
        "parse": parse_stage,
        "navigation": None,
        "vision": None,
        "approach": None,
    }

    navigation_success = None

    # ------------------------------------------
    # GOTO ROOM (then describe what is there)
    # ------------------------------------------

    if action == "goto_room":

        room = command.get(
            "room"
        )

        if (
            not isinstance(
                room,
                int
            )
            or room < 1
            or room > 6
        ):

            reply = (
                "That room is invalid. "
                "Please choose Room 1 "
                "to Room 6."
            )

        else:

            started = time.time()

            success = navigate_to_room(
                room
            )

            navigation_success = (
                success
            )

            turn["navigation"] = {
                "success": success,
                "room": room,
                "latency_s": round(time.time() - started, 3),
            }

            if success:

                reply = (
                    f"I have arrived "
                    f"in Room {room}."
                )

                started = time.time()

                try:

                    print(
                        "Robot: Looking around..."
                    )

                    vision_response = ask_current_view()
                    turn["vision"] = _vision_stage(vision_response, started)

                    if vision_response.text:
                        reply += " " + vision_response.text

                except Exception as error:

                    print(
                        f"[Vision error: {error}]"
                    )

                    turn["vision"] = {"error": str(error)[:300],
                                      "latency_s": round(time.time() - started, 3)}

            else:

                reply = (
                    f"Sorry, I could not "
                    f"reach Room {room}."
                )

    # ------------------------------------------
    # DESCRIBE CURRENT CAMERA VIEW
    # ------------------------------------------

    elif action == "describe":

        started = time.time()

        try:

            print(
                "Robot: Capturing the current "
                "camera view..."
            )

            vision_response = ask_current_view()
            turn["vision"] = _vision_stage(vision_response, started)

            reply = (
                vision_response.text
                or
                "The vision model returned "
                "an empty description."
            )

        except Exception as error:

            print(
                f"[Vision error: {error}]"
            )

            turn["vision"] = {"error": str(error)[:300],
                              "latency_s": round(time.time() - started, 3)}

            reply = (
                "Sorry, I could not describe "
                "the current camera view."
            )

    # ------------------------------------------
    # VISUAL QUESTION
    # ------------------------------------------

    elif action == "visual_question":

        question = command.get(
            "question",
            user_text,
        )

        started = time.time()

        try:

            print(
                "Robot: Checking the current "
                "camera view..."
            )

            vision_response = ask_current_view(
                question
            )
            turn["vision"] = _vision_stage(vision_response, started)

            reply = (
                vision_response.text
                or
                "The vision model returned "
                "an empty answer."
            )

        except Exception as error:

            print(
                f"[Vision error: {error}]"
            )

            turn["vision"] = {"error": str(error)[:300],
                              "latency_s": round(time.time() - started, 3)}

            reply = (
                "Sorry, I could not answer "
                "that visual question."
            )

    # ------------------------------------------
    # APPROACH AN OBJECT
    # ------------------------------------------

    elif action == "approach":

        obj = command.get(
            "object"
        )

        if not isinstance(obj, str) or not obj.strip():

            reply = "Which object should I approach?"

        else:

            print(
                f"Robot: Looking for the {obj.strip()}..."
            )

            try:

                reply, turn["approach"] = run_approach_command(
                    obj.strip()
                )

            except Exception as error:

                print(
                    f"[Approach error: {error}]"
                )

                turn["approach"] = {"success": False, "reason": "start_error",
                                    "error": str(error)[:300]}

                reply = (
                    f"Sorry, I could not start "
                    f"approaching the {obj.strip()}."
                )

    # ------------------------------------------
    # STOP
    # ------------------------------------------

    elif action == "stop":

        # Commands run to completion before the next input is read, so
        # nothing is moving when a typed stop arrives.
        reply = (
            "I have stopped and am not moving."
        )

    # ------------------------------------------
    # CHAT / CLARIFICATION
    # ------------------------------------------

    elif action == "chat":

        reply = command.get(
            "reply",
            "Could you clarify "
            "your request?"
        )

    # ------------------------------------------
    # UNKNOWN ACTION
    # ------------------------------------------

    else:

        reply = (
            "I could not determine "
            "the requested action. "
            "Please try again."
        )

    turn["reply"] = reply

    log_interaction(
        user_text=user_text,
        command=command,
        reply=reply,
        latency=latency,
        navigation_success=(
            navigation_success
        )
    )

    log_turn(turn)

    # Save the actual robot reply so follow-ups can refer to it.
    history.append({
        "role": "assistant",
        "content": reply
    })

    return turn


# ============================================================
# MAIN MULTI-TURN CHAT LOOP
# ============================================================

def main():

    rclpy.init()

    print("")
    print(
        "=========================================="
    )
    print(
        "   EE4705 TurtleBot3 Terminal Assistant"
    )
    print(
        "=========================================="
    )
    print("")
    print(
        "Type a command and press ENTER."
    )
    print(
        "Type 'exit' to close the program."
    )
    print("")

    while rclpy.ok():

        try:

            user_text = input(
                "You: "
            ).strip()

        except (
            KeyboardInterrupt,
            EOFError
        ):

            print(
                "\nRobot: Goodbye."
            )

            break

        if not user_text:

            continue

        if user_text.lower() in [
            "exit",
            "quit"
        ]:

            print(
                "Robot: Goodbye."
            )

            break

        turn = handle_turn(
            user_text
        )

        # ------------------------------------------
        # DISPLAY FINAL RESPONSE AND TIMING
        # ------------------------------------------

        print(
            f"Robot: {turn['reply']}"
        )

        print(
            f"[Parser: {turn['parse'].get('model', 'failed')}, "
            f"{turn['parse']['latency_s']:.2f} s]"
        )

        if turn["navigation"]:

            print(
                f"[Navigation: {turn['navigation']['latency_s']:.1f} s]"
            )

        if turn["vision"] and "model" in turn["vision"]:

            print(
                f"[Vision: {turn['vision']['model']}, "
                f"{turn['vision']['latency_s']:.2f} s]"
            )

        if turn["approach"] and "latency_s" in turn["approach"]:

            print(
                f"[Approach: {turn['approach']['reason']}, "
                f"{turn['approach']['calls']} VLM calls, "
                f"{turn['approach']['latency_s']:.1f} s]"
            )

        print("")

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
