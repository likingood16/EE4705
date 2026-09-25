import csv
import json
import os
import time
from datetime import datetime
from pathlib import Path

import rclpy

from google import genai
from google.genai import types

from ee4705_perception.approach_controller import approach_object
from ee4705_perception.camera_snapshot import capture_one_frame
from ee4705_perception.goto_room import GotoRoom
from ee4705_perception.scene_describer import SceneDescriber
from ee4705_perception.vlm_client import GeminiVLMClient


# ============================================================
# PORTABLE PROJECT PATHS
# ============================================================

PROJECT_ROOT = Path(
    os.environ.get(
        "EE4705_ROOT",
        Path.home() / "EE4705",
    )
)

CURRENT_CAMERA_IMAGE = (
    PROJECT_ROOT
    / "evaluation"
    / "scenes"
    / "current_camera.jpg"
)

LOG_DIR = PROJECT_ROOT / "logs"
LOG_FILE = LOG_DIR / "terminal_chat_log.csv"

LOG_DIR.mkdir(
    parents=True,
    exist_ok=True,
)

CURRENT_CAMERA_IMAGE.parent.mkdir(
    parents=True,
    exist_ok=True,
)


# ============================================================
# GEMINI COMMAND-PARSER CONFIGURATION
# ============================================================

GEMINI_MODEL = os.getenv(
    "GEMINI_MODEL",
    "gemini-3.5-flash-lite",
)

client = genai.Client(
    api_key=os.getenv("GEMINI_API_KEY")
)


# ============================================================
# VISION MODEL
# ============================================================

vision_describer = None


def get_vision_describer():
    """Create the vision client only when vision is requested."""

    global vision_describer

    if vision_describer is None:

        vision_describer = SceneDescriber(
            GeminiVLMClient()
        )

    return vision_describer


def ask_current_view(
    question=None,
):
    """Capture the latest frame and send it to Gemini."""

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

Use this when the user asks the robot to physically approach
or move toward a visible object.

Examples:
- "Approach the tire"
- "Move toward the cup"
- "Go closer to the fire hydrant"
- "Drive up to the chair"

Pronoun resolution for object approach:

When the user uses a pronoun such as:
- "it"
- "that"
- "that object"
- "this object"

use the conversation history and the most recent visual
description to resolve the pronoun to a concrete object name.

For example:

Robot previously described:
"I can see a white humanoid near the wall."

User:
"Approach it"

Correct output:
{"action": "approach", "object": "humanoid"}

Do NOT output:
{"action": "approach", "object": "it"}

If more than one object could reasonably be referred to,
use a chat action to ask the user which object they mean.



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


IMPORTANT:
For explicit commands such as "approach the X", "find the X",
"go to the X", or "look for the X", always return:

{"action":"approach","object":"X"}

Do NOT refuse because the object is not currently visible.
Do NOT answer that the object cannot be seen or found.
The approach controller is responsible for searching by rotating
until the target becomes visible.


Visibility from previous scene descriptions must NOT prevent an
approach/search command from being executed.





"""


# ============================================================
# CHAT HISTORY
# ===============================

history = []


# ============================================================
# LOGGING
# ============================================================

def log_interaction(
    user_text,
    command,
    reply,
    latency,
    navigation_success=None,
):

    file_exists = os.path.isfile(
        LOG_FILE
    )

    with open(
        LOG_FILE,
        "a",
        newline="",
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
                "navigation_success",
            ])

        writer.writerow([
            datetime.now().isoformat(
                timespec="seconds"
            ),
            user_text,
            command.get(
                "action",
                "",
            ),
            command.get(
                "room",
                "",
            ),
            command.get(
                "object",
                "",
            ),
            reply,
            round(
                latency,
                3,
            ),
            navigation_success,
        ])


# ============================================================
# BUILD MULTI-TURN PROMPT
# ============================================================

def build_prompt(
    user_text,
):

    conversation = ""

    for item in history:

        conversation += (
            f"{item['role'].upper()}: "
            f"{item['content']}\n"
        )

    conversation += (
        f"USER: {user_text}\n"
    )

    return conversation


# ============================================================
# GEMINI COMMAND PARSER
# ============================================================

def parse_command(
    user_text,
):

    prompt = build_prompt(
        user_text
    )

    start_time = time.time()

    try:

        response = client.models.generate_content(

            model=GEMINI_MODEL,

            contents=prompt,

            config=types.GenerateContentConfig(

                system_instruction=SYSTEM_PROMPT,

                response_mime_type="application/json",

                temperature=0.1,
            ),
        )

        latency = (
            time.time()
            - start_time
        )

        output = (
            response.text
            .strip()
        )

        command = json.loads(
            output
        )

    except Exception as error:

        latency = (
            time.time()
            - start_time
        )

        print(
            f"[Parser error: {error}]"
        )

        command = {
            "action": "chat",
            "reply": (
                "Sorry, I could not understand "
                "that request. Please try again."
            ),
        }

    history.append({
        "role": "user",
        "content": user_text,
    })

    history.append({
        "role": "assistant",
        "content": json.dumps(
            command
        ),
    })

    return command, latency
# ============================================================
# NAVIGATION
# ============================================================

def navigate_to_room(
    room_number,
):

    room_name = (
        f"room{room_number}"
    )

    print(
        f"Robot: Navigating to "
        f"Room {room_number}..."
    )

    node = GotoRoom(
        room_name
    )

    while (
        rclpy.ok()
        and not node.finished
    ):

        rclpy.spin_once(
            node,
            timeout_sec=0.1,
        )

    success = (
        node.success
    )

    node.destroy_node()

    return success


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

    try:

        while rclpy.ok():

            try:

                user_text = input(
                    "You: "
                ).strip()

            except (
                KeyboardInterrupt,
                EOFError,
            ):

                print(
                    "\nRobot: Goodbye."
                )

                break

            if not user_text:
                continue

            if user_text.lower() in [
                "exit",
                "quit",
            ]:

                print(
                    "Robot: Goodbye."
                )

                break

            # ------------------------------------------
            # Parse natural language
            # ------------------------------------------

            command, latency = parse_command(
                user_text
            )

            action = command.get(
                "action"
            )

            navigation_success = None

            # ------------------------------------------
            # GOTO ROOM
            # ------------------------------------------

            if action == "goto_room":

                room = command.get(
                    "room"
                )

                if (
                    not isinstance(
                        room,
                        int,
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

                    success = navigate_to_room(
                        room
                    )

                    navigation_success = (
                        success
                    )

                    if success:

                        reply = (
                            f"I have arrived "
                            f"in Room {room}."
                        )

                    else:

                        reply = (
                            f"Sorry, I could not "
                            f"reach Room {room}."
                        )

            # ------------------------------------------
            # DESCRIBE CURRENT CAMERA VIEW
            # ------------------------------------------

            elif action == "describe":

                try:

                    print(
                        "Robot: Capturing the current "
                        "camera view..."
                    )

                    vision_response = (
                        ask_current_view()
                    )

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

                try:

                    print(
                        "Robot: Checking the current "
                        "camera view..."
                    )

                    vision_response = (
                        ask_current_view(
                            question
                        )
                    )

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

                    reply = (
                        "Sorry, I could not answer "
                        "that visual question."
                    )

            # ------------------------------------------
            # APPROACH OBJECT
            # ------------------------------------------

            elif action == "approach":

                obj = command.get(
                    "object",
                    "",
                )

                if isinstance(
                    obj,
                    str,
                ):
                    obj = obj.strip()

                if not obj:

                    reply = (
                        "Please specify which object "
                        "you want me to approach."
                    )

                else:

                    print(
                        f"Robot: Approaching "
                        f"the {obj}..."
                    )

                    try:

                        success, reply = approach_object(
                            obj,
                            CURRENT_CAMERA_IMAGE,
                        )

                    except Exception as error:

                        print(
                            f"[Approach error: {error}]"
                        )

                        success = False

                        reply = (
                            f"Sorry, I could not "
                            f"approach the {obj}."
                        )

            # ------------------------------------------
            # STOP
            # ------------------------------------------

            elif action == "stop":

                reply = (
                    "Stopped."
                )

            # ------------------------------------------
            # CHAT / CLARIFICATION
            # ------------------------------------------

            elif action == "chat":

                reply = command.get(
                    "reply",
                    "How can I help?",
                )

            # ------------------------------------------
            # UNKNOWN ACTION
            # ------------------------------------------

            else:

                reply = (
                    "Sorry, I did not understand "
                    "that command."
                )

            # ------------------------------------------
            # ROBOT RESPONSE
            # ------------------------------------------

            print(
                f"Robot: {reply}"
            )

            print(
                f"[Parser latency: "
                f"{latency:.3f} s]"
            )

            # Add actual robot response to history.
            history.append({
                "role": "assistant",
                "content": reply,
            })

            # Save interaction log.
            log_interaction(
                user_text,
                command,
                reply,
                latency,
                navigation_success,
            )

            print("")

    finally:

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
