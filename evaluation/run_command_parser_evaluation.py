
import csv
import json
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
PACKAGE_SOURCE = PROJECT_ROOT / "ros2_ws/src/ee4705_perception"
sys.path.insert(0, str(PACKAGE_SOURCE))

from ee4705_perception.terminal_chat import parse_command

TRIALS = [
    ("direct", "Go to Room 1.", "goto_room", "1"),
    ("paraphrase", "Could you check room three please?", "goto_room", "3"),
    ("paraphrase", "Head over to the fourth room.", "goto_room", "4"),
    ("paraphrase", "Make your way to room six.", "goto_room", "6"),
    ("paraphrase", "Please visit the second room.", "goto_room", "2"),
    ("paraphrase", "Navigate to the fifth room when ready.", "goto_room", "5"),
    ("follow_up", "Go to Room 3.", "goto_room", "3"),
    ("follow_up", "Please go there again.", "goto_room", "3"),
    ("invalid", "Go to Room 99.", "chat", ""),
    ("invalid", "Take me to Room 0.", "chat", ""),
    ("invalid", "Navigate to Room 7.", "chat", ""),
    ("out_of_scope", "Do a backflip.", "chat", ""),
    ("out_of_scope", "Fly upstairs.", "chat", ""),
    ("out_of_scope", "What will tomorrow's weather be?", "chat", ""),
    ("capability", "What can you see?", "describe", ""),
    ("capability", "Describe the current room.", "describe", ""),
    ("capability", "Move towards the cup.", "approach", "cup"),
    ("capability", "Approach the ball.", "approach", "ball"),
    ("capability", "Stop moving immediately.", "stop", ""),
    ("chat", "Hello, how are you?", "chat", ""),
]
def predicted_argument(command):
    """Extract the room number or object name from a parsed command."""
    action = command.get("action")

    if action == "goto_room":
        return str(command.get("room", ""))

    if action == "approach":
        return str(command.get("object", "")).strip().lower()

    return ""


def evaluate_match(command, expected_action, expected_argument):
    """Return True when Gemini's command matches the expected command."""
    if not isinstance(command, dict):
        return False

    if command.get("action") != expected_action:
        return False

    if expected_argument:
        return predicted_argument(command) == expected_argument.lower()

    return True


def main():
    """Run all trials, grade them and save the results."""
    output_path = (
        PROJECT_ROOT / "evaluation/command_parser_trials.csv"
    )
    rows = []

    for number, trial in enumerate(TRIALS, start=1):
        category, utterance, expected_action, expected_argument = trial

        print(f"\n[{number:02d}/{len(TRIALS)}] {utterance}")

        command = {}
        latency = 0.0
        error = ""

        try:
            command, latency = parse_command(utterance)

            passed = evaluate_match(
                command,
                expected_action,
                expected_argument,
            )

        except Exception as exc:
            passed = False
            error = f"{type(exc).__name__}: {exc}"

        if isinstance(command, dict):
            predicted_action = command.get("action", "")
            predicted_arg = predicted_argument(command)
            raw_command = json.dumps(
                command,
                ensure_ascii=False,
            )
        else:
            predicted_action = ""
            predicted_arg = ""
            raw_command = str(command)

        rows.append({
            "trial": number,
            "category": category,
            "utterance": utterance,
            "expected_action": expected_action,
            "expected_argument": expected_argument,
            "predicted_action": predicted_action,
            "predicted_argument": predicted_arg,
            "correct": passed,
            "latency_seconds": f"{latency:.3f}",
            "raw_command": raw_command,
            "error": error,
        })

        print(
            f"  Expected: {expected_action}:"
            f"{expected_argument or '-'}"
        )
        print(
            f"  Predicted: {predicted_action}:"
            f"{predicted_arg or '-'}"
        )
        print(f"  Correct: {passed}")
        print(f"  Latency: {latency:.3f}s")

        if error:
            print(f"  Error: {error}")

        # Pause to reduce the chance of Gemini free-tier rate limiting.
        if number < len(TRIALS):
            time.sleep(4)


    fieldnames = [
        "trial",
        "category",
        "utterance",
        "expected_action",
        "expected_argument",
        "predicted_action",
        "predicted_argument",
        "correct",
        "latency_seconds",
        "raw_command",
        "error",
    ]

    with output_path.open(
        "w",
        newline="",
        encoding="utf-8",
    ) as csv_file:
        writer = csv.DictWriter(
            csv_file,
            fieldnames=fieldnames,
        )
        writer.writeheader()
        writer.writerows(rows)


    correct_count = sum(
        row["correct"] for row in rows
    )

    measured_latencies = [
        float(row["latency_seconds"])
        for row in rows
        if float(row["latency_seconds"]) > 0
    ]

    if measured_latencies:
        average_latency = (
            sum(measured_latencies)
            / len(measured_latencies)
        )
    else:
        average_latency = 0.0

    print("\n=== CATEGORY RESULTS ===")

    categories = sorted({
        row["category"] for row in rows
    })

    for category in categories:
        category_rows = [
            row for row in rows
            if row["category"] == category
        ]
        category_correct = sum(
            row["correct"] for row in category_rows
        )

        print(
            f"{category}: "
            f"{category_correct}/{len(category_rows)}"
        )


    failed_rows = [
        row for row in rows
        if not row["correct"]
    ]

    print("\n=== SUMMARY ===")
    print(f"Correct: {correct_count}/{len(rows)}")
    print(
        f"Accuracy: "
        f"{100 * correct_count / len(rows):.1f}%"
    )
    print(
        f"Average latency: "
        f"{average_latency:.3f}s"
    )
    print(f"Failed trials: {len(failed_rows)}")
    print(f"Results saved to: {output_path}")

    if failed_rows:
        print("\n=== FAILURES TO ANALYSE ===")

        for row in failed_rows:
            print(
                f"Trial {row['trial']}: "
                f"{row['utterance']}"
            )
            print(
                f"  Expected: "
                f"{row['expected_action']}:"
                f"{row['expected_argument'] or '-'}"
            )
            print(
                f"  Predicted: "
                f"{row['predicted_action']}:"
                f"{row['predicted_argument'] or '-'}"
            )


if __name__ == "__main__":
    main()