"""Replay the end-to-end approach turns through the current command parser.

In the end-to-end trials the parser refused some "move to the X" requests
because the room description in the chat history had not named X. This
script rebuilds each trial's history exactly as logged (turn 1: the
navigation request, its parsed command and the robot's reply), sends the
approach request to the *current* parser, and checks it returns an approach
command naming the target. No simulation is needed; results go to
evaluation/parser_approach_replay.csv.

    python evaluation/parser_approach_replay.py
"""

from __future__ import annotations

import csv
import json
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "ros2_ws/src/ee4705_perception"))
sys.path.insert(0, str(PROJECT_ROOT / "evaluation"))

from ee4705_perception import terminal_chat  # noqa: E402
from run_end_to_end_trials import names_target  # noqa: E402

TRIALS = PROJECT_ROOT / "evaluation" / "end_to_end_trials.csv"
RESULTS = PROJECT_ROOT / "evaluation" / "parser_approach_replay.csv"


def main() -> None:
    rows = []
    with TRIALS.open(newline="", encoding="utf-8") as file:
        trials = list(csv.DictReader(file))
    latest = {}
    for trial in trials:
        latest[trial["trial_id"].split("r")[0]] = trial
    for trial in latest.values():
        room = int(trial["room"])
        reply = "I have arrived in Room {}. {}".format(room, trial["description"]).strip()
        terminal_chat.history[:] = [
            {"role": "user", "content": trial["nav_command"]},
            {"role": "assistant", "content": trial["parsed_nav"]},
            {"role": "assistant", "content": reply},
        ]
        command, latency, stage = terminal_chat.parse_command(trial["approach_command"])
        ok = command.get("action") == "approach" and names_target(str(command.get("object", "")), room)
        before = json.loads(trial["parsed_approach"])
        was_ok = before.get("action") == "approach" and names_target(str(before.get("object", "")), room)
        rows.append({"trial_id": trial["trial_id"], "approach_command": trial["approach_command"],
                     "logged_command": trial["parsed_approach"], "logged_correct": was_ok,
                     "replayed_command": json.dumps(command), "replayed_correct": ok,
                     "model": stage.get("model", ""), "latency_s": f"{latency:.2f}"})
        print(f"{trial['trial_id']:6s} logged {'OK ' if was_ok else 'BAD'} -> now {'OK ' if ok else 'BAD'} "
              f"{json.dumps(command)[:90]}")
    with RESULTS.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    print(f"logged: {sum(r['logged_correct'] for r in rows)}/{len(rows)} correct; "
          f"replayed with the current prompt: {sum(r['replayed_correct'] for r in rows)}/{len(rows)}")


if __name__ == "__main__":
    main()
