# EE4705 Project 1.2 | Task 5 - end-to-end trials
# Contributors (from git history): Alexander Likin (1 commit)
"""Task 5.ii: 20 randomized end-to-end trials through the chat pipeline.

Each trial is a two-turn conversation run through terminal_chat.handle_turn()
(exactly what the interactive chat does): turn 1 asks the robot to go to a
room (it navigates, then describes what it sees), turn 2 asks it to move to
that room's object. Room, object wording and command phrasing come from a
seeded random generator, so the trial list is reproducible. Results are
appended to evaluation/end_to_end_trials.csv after every trial (resumable);
failed trials are kept. Success definitions: evaluation/README.md.

Usage (simulation running, from the project root, after
`source scripts/activate_ubuntu.sh`):

    python evaluation/run_end_to_end_trials.py plan      # print the 20 trials
    python evaluation/run_end_to_end_trials.py run
    python evaluation/run_end_to_end_trials.py grade E07 description no "says a chair"
    python evaluation/run_end_to_end_trials.py grade E07 grounding yes
    python evaluation/run_end_to_end_trials.py summary
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import random
import shutil
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "evaluation"))

from sim_helpers import gazebo_pose, load_rooms, world_to_map  # noqa: E402
from run_approach_trials import object_distance  # noqa: E402

RESULTS = PROJECT_ROOT / "evaluation" / "end_to_end_trials.csv"
EVIDENCE = PROJECT_ROOT / "evaluation" / "e2e_evidence"
SEED = 4705
TRIAL_COUNT = 20
NAV_TOLERANCE_M = 0.5
APPROACH_DISTANCE_M = 1.0
MIN_SAFE_LASER_RANGE_M = 0.20

# Rooms whose waypoint view contains a floor object: (ways of naming it,
# keywords that count as naming it, Gazebo model).
ROOM_OBJECTS = {
    2: (["fire hydrant", "hydrant", "red fire hydrant"], ["hydrant"], "fire_hydrant"),
    4: (["car wheel", "wheel", "tyre"], ["wheel", "tyre", "tire"], "car_wheel"),
    5: (["person", "human figure", "standing person"],
        ["person", "human", "figure", "man", "woman", "mannequin", "doll", "body"],
        "ragdoll"),
    6: (["cinder block", "concrete block", "grey block"], ["cinder", "block", "brick"],
        "cinder_block"),
}
WORDS = {1: "one", 2: "two", 3: "three", 4: "four", 5: "five", 6: "six"}
ORDINALS = {1: "first", 2: "second", 3: "third", 4: "fourth", 5: "fifth", 6: "sixth"}
NAV_PHRASES = [
    "Go to Room {n} and have a look.",
    "Could you check room {word} please?",
    "Head over to the {ordinal} room and tell me what you see.",
    "Please navigate to room number {n}.",
    "Take me to room {word}. What is there?",
    "I'd like you to visit Room {n}.",
]
APPROACH_PHRASES = [
    "Move to the {obj}.",
    "Go over to the {obj}.",
    "Drive up to the {obj}, please.",
    "Can you get close to the {obj}?",
    "Approach the {obj}.",
    "Please go and stand next to the {obj}.",
]

FIELDS = [
    "trial_id", "room", "target_object", "nav_command", "approach_command",
    "parsed_nav", "parsed_approach", "parsing_success", "navigation_success",
    "nav_final_error_m", "description", "description_success", "approach_arrived",
    "grounding_correct", "approach_final_distance_m", "approach_success",
    "end_to_end_success", "parsing_latency_s", "navigation_latency_s",
    "description_latency_s", "approach_latency_s", "parse_calls", "vision_calls",
    "approach_calls", "input_tokens", "output_tokens", "parser_models", "total_cost_usd",
    "failure_reason", "evidence", "notes",
]


def make_plan() -> list[dict]:
    rng = random.Random(SEED)
    plan = []
    for index in range(TRIAL_COUNT):
        room = rng.choice(sorted(ROOM_OBJECTS))
        names, _, _ = ROOM_OBJECTS[room]
        plan.append({
            "trial_id": f"E{index + 1:02d}",
            "room": room,
            "object": rng.choice(names),
            "nav_command": rng.choice(NAV_PHRASES).format(
                n=room, word=WORDS[room], ordinal=ORDINALS[room]),
            "approach_phrase": rng.choice(APPROACH_PHRASES),
        })
    for trial in plan:
        trial["approach_command"] = trial.pop("approach_phrase").format(obj=trial["object"])
    return plan


def read_rows() -> list[dict]:
    if not RESULTS.exists() or RESULTS.stat().st_size == 0:
        return []
    with RESULTS.open(newline="", encoding="utf-8") as file:
        return [row for row in csv.DictReader(file) if row.get("trial_id")]


def write_rows(rows: list[dict]) -> None:
    with RESULTS.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(rows)


def names_target(text: str, room: int) -> bool:
    text = text.lower()
    return any(word in text for word in ROOM_OBJECTS[room][1])


def stage_value(turn: dict, stage: str, key: str, default=0):
    record = turn.get(stage) or {}
    value = record.get(key)
    return default if value is None else value


def finalize(row: dict) -> None:
    """Recompute the approach and end-to-end verdicts after grading."""

    approach_ok = ""
    if row["grounding_correct"] in ("True", "False"):
        try:
            distance = float(row["approach_final_distance_m"])
        except ValueError:
            distance = math.inf
        approach_ok = str(row["approach_arrived"] == "True" and row["grounding_correct"] == "True"
                          and distance <= APPROACH_DISTANCE_M
                          and "contact" not in row["notes"])
    row["approach_success"] = approach_ok
    stages = [row["parsing_success"], row["navigation_success"],
              row["description_success"], row["approach_success"]]
    row["end_to_end_success"] = "" if "" in stages else str(all(s == "True" for s in stages))


def run(arguments: argparse.Namespace) -> None:
    import rclpy

    from ee4705_perception import terminal_chat

    rows = read_rows()
    done = {row["trial_id"] for row in rows}
    rooms = load_rooms()
    EVIDENCE.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    try:
        for trial in make_plan():
            if arguments.only and trial["trial_id"] not in arguments.only:
                continue
            # A retry repeats a planned trial under its own id (e.g. E12r);
            # the original row is kept.
            trial = dict(trial, trial_id=trial["trial_id"] + arguments.retry_suffix)
            if trial["trial_id"] in done:
                continue
            room = trial["room"]
            _, _, model = ROOM_OBJECTS[room]
            print(f"{trial['trial_id']}: room {room}, {trial['object']!r}", flush=True)
            # Each trial is its own conversation.
            terminal_chat.history.clear()

            print(f"  You: {trial['nav_command']}", flush=True)
            nav_turn = terminal_chat.handle_turn(trial["nav_command"])
            print(f"  Robot: {nav_turn['reply'][:160]}", flush=True)
            image = EVIDENCE / f"{trial['trial_id']}_description.jpg"
            if nav_turn.get("vision") and "model" in nav_turn["vision"]:
                shutil.copy(terminal_chat.CURRENT_CAMERA_IMAGE, image)
            truth = world_to_map(gazebo_pose("waffle_pi"))
            goal = rooms[f"room{room}"]
            nav_error = math.dist((truth.x, truth.y), (goal.x, goal.y))

            print(f"  You: {trial['approach_command']}", flush=True)
            approach_turn = terminal_chat.handle_turn(trial["approach_command"])
            print(f"  Robot: {approach_turn['reply']}", flush=True)
            time.sleep(1.0)
            robot = gazebo_pose("waffle_pi")
            final_distance = object_distance(model, robot)

            command_1, command_2 = nav_turn["command"], approach_turn["command"]
            parse_ok = (command_1.get("action") == "goto_room" and command_1.get("room") == room
                        and command_2.get("action") == "approach"
                        and names_target(str(command_2.get("object", "")), room))
            nav_ok = bool(stage_value(nav_turn, "navigation", "success", False)) \
                and nav_error <= NAV_TOLERANCE_M
            description = stage_value(nav_turn, "vision", "text", "")
            approach = approach_turn.get("approach") or {}
            laser = approach.get("final_range_m")
            notes = []
            if laser is not None and laser < MIN_SAFE_LASER_RANGE_M:
                notes.append("contact: final laser range below 0.20 m")
            turns = (nav_turn, approach_turn)
            row = {
                "trial_id": trial["trial_id"],
                "room": room,
                "target_object": trial["object"],
                "nav_command": trial["nav_command"],
                "approach_command": trial["approach_command"],
                "parsed_nav": json.dumps(command_1),
                "parsed_approach": json.dumps(command_2),
                "parsing_success": str(parse_ok),
                "navigation_success": str(nav_ok),
                "nav_final_error_m": f"{nav_error:.3f}",
                "description": description,
                # Keyword check here; confirmed or overridden with `grade`.
                "description_success": str(bool(description) and names_target(description, room)),
                "approach_arrived": str(bool(approach.get("success"))),
                "grounding_correct": "" if approach.get("evidence_dir") else "False",
                "approach_final_distance_m": f"{final_distance:.3f}",
                "approach_success": "",
                "end_to_end_success": "",
                "parsing_latency_s": f"{sum(t['parse']['latency_s'] for t in turns):.2f}",
                "navigation_latency_s": f"{stage_value(nav_turn, 'navigation', 'latency_s'):.2f}",
                "description_latency_s": f"{stage_value(nav_turn, 'vision', 'latency_s'):.2f}",
                "approach_latency_s": f"{approach.get('latency_s', 0):.2f}",
                "parse_calls": sum(t["parse"]["calls"] for t in turns),
                "vision_calls": stage_value(nav_turn, "vision", "calls"),
                "approach_calls": approach.get("calls", 0),
                "input_tokens": sum(t["parse"]["input_tokens"] for t in turns)
                + stage_value(nav_turn, "vision", "input_tokens") + approach.get("input_tokens", 0),
                "output_tokens": sum(t["parse"]["output_tokens"] for t in turns)
                + stage_value(nav_turn, "vision", "output_tokens") + approach.get("output_tokens", 0),
                "parser_models": " / ".join(t["parse"].get("model", "failed") for t in turns),
                "total_cost_usd": "0.00 (free tier)",
                "failure_reason": approach.get("reason", "") if not approach.get("success") else "",
                "evidence": f"{image.relative_to(PROJECT_ROOT) if image.exists() else ''} | "
                            f"{approach.get('evidence_dir', '')}",
                "notes": "; ".join(notes),
            }
            finalize(row)
            rows.append(row)
            write_rows(rows)
            print(f"  -> parse {parse_ok}, nav {nav_ok} ({nav_error:.2f} m), description "
                  f"{row['description_success']}, arrived {row['approach_arrived']} "
                  f"({final_distance:.2f} m)", flush=True)
    finally:
        if rclpy.ok():
            rclpy.shutdown()


def grade(arguments: argparse.Namespace) -> None:
    rows = read_rows()
    for row in rows:
        if row["trial_id"] != arguments.trial_id:
            continue
        verdict = str(arguments.correct == "yes")
        if arguments.stage == "description":
            row["description_success"] = verdict
        else:
            row["grounding_correct"] = verdict
        if arguments.note:
            row["notes"] = "; ".join(filter(None, [row["notes"], f"{arguments.stage}: {arguments.note}"]))
        finalize(row)
        write_rows(rows)
        print(f"{row['trial_id']}: {arguments.stage} {verdict}; approach {row['approach_success']}, "
              f"end-to-end {row['end_to_end_success']}")
        return
    raise SystemExit(f"No trial {arguments.trial_id} in {RESULTS}")


def summary(arguments: argparse.Namespace) -> None:
    rows = read_rows()
    if not rows:
        raise SystemExit("No trials logged yet.")
    if arguments.latest:
        # One row per planned trial: its last attempt (reruns such as E12r2
        # replace trials lost to the network outage; all rows stay in the file).
        latest = {}
        for row in rows:
            latest[row["trial_id"].split("r")[0]] = row
        rows = [latest[key] for key in sorted(latest)]
        print("latest attempt of each planned trial: "
              + ", ".join(row["trial_id"] for row in rows))
    n = len(rows)

    def rate(key):
        return f"{sum(r[key] == 'True' for r in rows)}/{n}"

    def mean(key, subset=None):
        # 0 means the stage did not run in that trial.
        values = [float(r[key]) for r in (subset or rows) if r[key] not in ("", None)
                  and float(r[key]) > 0]
        return sum(values) / len(values) if values else float("nan")

    print(f"trials: {n}")
    for key in ("parsing_success", "navigation_success", "description_success",
                "approach_success", "end_to_end_success"):
        print(f"  {key:22s} {rate(key)}")
    print("mean latency per stage (s):")
    print(f"  parsing (2 turns) {mean('parsing_latency_s'):.1f}, navigation "
          f"{mean('navigation_latency_s', [r for r in rows if r['navigation_success'] == 'True']):.1f} "
          f"(successful goals), description {mean('description_latency_s'):.1f}, "
          f"approach {mean('approach_latency_s'):.1f}")
    calls = {key: sum(int(r[key] or 0) for r in rows)
             for key in ("parse_calls", "vision_calls", "approach_calls")}
    tokens_in = sum(int(r["input_tokens"] or 0) for r in rows)
    tokens_out = sum(int(r["output_tokens"] or 0) for r in rows)
    print(f"API calls: {calls}; tokens {tokens_in} in, {tokens_out} out; cost US$0 (free tiers)")
    for r in rows:
        if r["end_to_end_success"] != "True":
            failed = [k for k in ("parsing_success", "navigation_success", "description_success",
                                  "approach_success") if r[k] != "True"]
            print(f"  {r['trial_id']} (room {r['room']}): failed {', '.join(failed)} "
                  f"{r['failure_reason']} {r['notes']}")


def plan(arguments: argparse.Namespace) -> None:
    for trial in make_plan():
        print(f"{trial['trial_id']}: room {trial['room']}: {trial['nav_command']!r} -> "
              f"{trial['approach_command']!r}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    steps = parser.add_subparsers(dest="step", required=True)
    steps.add_parser("plan", help="print the seeded trial list")
    run_parser = steps.add_parser("run", help="run pending trials (needs the simulation)")
    run_parser.add_argument("--only", nargs="+")
    run_parser.add_argument("--retry-suffix", default="",
                            help="rerun --only trials as new rows, e.g. r -> E12r")
    grade_parser = steps.add_parser("grade", help="record a manual check")
    grade_parser.add_argument("trial_id")
    grade_parser.add_argument("stage", choices=["description", "grounding"])
    grade_parser.add_argument("correct", choices=["yes", "no"])
    grade_parser.add_argument("note", nargs="?", default="")
    summary_parser = steps.add_parser("summary", help="print the results")
    summary_parser.add_argument("--latest", action="store_true",
                                help="count only the last attempt of each planned trial")
    arguments = parser.parse_args()
    {"plan": plan, "run": run, "grade": grade, "summary": summary}[arguments.step](arguments)


if __name__ == "__main__":
    main()
