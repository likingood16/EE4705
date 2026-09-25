# EE4705 Project 1.2 | Task 4 - object approach
# Contributors (from git history): Alexander Likin (1 commit)
"""Task 4.iii: object-approach trials in Gazebo (resumable).

Each trial teleports the robot to a fixed start pose (Gazebo world frame),
runs the same approach controller the chat uses ("move to the <object>"),
then measures the result against Gazebo ground truth. One row per trial is
appended to evaluation/object_approach_trials.csv; trials already logged are
skipped, and failed trials are kept.

Grounding correctness is judged by looking at the evidence images (did the
box in every frame mark the named object?), then recorded with `grade`.

Usage (simulation running, from the project root, after
`source scripts/activate_ubuntu.sh`):

    python evaluation/run_approach_trials.py run            # all pending trials
    python evaluation/run_approach_trials.py run --only T03
    python evaluation/run_approach_trials.py grade T03 yes "box on the hydrant in all 9 frames"
    python evaluation/run_approach_trials.py summary

Runs: run 1 used the first controller and grounding prompt; its failures led
to the fixes in commit history (grounding prompt, turn clearance, camera
staleness during motion, Nav2 around blocked paths). Run 2 repeated the same
12 trials with those fixes; run 3 adds laser-confirmed arrival when the target
drops out of view at close range, and backing off when a turn is blocked.
All runs stay in the CSV (`run` column);
--run selects which one a command acts on (default: the current run).

Success (defined before the trials, see evaluation/README.md): the controller
reports arrival, grounding is correct, the robot's centre ends within 1.0 m
of the object (nearest body link for the human figure), and the final laser
range to the object is at least 0.20 m (no contact).
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "evaluation"))
sys.path.insert(0, str(PROJECT_ROOT / "ros2_ws/src/ee4705_perception"))

from sim_helpers import Pose2D, angle_diff, gazebo_pose, set_gazebo_pose  # noqa: E402

RESULTS = PROJECT_ROOT / "evaluation" / "object_approach_trials.csv"
EVIDENCE = PROJECT_ROOT / "evaluation" / "task4_evidence" / "trials"
WORLD = PROJECT_ROOT / "worlds" / "house_with_objects.world"

CURRENT_RUN = "3"
SUCCESS_DISTANCE_M = 1.0
MIN_SAFE_LASER_RANGE_M = 0.20

# (trial id, spoken target, Gazebo model, start x, y (world), start yaw (deg)).
# Start poses have >= 0.45 m clearance in the SLAM map and a clear line of
# sight to the object; the yaw sets whether it starts in view (camera HFOV
# is 62 deg).
TRIALS = [
    ("T01", "fire hydrant", "fire_hydrant", -3.08, 3.46, 180),   # ahead, 3.5 m
    ("T02", "fire hydrant", "fire_hydrant", -4.41, 2.21, 170),   # 20 deg off, 2.5 m
    ("T03", "fire hydrant", "fire_hydrant", -6.58, 0.46, -90),   # behind, 3.0 m
    ("T04", "car wheel", "car_wheel", 4.34, 0.97, -15),          # ahead, 3.0 m
    ("T05", "car wheel", "car_wheel", 6.46, 3.09, -50),          # 25 deg off, 3.0 m
    ("T06", "car wheel", "car_wheel", 5.12, 2.31, 75),           # 120 deg right, 3.0 m
    ("T07", "cinder block", "cinder_block", 6.44, -1.51, -90),   # ahead, 3.5 m
    ("T08", "cinder block", "cinder_block", 5.79, -2.60, 105),   # behind, 2.5 m
    ("T09", "cinder block", "cinder_block", 5.44, -3.28, -80),   # 20 deg off, 2.0 m
    ("T10", "person", "ragdoll", -6.30, 0.12, -105),             # ahead, 3.5 m
    ("T11", "person", "ragdoll", -6.46, -1.96, -30),             # 90 deg right, 1.5 m
    ("T12", "person", "ragdoll", -5.71, -0.66, -105),            # 15 deg off, 3.0 m
]

FIELDS = [
    "run", "trial_id", "target", "gazebo_model", "start_pose_world", "start_distance_m",
    "start_bearing_deg", "planned_visible", "initially_visible", "grounding_correct",
    "arrived", "success", "final_distance_m", "final_laser_range_m", "time_s",
    "api_calls", "input_tokens", "output_tokens", "search_rotation_deg",
    "failure_reason", "model", "evidence_dir", "notes",
]

CAMERA_HALF_FOV_DEG = 31.1


def ragdoll_link_names() -> list[str]:
    """Names of the human figure's 24 links, from the world file."""

    for model in ET.parse(WORLD).getroot().find("world").findall("model"):
        if model.get("name") == "ragdoll":
            return [link.get("name") for link in model.findall("link")]
    raise RuntimeError("ragdoll not found in the world file")


def object_distance(model: str, robot: Pose2D) -> float:
    """Robot centre to the object's centre (nearest body link for the human figure)."""

    if model == "ragdoll":
        links = [gazebo_pose(f"ragdoll::{name}") for name in ragdoll_link_names()]
        return min(math.dist((robot.x, robot.y), (link.x, link.y)) for link in links)
    target = gazebo_pose(model)
    return math.dist((robot.x, robot.y), (target.x, target.y))


def read_rows() -> list[dict]:
    if not RESULTS.exists() or RESULTS.stat().st_size == 0:
        return []
    with RESULTS.open(newline="", encoding="utf-8") as file:
        rows = list(csv.DictReader(file))
    for row in rows:
        row["run"] = row.get("run") or "1"  # rows from before the run column
    return rows


def write_rows(rows: list[dict]) -> None:
    with RESULTS.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(rows)


def decide_success(row: dict) -> str:
    if row["grounding_correct"] == "N/A":
        return "False"
    if row["grounding_correct"] not in ("True", "False"):
        return ""  # not graded yet
    try:
        distance = float(row["final_distance_m"])
        laser = float(row["final_laser_range_m"]) if row["final_laser_range_m"] else math.inf
    except ValueError:
        return "False"
    return str(row["arrived"] == "True" and row["grounding_correct"] == "True"
               and distance <= SUCCESS_DISTANCE_M and laser >= MIN_SAFE_LASER_RANGE_M)


def run(arguments: argparse.Namespace) -> None:
    import rclpy

    from ee4705_perception.approach_robot import make_vlm_client, run_approach
    from ee4705_perception.goto_room import navigate_to_pose

    rows = read_rows()
    done = {row["trial_id"] for row in rows if row["run"] == arguments.run}
    rclpy.init()
    client = make_vlm_client("qwen")
    try:
        for trial_id, target, model, x, y, yaw_deg in TRIALS:
            if arguments.only and trial_id not in arguments.only:
                continue
            if trial_id in done:
                print(f"{trial_id}: already logged, skipping")
                continue
            start = Pose2D(x, y, math.radians(yaw_deg))
            set_gazebo_pose(start)
            time.sleep(3.0)  # let odometry and the sensors settle
            actual = gazebo_pose("waffle_pi")
            start_distance = object_distance(model, actual)
            obj = gazebo_pose(model)
            bearing = math.degrees(angle_diff(math.atan2(obj.y - actual.y, obj.x - actual.x),
                                              actual.yaw))
            print(f"{trial_id}: {target} from ({actual.x:.2f}, {actual.y:.2f}, "
                  f"{math.degrees(actual.yaw):.0f} deg), {start_distance:.2f} m, "
                  f"bearing {bearing:+.0f} deg", flush=True)
            started = time.monotonic()
            outcome = run_approach(
                target, client, enable_motion=True, exclusive_control=True,
                inf_is_clear=True, evidence_dir=EVIDENCE / f"run{arguments.run}",
                trial_id=f"run{arguments.run}-{trial_id}",
                navigate=navigate_to_pose,
            )
            elapsed = time.monotonic() - started
            time.sleep(1.0)
            final = gazebo_pose("waffle_pi")
            row = {
                "run": arguments.run,
                "trial_id": trial_id,
                "target": target,
                "gazebo_model": model,
                "start_pose_world": f"({actual.x:.2f}, {actual.y:.2f}, {math.degrees(actual.yaw):.0f} deg)",
                "start_distance_m": f"{start_distance:.2f}",
                "start_bearing_deg": f"{bearing:+.0f}",
                "planned_visible": str(abs(bearing) < CAMERA_HALF_FOV_DEG),
                "initially_visible": str(outcome.initially_visible),
                "grounding_correct": "",
                "arrived": str(outcome.visual_candidate),
                "success": "",
                "final_distance_m": f"{object_distance(model, final):.3f}",
                "final_laser_range_m": "" if outcome.final_range_m is None
                else f"{outcome.final_range_m:.3f}",
                "time_s": f"{elapsed:.1f}",
                "api_calls": outcome.api_calls,
                "input_tokens": outcome.input_tokens,
                "output_tokens": outcome.output_tokens,
                "search_rotation_deg": f"{outcome.search_rotation_deg:.0f}",
                "failure_reason": "" if outcome.visual_candidate else outcome.reason,
                "model": getattr(client, "model", ""),
                "evidence_dir": str(Path(outcome.evidence_dir).relative_to(PROJECT_ROOT)),
                "notes": outcome.reply,
            }
            rows.append(row)
            write_rows(rows)  # save after every trial
            print(f"  -> {outcome.reason}, final distance {row['final_distance_m']} m, "
                  f"{outcome.api_calls} calls, {elapsed:.0f} s", flush=True)
    finally:
        if rclpy.ok():
            rclpy.shutdown()


def grade(arguments: argparse.Namespace) -> None:
    rows = read_rows()
    for row in rows:
        if row["trial_id"] == arguments.trial_id and row["run"] == arguments.run:
            # "na": the target never came into view, so grounding was not tested.
            row["grounding_correct"] = {"yes": "True", "no": "False", "na": "N/A"}[arguments.correct]
            if arguments.note:
                row["notes"] = f"{row['notes']} Grounding: {arguments.note}".strip()
            row["success"] = decide_success(row)
            write_rows(rows)
            print(f"run {row['run']} {row['trial_id']}: grounding {row['grounding_correct']}, "
                  f"success {row['success']}")
            return
    raise SystemExit(f"No trial {arguments.trial_id} (run {arguments.run}) in {RESULTS}")


def summary(arguments: argparse.Namespace) -> None:
    rows = [row for row in read_rows() if row["run"] == arguments.run]
    if not rows:
        raise SystemExit(f"No trials logged for run {arguments.run} yet.")
    print(f"run {arguments.run}")
    graded = [r for r in rows if r["grounding_correct"] in ("True", "False")]
    successes = [r for r in rows if r["success"] == "True"]
    arrived = [r for r in rows if r["arrived"] == "True"]
    hidden = [r for r in rows if r["planned_visible"] == "False"]
    print(f"trials: {len(rows)} ({len(hidden)} starting with the object out of view)")
    print(f"objects: {', '.join(sorted({r['target'] for r in rows}))}")
    print(f"grounding correct: {sum(r['grounding_correct'] == 'True' for r in graded)}/{len(graded)}")
    print(f"approach success: {len(successes)}/{len(rows)}")
    print(f"  out-of-view starts: {sum(r['success'] == 'True' for r in hidden)}/{len(hidden)}")
    distances = [float(r["final_distance_m"]) for r in arrived]
    if distances:
        print(f"final distance when arrived: mean {sum(distances) / len(distances):.2f} m, "
              f"min {min(distances):.2f}, max {max(distances):.2f}")
    times = [float(r["time_s"]) for r in rows]
    calls = [int(r["api_calls"]) for r in rows]
    print(f"time: mean {sum(times) / len(times):.1f} s; VLM calls: mean "
          f"{sum(calls) / len(calls):.1f}, total {sum(calls)}")
    tokens_in = sum(int(r["input_tokens"] or 0) for r in rows)
    tokens_out = sum(int(r["output_tokens"] or 0) for r in rows)
    print(f"tokens: {tokens_in} in, {tokens_out} out")
    for r in rows:
        if r["success"] != "True":
            print(f"  {r['trial_id']} failed: {r['failure_reason'] or 'see notes'} "
                  f"(arrived={r['arrived']}, grounding={r['grounding_correct']}, "
                  f"distance={r['final_distance_m']})")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    steps = parser.add_subparsers(dest="step", required=True)
    run_parser = steps.add_parser("run", help="run pending trials (needs the simulation)")
    run_parser.add_argument("--only", nargs="+", help="trial ids to run")
    grade_parser = steps.add_parser("grade", help="record grounding correctness for a trial")
    grade_parser.add_argument("trial_id")
    grade_parser.add_argument("correct", choices=["yes", "no", "na"])
    grade_parser.add_argument("note", nargs="?", default="")
    summary_parser = steps.add_parser("summary", help="print the results")
    for sub in (run_parser, grade_parser, summary_parser):
        sub.add_argument("--run", default=CURRENT_RUN)
    arguments = parser.parse_args()
    {"run": run, "grade": grade, "summary": summary}[arguments.step](arguments)


if __name__ == "__main__":
    main()
