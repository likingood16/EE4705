"""Task 3 VLM comparison: capture 10 scenes and compare Gemini with Qwen.

Three steps, each resumable:

1. capture  - needs the simulation running. Drives to each room with Nav2,
              saves a camera frame per view to evaluation/scenes/comparison/, and writes
              evaluation/scenes/comparison/scenes.csv with the objects that should be
              visible (estimated from the world file and map; check them).
2. evaluate - no ROS needed. Sends every scene to every model with the same
              description prompt and appends one row per trial to
              evaluation/vlm_scene_trials.csv. Trials already logged are skipped.
3. score    - fills correct/missed/hallucinated objects by keyword matching
              and prints a per-model summary for the report.

Usage (from the project root, after `source scripts/activate_ubuntu.sh`):

    python evaluation/run_vlm_comparison.py capture
    python evaluation/run_vlm_comparison.py evaluate
    python evaluation/run_vlm_comparison.py score
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
PACKAGE_SOURCE = PROJECT_ROOT / "ros2_ws/src/ee4705_perception"
sys.path.insert(0, str(PACKAGE_SOURCE))

SCENES_DIR = PROJECT_ROOT / "evaluation" / "scenes" / "comparison"
MANIFEST = SCENES_DIR / "scenes.csv"
RESULTS = PROJECT_ROOT / "evaluation" / "vlm_scene_trials.csv"
WAYPOINTS = PROJECT_ROOT / "config" / "room_waypoints.yaml"
WORLD = PROJECT_ROOT / "worlds" / "house_with_objects.world"
MAP_IMAGE = PROJECT_ROOT / "maps" / "house_map_final.pgm"

# Map frame = Gazebo world frame shifted by this offset (see simulation.launch.py).
WORLD_TO_MAP = (2.0, 0.45)
MAP_RESOLUTION = 0.05
MAP_ORIGIN = (-5.74, -5.08)
CAMERA_HFOV = 1.085  # TurtleBot3 Waffle Pi camera horizontal field of view (rad)
MAX_VISIBLE_RANGE = 6.0

SCENE_COUNT = 10
# Each room is photographed facing its waypoint heading, then turned around.
VIEWS = [(f"room{n}", offset) for offset in (0.0, math.pi) for n in range(1, 7)]

# Gazebo model name -> object label used for scoring.
WORLD_OBJECTS = {
    "fire_hydrant": "fire hydrant",
    "cinder_block": "cinder block",
    "car_wheel": "car wheel",
    "ragdoll": "human figure",
    "bookshelf": "bookshelf",
    "cabinet": "cabinet",
    "cafe_table": "table",
    "first_2015_trash_can": "trash can",
    "table_marble": "table",
    "table": "table",
}

# Words that count as mentioning each object label.
SYNONYMS = {
    "fire hydrant": ["hydrant"],
    "cinder block": ["cinder", "concrete block", "breeze block", "brick block"],
    "car wheel": ["wheel", "tyre", "tire"],
    "human figure": ["person", "human", "figure", "mannequin", "ragdoll", "doll", "legs"],
    "bookshelf": ["bookshelf", "bookcase", "shelf", "shelves"],
    "cabinet": ["cabinet", "cupboard", "drawer"],
    "trash can": ["trash", "bin", "garbage", "waste"],
    "table": ["table", "desk"],
}

RESULT_FIELDS = [
    "trial_id",
    "scene_id",
    "image_path",
    "model",
    "question",
    "expected_objects",
    "reported_objects",
    "correct_objects",
    "missed_objects",
    "hallucinated_objects",
    "latency_s",
    "cost_usd",
    "notes",
]


# ---------------------------------------------------------------------------
# Expected-object estimation (no ROS)
# ---------------------------------------------------------------------------


def load_world_objects() -> list[tuple[str, float, float]]:
    """Return (label, map_x, map_y) for every scoreable object in the world."""

    state = ET.parse(WORLD).getroot().find("world/state")
    objects = []
    for model in state.iter("model"):
        base_name = model.get("name", "").split("::")[-1]
        label = WORLD_OBJECTS.get(base_name.rstrip("_0123456789"))
        label = WORLD_OBJECTS.get(base_name, label)
        pose = model.find("pose")
        if label is None or pose is None:
            continue
        x, y = (float(value) for value in pose.text.split()[:2])
        objects.append((label, x + WORLD_TO_MAP[0], y + WORLD_TO_MAP[1]))
    return objects


def load_map() -> tuple[int, int, bytes]:
    """Read the binary PGM map as (width, height, pixels)."""

    data = MAP_IMAGE.read_bytes()
    fields = []
    index = 0
    while len(fields) < 4:
        while data[index:index + 1].isspace():
            index += 1
        if data[index:index + 1] == b"#":
            index = data.index(b"\n", index)
            continue
        start = index
        while not data[index:index + 1].isspace():
            index += 1
        fields.append(data[start:index])
    width, height = int(fields[1]), int(fields[2])
    return width, height, data[index + 1:]


def line_of_sight(grid, start, end, stop_short: float = 0.4) -> bool:
    """True when no wall pixel lies between start and (near) end."""

    width, height, pixels = grid
    distance = math.dist(start, end)
    steps = int((distance - stop_short) / (MAP_RESOLUTION / 2))
    for step in range(1, max(steps, 0)):
        t = step * (MAP_RESOLUTION / 2) / distance
        x = start[0] + (end[0] - start[0]) * t
        y = start[1] + (end[1] - start[1]) * t
        column = int((x - MAP_ORIGIN[0]) / MAP_RESOLUTION)
        row = height - 1 - int((y - MAP_ORIGIN[1]) / MAP_RESOLUTION)
        if 0 <= column < width and 0 <= row < height:
            if pixels[row * width + column] < 50:
                return False
    return True


def visible_objects(x: float, y: float, yaw: float, objects, grid) -> list[str]:
    """Estimate which objects the camera sees from a map-frame pose."""

    seen = []
    for label, ox, oy in objects:
        distance = math.dist((x, y), (ox, oy))
        bearing = math.atan2(oy - y, ox - x) - yaw
        bearing = math.atan2(math.sin(bearing), math.cos(bearing))
        if distance > MAX_VISIBLE_RANGE or abs(bearing) > CAMERA_HFOV / 2:
            continue
        if line_of_sight(grid, (x, y), (ox, oy)) and label not in seen:
            seen.append(label)
    return seen


# ---------------------------------------------------------------------------
# Step 1: capture (needs ROS and the running simulation)
# ---------------------------------------------------------------------------


def navigate(node, client, x: float, y: float, yaw: float, timeout_s: float) -> bool:
    """Send one NavigateToPose goal and wait for success."""

    import rclpy
    from nav2_msgs.action import NavigateToPose

    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = "map"
    goal.pose.header.stamp = node.get_clock().now().to_msg()
    goal.pose.pose.position.x = x
    goal.pose.pose.position.y = y
    goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
    goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

    sent = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, sent, timeout_sec=10.0)
    handle = sent.result()
    if handle is None or not handle.accepted:
        return False
    result = handle.get_result_async()
    rclpy.spin_until_future_complete(node, result, timeout_sec=timeout_s)
    if not result.done():
        handle.cancel_goal_async()
        return False
    return result.result().status == 4  # GoalStatus.STATUS_SUCCEEDED


def capture(arguments: argparse.Namespace) -> None:
    import rclpy
    import yaml
    from nav2_msgs.action import NavigateToPose
    from rclpy.action import ActionClient

    from ee4705_perception.camera_snapshot import capture_one_frame

    rooms = yaml.safe_load(WAYPOINTS.read_text())
    objects = load_world_objects()
    grid = load_map()
    SCENES_DIR.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = rclpy.create_node("vlm_scene_capture")
    client = ActionClient(node, NavigateToPose, "navigate_to_pose")
    if not client.wait_for_server(timeout_sec=30.0):
        raise SystemExit("Nav2 is not running. Start scripts/start_simulation.sh first.")

    rows = []
    try:
        for room, offset in VIEWS:
            if len(rows) == SCENE_COUNT:
                break
            scene_id = f"scene-{len(rows) + 1:03d}"
            image = SCENES_DIR / f"{scene_id}.jpg"
            pose = rooms[room]
            yaw = math.atan2(math.sin(pose["yaw"] + offset), math.cos(pose["yaw"] + offset))
            print(f"{scene_id}: going to {room} facing {math.degrees(yaw):.0f} deg...")
            if not navigate(node, client, pose["x"], pose["y"], yaw, arguments.timeout):
                print(f"  could not reach {room}; skipping this view")
                continue
            time.sleep(1.5)  # let the camera settle after stopping
            capture_one_frame(image)
            expected = visible_objects(pose["x"], pose["y"], yaw, objects, grid)
            print(f"  saved {image.name}; expected objects: {', '.join(expected) or 'none'}")
            rows.append({
                "scene_id": scene_id,
                "image_path": str(image.relative_to(PROJECT_ROOT)),
                "room": room,
                "heading_deg": f"{math.degrees(yaw):.0f}",
                "expected_objects": "; ".join(expected),
            })
    finally:
        node.destroy_node()
        rclpy.shutdown()

    with MANIFEST.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0]) if rows else ["scene_id"])
        writer.writeheader()
        writer.writerows(rows)
    print(f"\nCaptured {len(rows)} scenes. Check expected_objects in {MANIFEST.relative_to(PROJECT_ROOT)}")
    print("against the images before running `evaluate`; the list is an estimate.")


# ---------------------------------------------------------------------------
# Step 2: evaluate (no ROS)
# ---------------------------------------------------------------------------


def make_client(provider: str):
    from ee4705_perception.vlm_client import GeminiVLMClient, QwenVLMClient

    if provider == "gemini":
        return GeminiVLMClient()
    if provider == "qwen":
        return QwenVLMClient()
    raise ValueError(f"Unknown provider: {provider}")


def read_rows(path: Path) -> list[dict]:
    if not path.exists() or path.stat().st_size == 0:
        return []
    with path.open(newline="", encoding="utf-8") as file:
        return list(csv.DictReader(file))


def write_rows(path: Path, rows: list[dict]) -> None:
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=RESULT_FIELDS)
        writer.writeheader()
        writer.writerows(rows)


def evaluate(arguments: argparse.Namespace) -> None:
    from ee4705_perception.scene_describer import DESCRIPTION_PROMPT, SceneDescriber

    scenes = read_rows(MANIFEST)
    if not scenes:
        raise SystemExit(f"No scenes in {MANIFEST}. Run `capture` first.")

    # Failed API calls stay in the file as their own rows (the error rate is
    # part of the comparison); a scene is only re-asked until it succeeds.
    results = read_rows(RESULTS)
    done = {row["trial_id"] for row in results if not row["notes"].startswith("ERROR")}

    for provider in arguments.providers:
        describer = SceneDescriber(make_client(provider))
        for scene in scenes:
            trial_id = f"{scene['scene_id']}-{provider}"
            if trial_id in done:
                print(f"{trial_id}: already logged, skipping")
                continue
            print(f"{trial_id}: asking {provider}...", flush=True)
            row = {
                "trial_id": trial_id,
                "scene_id": scene["scene_id"],
                "image_path": scene["image_path"],
                "model": provider,
                "question": "scene description",
                "expected_objects": scene["expected_objects"],
                "notes": "",
            }
            try:
                response = describer.describe(PROJECT_ROOT / scene["image_path"])
            except Exception as error:  # keep failed trials as evidence
                attempts = sum(r["trial_id"].startswith(f"{trial_id}-error") for r in results)
                row["trial_id"] = f"{trial_id}-error-{attempts + 1}"
                row["notes"] = f"ERROR: {error}"[:500]
                print(f"  failed: {str(error)[:200]}")
            else:
                row.update(
                    model=response.model,
                    reported_objects=response.text,
                    latency_s=f"{response.latency_s:.3f}",
                    cost_usd="" if response.cost_usd is None else response.cost_usd,
                    notes=(f"tokens in={response.input_tokens} out={response.output_tokens} "
                           f"retries={response.retries}"),
                )
                print(f"  {response.latency_s:.2f} s")
            results.append(row)
            write_rows(RESULTS, results)  # save after every trial

    print(f"\nPrompt used for every trial:\n{DESCRIPTION_PROMPT}")
    print(f"\nResults: {RESULTS}. Next: run `score`.")


# ---------------------------------------------------------------------------
# Step 3: score
# ---------------------------------------------------------------------------


def mentioned(label: str, text: str) -> bool:
    text = text.lower()
    return any(word in text for word in [label, *SYNONYMS.get(label, [])])


def score(arguments: argparse.Namespace) -> None:
    rows = read_rows(RESULTS)
    summary: dict[str, dict] = {}
    errors: dict[str, int] = {}
    for row in rows:
        if row["notes"].startswith("ERROR"):
            errors[row["model"]] = errors.get(row["model"], 0) + 1
            continue
        if not row.get("reported_objects"):
            continue
        expected = [item.strip() for item in row["expected_objects"].split(";") if item.strip()]

        def split(value):
            return [item.strip() for item in value.split(";") if item.strip()]

        if "manually verified" in row["notes"]:
            # Keep the verdicts recorded after checking the image by hand.
            correct = split(row["correct_objects"])
            extra = split(row["hallucinated_objects"])
        else:
            text = row["reported_objects"]
            correct = [label for label in expected if mentioned(label, text)]
            missed = [label for label in expected if label not in correct]
            extra = [label for label in SYNONYMS if label not in expected and mentioned(label, text)]
            row["correct_objects"] = "; ".join(correct)
            row["missed_objects"] = "; ".join(missed)
            row["hallucinated_objects"] = "; ".join(extra)
            if "Keyword-scored" not in row["notes"]:
                row["notes"] = f"{row['notes']}; Keyword-scored, verify hallucinations".lstrip("; ")

        stats = summary.setdefault(row["model"], {"trials": 0, "expected": 0, "correct": 0,
                                                  "extra": 0, "latency": 0.0})
        stats["trials"] += 1
        stats["expected"] += len(expected)
        stats["correct"] += len(correct)
        stats["extra"] += len(extra)
        stats["latency"] += float(row["latency_s"] or 0)
    write_rows(RESULTS, rows)

    print(f"{'model':28s} {'trials':>6s} {'recall':>7s} {'possible halluc.':>17s} "
          f"{'mean latency':>13s} {'failed calls':>13s}")
    for model, stats in summary.items():
        recall = stats["correct"] / stats["expected"] if stats["expected"] else float("nan")
        failed = sum(count for name, count in errors.items()
                     if name == model or model.startswith(name))
        print(f"{model:28s} {stats['trials']:6d} {recall:7.0%} {stats['extra']:17d} "
              f"{stats['latency'] / stats['trials']:12.2f}s {failed:13d}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    steps = parser.add_subparsers(dest="step", required=True)
    capture_parser = steps.add_parser("capture", help="drive to rooms and save 10 scenes (needs sim)")
    capture_parser.add_argument("--timeout", type=float, default=120.0, help="seconds per navigation goal")
    evaluate_parser = steps.add_parser("evaluate", help="run every scene through every model")
    evaluate_parser.add_argument("--providers", nargs="+", default=["gemini", "qwen"],
                                 choices=["gemini", "qwen"])
    steps.add_parser("score", help="keyword-score results and print a summary")
    arguments = parser.parse_args()
    {"capture": capture, "evaluate": evaluate, "score": score}[arguments.step](arguments)


if __name__ == "__main__":
    main()
