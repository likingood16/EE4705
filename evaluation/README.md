# Evaluation records

Use these CSV templates while testing instead of creating results from memory at
the end. Add one row per trial and preserve failed runs.

- `command_parser_trials.csv` - at least 20 utterances
- `vlm_scene_trials.csv` - at least 2 models across 10 scenes
- `object_approach_trials.csv` - at least 10 trials
- `end_to_end_trials.csv` - at least 20 randomized trials

Define success criteria before running the final evaluation. Record latency, cost,
and failure reasons as required by the brief.

## Task 3 VLM comparison (Gemini vs Qwen, 10 scenes)

`run_vlm_comparison.py` produces the 2 models x 10 scenes = 20 trials. Put
`GEMINI_API_KEY` and `QWEN_API_KEY` in `.env` (see `.env.example`), then from the
project root after `source scripts/activate_ubuntu.sh`:

```bash
# 1. With the simulation running: drive to the rooms and save 10 camera frames.
python evaluation/run_vlm_comparison.py capture
# Check evaluation/scenes/comparison/scenes.csv: expected_objects is estimated
# from the world file and map; correct it against the images if needed.

# 2. No simulation needed: send every scene to both models (resumable).
python evaluation/run_vlm_comparison.py evaluate

# 3. Fill correct/missed/hallucinated objects and print the summary table.
python evaluation/run_vlm_comparison.py score
```

Results go to `vlm_scene_trials.csv`. Keyword scoring flags possible
hallucinations; confirm them against the images before quoting numbers.

## Task 4.iii object-approach trials (defined before running)

`run_approach_trials.py` runs 12 fixed trials: 4 objects (fire hydrant, car
wheel, cinder block, the human figure asked for as "person"), 3 start poses
each (straight ahead, 15-25 deg off-centre, and out of view: behind or 90-120
deg to the side, 4 trials in total). The robot is teleported to each start
pose in Gazebo; the approach itself is the same code the chat runs.

- **Initially visible**: the first VLM query found the target.
- **Grounding correct**: judged from the evidence images: the named object
  is boxed at least once, every box drawn marks the named object, and no frame
  boxes a different object. (Tightened before grading run 1: as first written,
  a trial with no boxes at all would have counted as correct.) Recorded with
  `run_approach_trials.py grade`.
- **Approach success**: the controller reports arrival AND grounding is
  correct AND the robot centre ends within 1.0 m of the object centre
  (nearest body link for the human figure), measured from Gazebo model poses
  AND the final laser range to the object is at least 0.20 m (no contact).
- **Final distance**: robot centre to object centre (Gazebo ground truth).
- **Time**: from the start of the approach to the controller's reply.
- **API calls / tokens**: grounding requests, including retries.

## Task 5.ii end-to-end trials (defined before running)

`run_end_to_end_trials.py` runs 20 randomized two-turn conversations through
`terminal_chat.handle_turn()` (the same code as the interactive chat). A
seeded random generator picks, per trial, a room that has an object in view
of its waypoint, the phrasing of the navigation request and the phrasing of
the approach request. Turn 1 asks the robot to go to the room (it then
describes what it sees); turn 2 asks it to move to that room's object.

- **Parsing**: turn 1 parses to `goto_room` with the right room number AND
  turn 2 parses to `approach` with an object name naming the target (keyword
  match, e.g. "hydrant").
- **Navigation**: Nav2 reports success AND the robot ends within 0.5 m of the
  room waypoint (Gazebo ground truth).
- **Description**: the arrival description names the room's target object
  (keyword match, then checked against the saved image) and names no object
  that is not in view.
- **Approach**: same definition as Task 4.iii above.
- **End-to-end success**: all four stages succeed in the same trial.
- **Latency per stage**: wall-clock time of that stage as measured by the
  chat (parser call, Nav2 goal, camera capture + VLM description, approach).
- **API cost**: calls and tokens per stage. Gemini and Qwen are both used on
  their free tiers, so the money spent is US$0; tokens are reported so the
  cost at paid rates can be worked out.
