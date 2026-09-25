# EE4705 Project 1.2 - AI Robot in a World Model

A language-controlled TurtleBot3 Waffle Pi in Gazebo. You type a request in a
terminal chat ("Go to Room 4 and tell me what you see", "Move to the car
wheel"). The robot parses it with an LLM, drives to the room with Nav2,
describes the camera view with a VLM, and approaches the named object using VLM
grounding and the laser.

```text
User command -> command parser (Qwen, Gemini fallback)
             -> room navigation (Nav2)
             -> scene description (Qwen3-VL-Plus)
             -> object grounding and approach
             -> robot reply
```

Platform: Ubuntu 22.04, ROS 2 Humble, Gazebo Classic 11, Nav2, CycloneDDS.

## Results

Full details, per-trial tables and failure analysis: `docs/results_summary.md`.

| Task | Evaluation | Result | Data |
|---|---|---|---|
| 2 | Command parser, 20 utterances (paraphrases, follow-ups, invalid, out of scope) | Qwen 20/20, Gemini 20/20 (~0.7 s) | `evaluation/command_parser_trials*.csv` |
| 3 | VLM comparison, 10 scenes | Gemini 7/9 objects, 18.3 s mean; Qwen3-VL-Plus 5/9, 1.9 s mean; no hallucinations. Qwen chosen for its latency and reliability | `evaluation/vlm_comparison.md` |
| 4 | Object approach, 12 trials x 3 runs | Run 3 (final): 10/12 arrived, grounding 12/12, 3/4 from out-of-view starts | `evaluation/object_approach_trials.csv` |
| 4 | Grounding prompt on 12 saved frames | 6/12 before, 12/12 after | `evaluation/grounding_prompt_test.csv` |
| 5 | 20 randomized end-to-end trials | 9/20 end-to-end; navigation 20/20, parsing 13/20, description 10/20, approach 10/20 | `evaluation/end_to_end_trials.csv` |

All API use was on free tiers (US$0). Token counts are logged per call.

## Setup

Install ROS 2 Humble first, then:

```bash
cd ~
git clone https://github.com/likingood16/EE4705.git
cd EE4705
bash scripts/install_dependencies_ubuntu.sh   # once
cp .env.example .env                           # then add QWEN_API_KEY (and GEMINI_API_KEY for the fallback)
```

The chat uses Qwen by default for both parsing and vision (`PARSER_PROVIDER`,
`VISION_PROVIDER`). In every new terminal:

```bash
source scripts/activate_ubuntu.sh
```

Build the workspace. Use the `python3 /usr/bin/colcon` form so the installed
scripts use the `.venv` interpreter:

```bash
cd ros2_ws && python3 /usr/bin/colcon build --symlink-install && cd ..
```

The house world needs the Gazebo models `cafe_table`, `car_wheel`,
`cinder_block`, `fire_hydrant`, `first_2015_trash_can` and `table_marble` in
`~/.gazebo/models` (from the Gazebo model database). More setup notes are in
`environment/README.md` and `docs/setup_notes/`.

## Running

Terminal 1, the full simulation (Gazebo, the modified house world, the saved
map, Nav2 and the initial pose):

```bash
source scripts/activate_ubuntu.sh
bash scripts/start_simulation.sh
```

Wait for `Initial AMCL pose was set successfully` (about 30 s). Don't use
Gazebo's Reset World; restart the script instead.

Terminal 2, the chat:

```bash
source scripts/activate_ubuntu.sh
ros2 run ee4705_perception terminal_chat
```

Other entry points: `ros2 run ee4705_bringup system_check` (camera, laser and
odometry check), `ros2 run ee4705_perception goto_room`, `camera_snapshot`,
`vision_demo`, `approach_robot`. The demo walkthrough is in
`docs/demo_script.md`.

## Using the assistant

Adapted from Charansagar's operating guide (Task 2).

### Room navigation

```text
Go to Room 2
Could you head over to the fourth room?
Please check Room 1.
```

The parser turns the request into a `goto_room` command, and the robot sends
that room's waypoint from `config/room_waypoints.yaml` to Nav2. It reports
success or failure in the chat. Unknown rooms ("Go to Room 99") get a
clarification instead of a goal.

### Scene description and follow-up questions

After arriving, or at any time:

```text
What do you see?
Is there anything on the floor?
What colour is the object?
How many objects can you see?
```

The robot captures the current camera frame and sends it to the VLM. The chat
keeps the conversation history, so follow-up questions can refer to earlier turns.

### Object search and approach

```text
Approach the fire hydrant
Find the car wheel
Move to it            (refers to the object just described)
```

```text
request -> object grounding -> target visible?
             no  -> SEARCH (rotate in place and re-query; fail after a full turn)
             yes -> ALIGN (bounding-box centre to image centre)
                 -> MOVE_FORWARD while monitoring the laser front distance
                 -> stop safely near the object and confirm in the chat
```

### Example session

```text
You:   Go to Room 1
Robot: I have arrived in Room 1. I can see a humanoid figure near the wall.
You:   Approach it.
Robot: I am now next to the humanoid.
```

The recorded demo follows `docs/demo_script.md`. It covers two rooms, two
approaches and one approach that starts with the object out of view.

### Operating notes

- **Don't drive with the keyboard during autonomous motion.** Teleop, Nav2 and
  the approach controller all publish velocity commands.
- **Localization looks wrong** (the robot in RViz doesn't match Gazebo): stop and
  rerun `scripts/start_simulation.sh`, which resets the initial pose. Don't use
  Gazebo's Reset World.
- **Nav2 can't make progress**: check localization first. Other causes are a
  blocked path, a narrow doorway or a local costmap obstacle.
- **Robot flung by physics or behaving unrealistically**: restart the simulation script.
- **VLM errors** (`503 UNAVAILABLE`, timeouts): the client retries. If Gemini is
  overloaded, keep the default Qwen provider.

### Sensor checks

```bash
ros2 topic hz /camera/image_raw                 # camera
ros2 run rqt_image_view rqt_image_view          # view the camera
ros2 topic hz /scan                             # laser (used for the approach safety stop)
ros2 run ee4705_bringup system_check            # camera, laser and odometry in one check
```

### Manual launch (alternative)

Instead of `start_simulation.sh`, the world can be started on its own with
`ros2 launch ee4705_perception custom_house.launch.py`, followed by
`turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True
map:=$HOME/EE4705/maps/house_map_final.yaml` and a 2D Pose Estimate in RViz.
This path doesn't use the tuned Nav2 parameters in `config/nav2_waffle_pi.yaml`,
so the evaluation results above were produced with `start_simulation.sh`.

## Tests

The offline unit tests need no simulation and no API keys:

```bash
PYTHONPATH=ros2_ws/src/ee4705_perception python3 -m unittest discover ros2_ws/src/ee4705_perception/test
```

Expected: 117 tests, OK.

## Repository layout

| Path | Contents |
|---|---|
| `ros2_ws/src/ee4705_bringup/` | Task 1: simulation launch, Nav2 bringup, initial pose, scan relay, system check |
| `ros2_ws/src/ee4705_perception/` | Tasks 2-4: room navigation, terminal chat and parser, VLM client, scene description, object grounding and approach, tests |
| `config/` | Room waypoints, Nav2 parameters, CycloneDDS configuration |
| `maps/` | Saved SLAM map (`house_map_final.yaml` / `.pgm`) |
| `worlds/` | Modified Gazebo house world with the objects |
| `scripts/` | Dependency install, environment activation, `start_simulation.sh` |
| `evaluation/` | Evaluation scripts, CSV results and evidence (see below) |
| `docs/` | Results summary, demo script, floor plan, setup notes, Task 4 integration guide, assignment brief |
| `report/`, `demo/` | Report and demo video link |

Each source file starts with a header naming its task and contributors. Who
did what: `CONTRIBUTIONS.md`.

## Where the evidence is

| Evidence | Location |
|---|---|
| Labelled floor plan with room waypoints and objects | `docs/diagrams/floor_plan_labelled.png` |
| Parser trials (Gemini, Qwen) | `evaluation/command_parser_trials.csv`, `command_parser_trials_qwen.csv` |
| Post-evaluation parser replay | `evaluation/parser_approach_replay.csv` |
| VLM comparison: images, responses, scoring | `evaluation/scenes/`, `vlm_scene_trials.csv`, `vlm_comparison.md` |
| Approach trials: every frame sent to the VLM, boxes, JSON per call | `evaluation/task4_evidence/trials/run1/`, `run2/`, `run3/` |
| Approaches from the chat and preflight runs | `evaluation/task4_evidence/chat/`, `preflight/` |
| End-to-end trials: arrival images | `evaluation/e2e_evidence/` |
| Definitions of success for each evaluation | `evaluation/README.md` |

## Security

Never commit API keys, `.env`, ROS build outputs or large video files.
