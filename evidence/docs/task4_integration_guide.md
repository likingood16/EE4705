# Student C: Task 4 integration handoff

## Update: validated in Gazebo and connected to the chat

The sections below are the original fake-ROS handoff. Since then the
controller has been tested in the live simulation, and the pulse design
(at most 4 cm per VLM call) was replaced, because a 3 m approach needed ~75
calls. Current behaviour of `approach_robot.run_approach`:

1. Stop, settle, grab a fresh frame, ask the VLM (Qwen3-VL by default) for the
   box while stopped. The robot must not move during the request.
2. Bearing from the box centre (pinhole model, 62 degree HFOV). Range from the
   laser along that bearing, or from the box's bottom edge on the floor when
   the laser (0.13 m scan plane) passes over a low object.
3. Closed-loop turn on odometry to face the target, then a closed-loop drive
   of up to 1.2 m, stopping early if anything in the robot-width laser
   corridor is closer than 0.43 m. Rotation needs 0.32 m all-around clearance.
4. Arrival when the target range is <= 0.60 m (bumper ~0.4 m from the object)
   and it is within 10 degrees of centre. Reply: "I am now next to the <object>."
5. Not visible: turn 45 degrees and look again; after a full turn (8 views)
   reply "Sorry, I could not find the <object> after turning a full circle."

Preflight checks in the live simulation: `/cmd_vel` is `geometry_msgs/Twist`;
scan angle 0 is straight ahead and +inf means no return (so `--inf-is-clear`
is correct); image right is robot right (VLM bearing -5.3 deg vs Gazebo
ground truth -5.0 deg); the node uses simulation time. Camera subscribers use
RELIABLE QoS (see `config/cyclonedds.xml`).

Chat: "move to the fire hydrant" runs the approach with motion enabled
(`TASK4_ENABLE_MOTION=0` makes it observation-only; `GROUNDING_PROVIDER`
selects qwen or gemini). Standalone:

```bash
ros2 run ee4705_perception approach_robot --target "fire hydrant" \
  --inf-is-clear                                  # observation only
ros2 run ee4705_perception approach_robot --target "fire hydrant" \
  --inf-is-clear --enable-motion --exclusive-control
```

Trial results: `evaluation/object_approach_trials.csv`; evidence images (with
the box, bearing and range drawn on) under `evaluation/task4_evidence/`.

---

## Original handoff: status and scope

Prepared against uploaded commit `4d2846f20dcad0f29b57c343ed7db7e6087cd06d`.
The 57 supplied tests pass unchanged; 29 new tests pass (86 total).
The new tests use fake ROS transport and fake images, NOT ROS/Gazebo or an API.
No real model inference, camera decoding, DDS connectivity, Gazebo physics,
collision avoidance, or measured arrival has been verified here.

New code: `approach_runtime.py`, `approach_robot.py`, and two test modules.
Modified: perception `setup.py` (one executable) and `package.xml` (dependencies).
Your teammates' chat, navigation and VLM files are unchanged.
The archive is a changes-only handoff; do not replace the whole repository.

Authorship: prepared with AI assistance for Student C. Read and understand the
code, replace placeholder authorship with your actual contribution/reviewer
details, and record it in the group's contribution and AI-use declarations.

## 1. Apply on your Mac

Use the patch from this handoff while on a clean `integration/task4-approach`:

```bash
cd ~/projects/EE4705
git branch --show-current
git status
git apply --check ~/Downloads/EE4705-task4-controller/task4_controller.patch
git apply ~/Downloads/EE4705-task4-controller/task4_controller.patch
source ~/venvs/ee4705/bin/activate
python -m unittest discover ros2_ws/src/ee4705_perception/test -v
git diff --check
git status
```

Adjust the Downloads folder if your browser extracted the archive elsewhere.
If `git apply --check` fails, STOP and inspect; do not use `--reject`, force,
reset, or overwrite teammate files. Applying this patch twice will fail.
Expected: 86 tests pass. The fake ROS tests need no extra pip dependencies.

Review the code and commit only these paths:

```bash
git add docs/task4_integration_guide.md
git add ros2_ws/src/ee4705_perception/ee4705_perception/approach_runtime.py
git add ros2_ws/src/ee4705_perception/ee4705_perception/approach_robot.py
git add ros2_ws/src/ee4705_perception/test/test_approach_runtime.py
git add ros2_ws/src/ee4705_perception/test/test_approach_robot_fake_ros.py
git add ros2_ws/src/ee4705_perception/setup.py
git add ros2_ws/src/ee4705_perception/package.xml
git diff --cached --stat
git commit -m "Add experimental ROS object approach adapter and runtime tests"
git push -u origin integration/task4-approach
```

## 2. Prepare the group's Ubuntu machine

Do not change your teammate's dirty working tree. Save their work first using
their normal process. Obtain their agreement before changing branches.
Use the REAL full clone, not the review ZIP: the ZIP omitted `environment/`,
maps, and world assets needed by their existing setup scripts.

```bash
git fetch origin
git switch integration/task4-approach
git pull --ff-only
source scripts/activate_ubuntu.sh
python -m pip install -r requirements.txt
cd ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select ee4705_perception
source install/setup.bash
cd ..
python -m unittest discover ros2_ws/src/ee4705_perception/test -v
ros2 run ee4705_perception approach_robot --help
```

The group script uses its Ubuntu system-Python virtual environment with ROS
packages visible. Never copy your Mac virtual environment onto Ubuntu. Confirm
the interpreter used by `ros2 run` can import `google.genai`, `cv2`, `cv_bridge`
and `rclpy`; if not, fix the group's interpreter/build environment before moving.
The pinned Google SDK in the group requirements is preserved, not independently
version-validated. Use a model ID that Student B has actually called successfully;
the fallback model name in their code alone is not evidence it is available.

## 3. Preflight: keep motion disabled

Launch the group's actual modified world and robot using their tested commands.
Camera and scan frames must be fixed to the robot: scan angle zero forward,
camera image right corresponds to robot right, camera not mirrored. The adapter
does not perform TF transformations for misaligned sensors.

```bash
ros2 topic info /camera/image_raw -v
ros2 topic info /scan -v
ros2 topic info /odom -v
ros2 topic info /cmd_vel -v
ros2 topic hz /camera/image_raw
```

Use Ctrl+C to leave `topic hz`. Confirm:

- RGB camera, LaserScan and Odometry are publishing and advancing timestamps.
- `/cmd_vel` expects `geometry_msgs/msg/Twist`, not TwistStamped.
- The pose and laser frames/conventions match the assumptions above.
- Simulator `/clock` and the node use the same time base.
- Gemini API access works on this computer using the privately configured key.
- A verified base command timeout/watchdog stops movement if commands disappear.
  If none exists, add/configure it before motion tests; this Python process
  cannot stop a robot after process kill, host crash or DDS loss.

The controller uses sensor-data QoS, wall-clock deadlines and timestamp freshness.
Sensor receipt age >0.75 s blocks movement. Header timestamps older than 1 s,
future by >0.1 s, repeated or backwards are rejected. On simulation reset,
restart the command. Stale sensors and paused simulation must not resume movement.

Set the model name (NOT the key):

```bash
export TASK4_MODEL="REPLACE_WITH_STUDENT_B_VERIFIED_VISION_MODEL"
ros2 run ee4705_perception approach_robot \
  --target "red cup" --model "$TASK4_MODEL" \
  --ros-args -p use_sim_time:=true
```

Default is observation-only: it takes one live image, queries Gemini, draws a
box, prints a decision, saves evidence and publishes NO velocity messages.
The publisher is registered but sends nothing. A real API call may incur costs.
Grounding uses the existing custom `found/label/bbox` schema with x1,y1,x2,y2
normalized to 0..1000. Do not assume Gemini obeys it: inspect the annotation.
If a provider returns y1,x1,y2,x2 or pixels, adapt that provider explicitly.

The output prints the evidence folder. Compare `frame-000.jpg`,
`box-000.jpg` (when found), and `frame-000.json`. Test an absent object too.
If boxes target the wrong object, swap axes, or drift: STOP integration and
correct prompting/coordinate handling before allowing movement.

Laser +infinity is conservatively invalid by default. If the simulator's sensor
has been verified to encode no return as +infinity, add `--inf-is-clear`; this
maps those rays to range_max. NaN and invalid rays still block motion. A value
below range_min is treated as blocked. Rotation requires approximately 360-degree
scan coverage and minimum all-around clearance. Never bypass a missing scan.

## 4. Motion tests: simulation only

Nav2 must have no active goal and teleop/other controllers must have relinquished
velocity control. `--exclusive-control` is an OPERATOR ASSERTION, not an automatic
arbitrator. Publishing zero cannot override another process continuously publishing
nonzero commands. For robust multi-controller operation, use a velocity mux with
an explicit ownership/safety policy. Do not run multiple approach instances.

Verify 0.35 m laser clearance covers the actual footprint plus scanner offset and
stopping margin. It is not universally safe for every robot. Start in an open
area with floor-level, collidable objects visible to the scan. A cup on a table
may be unreachable; a table leg/edge may cause an obstacle stop instead of arrival.

In a SECOND sourced terminal, keep this cancellation command ready:

```bash
ros2 service call /task4/stop std_srvs/srv/Trigger '{}'
```

It latches cancellation for this attempt only; it does not cancel Nav2 or another
controller. Test its response while Gemini is waiting BEFORE enabling motion.
Standalone Ctrl+C also attempts a graceful zero-velocity stop. Neither is a
hardware emergency stop. The stop service exists only while the attempt runs.

Only after all preceding checks, run:

```bash
ros2 run ee4705_perception approach_robot \
  --target "red cup" --model "$TASK4_MODEL" \
  --enable-motion --exclusive-control \
  --ros-args -p use_sim_time:=true
```

Behavior: stop -> settle 0.3 s -> obtain new frame -> request VLM while stopped
and servicing ROS -> validate -> move briefly -> stop -> repeat.
Forward speed 0.08 m/s; alignment 0.25 rad/s; search 0.20 rad/s.
Forward/alignment pulses last <=0.5 s; search pulses <=1.75 s (~20 degrees).
Sensors are checked every loop (nominal 20 ms spin wait; Python/ROS are not
hard-real-time). Obstacle/stale data/cancellation interrupts the movement pulse.

Bounds: 180 s overall; 20 s per VLM wait; 20 requests; 90 s per search episode;
approximately one measured revolution per search. Limits may end a search before
a full turn with a slow provider. Change limits only with a documented reason.
Search rotation measures total angular travel, including jitter/reversals;
it is a bound, not proof of perfect visual room coverage. No object identity
tracking or planning around intervening obstacles is implemented.

If the robot moves >3 cm or >0.05 rad while waiting for a model response, the
attempt aborts rather than using that old view. This assumes a static simulated
scene; moving objects/people require a tracker or more conservative design.

Required tests (record PASS/FAIL; not pre-filled):

| Test | Expected |
| --- | --- |
| Observation only | No velocity messages, saved box correctly matches object |
| Target left/right | Turns in correct direction, no forward component |
| Centred distant target | Short forward pulse followed by stop |
| Target missing | Bounded rotation and re-query, limit stops the attempt |
| Stop service during API | Prompt cancellation, zero movement, late answer ignored |
| Stop service during pulse | Stops promptly and cannot resume |
| Scan/odom/camera interrupted | Stops; no movement with stale data |
| API delay/failure/malformed box | Stops, clear failure result, no retries in motion |
| Obstacle front/side during turn | Stops using appropriate scan sector |
| Paused world/reset | No resumed motion from stale data; restart after reset |
| Process killed in simulation | Base watchdog stops it without Python cleanup |

## 5. Results are evidence, not automatic success scores

Each attempt has a unique folder containing input images, annotated boxes,
per-call JSON, `outcome.json`, and one-row `trial.csv`. The CSV is mode=simulation
but observation-only rows must be excluded from approach success statistics.
Grounding correctness, approach success, final distance and API cost are left
blank deliberately. A laser range is NOT automatically distance to the target.

The program reports `visual_arrival_candidate` and says final distance needs
verification. It never claims certified success based only on box height.
The 0.55 height threshold is provisional. Calibrate per object type/size and
confirm with simulator ground truth. `--close-height` changes that threshold.
Manually record ground-truth robot-front-to-target distance and the measurement
method; define an acceptable interval before trials. Include collisions,
timeouts and wrong-object approaches as failures. API call count is attempted
calls and may include failed/timed-out requests. Timed-out HTTP work can still
finish and incur charges, but its isolated worker cannot command motion.

At least 10 actual approach trials: vary objects and start poses, include initially
invisible objects and failures. Unit-test counts are not grounding accuracy.
Save prompts/model IDs/boxes/latencies, and use provider usage to calculate cost.
The existing VLM wrapper does not populate cost_usd; do not substitute zero.

## 6. Chat integration AFTER standalone tests pass

No automatic chat edits were made. Review this hook with Student A. Their chat
initializes ROS already; run_approach reuses it and destroys only its own node.
Do not call rclpy.init/shutdown around this function inside the chat.

Replace only the existing approach placeholder with a guarded call, for example:

```python
elif action == "approach":
    from ee4705_perception.approach_robot import run_approach
    obj = command.get("object")
    if not isinstance(obj, str) or not obj.strip():
        reply = "Which object should I approach?"
    else:
        try:
            result = run_approach(
                obj,
                GeminiVLMClient(model=os.environ["GEMINI_VISION_MODEL"]),
                enable_motion=os.getenv("TASK4_ENABLE_MOTION") == "1",
                exclusive_control=os.getenv("TASK4_EXCLUSIVE_CONTROL") == "1",
                evidence_dir=PROJECT_ROOT / "evaluation" / "task4_evidence",
            )
            reply = result.reply
        except Exception:
            reply = "Approach could not start; check the model and ROS configuration."
```

Set only the verified Gemini model in the local environment; never commit keys.
Run chat with matching simulated time. Configure inf_is_clear only if verified.
Motion flags default OFF. Start by checking the hook in observation-only mode.

Important remaining manager work:

- Its blocking input loop cannot read typed `stop` during navigation/approach.
  Use the separate stop service for supervised approach testing. To claim a
  responsive natural-language stop, Student A must refactor command execution
  to a worker with cancellation and keep input responsive. Do NOT invoke the
  LLM parser for an urgent stop; exact stop/quit should bypass it.
- The existing idle `stop` branch only prints `Stopped.`. Change that misleading
  reply or implement actual goal cancellation and controller stopping.
- Its default ROS Ctrl+C handler may shut down ROS before approach cleanup can
  publish zero. Use the standalone CLI and separate service for initial tests;
  the manager needs coordinated signal handling/cleanup (including Nav2 cancel)
  before chat motion is enabled.
- GotoRoom does not retain its goal handle for cancellation and waits for the
  action server without a deadline. Student A should address both; destroying a
  client node alone does not cancel an accepted goal.
- After arrival, the current chat code does not auto-describe the room. Student A
  should wire this if required for the reference multi-action interaction.
- Full Task 5 evaluation, demo, contribution declarations and report remain.

## Reference API documentation

- ROS 2 Humble sensor QoS: https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html
- rclpy Humble implementation: https://github.com/ros2/rclpy/tree/humble/rclpy/rclpy
- LaserScan fields: https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/LaserScan.msg
- Trigger service: https://github.com/ros2/common_interfaces/blob/humble/std_srvs/srv/Trigger.srv

The adapter is an experimental simulation starter, not a real-robot safety system.
