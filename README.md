# EE4705 Project 1.2 – AI-Bot in World Model

[svg](https://github.com/likingood16/EE4705/edit/main/README.md#ee4705-project-12--ai-bot-in-world-model)

This project implements an integrated human-robot interaction system using TurtleBot3 Waffle Pi, ROS2 Humble, Gazebo Classic, Nav2, RViz2, and Gemini-based language/vision processing.

The final system supports:

- Natural-language room navigation
- Multi-turn terminal chat
- Scene description using the robot camera
- Visual follow-up questions
- Language-directed object search
- Object grounding
- Camera-based alignment
- Autonomous object approach
- LiDAR-based safe stopping
- Natural-language success/failure feedback

The overall workflow is:

`Natural Language → LLM Parser → Navigation / Vision / Approach → Robot Execution → Feedback`

---

## 1. System Requirements

[svg](https://github.com/likingood16/EE4705/edit/main/README.md#1-system-requirements)

Tested environment:

- Ubuntu 22.04
- ROS2 Humble
- Gazebo Classic
- TurtleBot3 Waffle Pi
- Nav2
- RViz2
- Python 3.10
- Google Gemini API

The repository is assumed to be located at:

```
~/EE4705
```

Important final files:

```
EE4705/
├── config/
│   └── room_waypoints.yaml
├── maps/
│   ├── house_map_final.yaml
│   └── house_map_final.pgm
├── worlds/
│   └── house_with_objects.world
└── ros2_ws/
    └── src/
        └── ee4705_perception/
            ├── launch/
            │   └── custom_house.launch.py
            ├── setup.py
            └── ee4705_perception/
                ├── terminal_chat.py
                ├── goto_room.py
                ├── camera_snapshot.py
                ├── scene_describer.py
                ├── vlm_client.py
                ├── object_grounder.py
                ├── approach_controller.py
                ├── approach_policy.py
                ├── approach_velocity.py
                ├── approach_geometry.py
                ├── approach_session.py
                └── search_tracker.py

```

---

## 2. Build the ROS2 Workspace

[svg](https://github.com/likingood16/EE4705/edit/main/README.md#2-build-the-ros2-workspace)

Open an Ubuntu terminal.

If using WSL:

```
wsl -d Ubuntu-22.04
```

Then run:

```
conda deactivate
cd ~/EE4705/ros2_ws

source /opt/ros/humble/setup.bash
colcon build --packages-select ee4705_perception
source install/setup.bash

export TURTLEBOT3_MODEL=waffle_pi
```

Rebuild the package whenever `setup.py` or `launch/custom_house.launch.py` is changed so that ROS2 installs the latest launch file.

---

## 3. Launch the Custom Gazebo World

[svg](https://github.com/likingood16/EE4705/edit/main/README.md#3-launch-the-custom-gazebo-world)

### Terminal 1 – Gazebo

[svg](https://github.com/likingood16/EE4705/edit/main/README.md#terminal-1--gazebo)

```
conda deactivate
cd ~/EE4705/ros2_ws

source /opt/ros/humble/setup.bash
source install/setup.bash

export TURTLEBOT3_MODEL=waffle_pi
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models

ros2 launch ee4705_perception custom_house.launch.py
```

Wait until Gazebo fully loads.

The custom launch file is:

```
ros2_ws/src/ee4705_perception/launch/custom_house.launch.py
```

This launch file loads the final custom world:

```
worlds/house_with_objects.world
```

and preserves the normal TurtleBot3 simulation setup required for the robot TF tree, odometry, LiDAR and camera.

Do **not** launch the custom world directly with `gazebo_ros gazebo.launch.py`, because that may result in an incomplete TurtleBot3 TF setup for Nav2.

Optional TF check after Gazebo has loaded:

```
ros2 run tf2_ros tf2_echo odom base_link
```

A working setup should display repeated translation and rotation values rather than an `Invalid frame ID "odom"` error.

---

## 4. Launch Nav2 and RViz

[svg](https://github.com/likingood16/EE4705/edit/main/README.md#4-launch-nav2-and-rviz)

### Terminal 2 – Nav2 + RViz

[svg](https://github.com/likingood16/EE4705/edit/main/README.md#terminal-2--nav2--rviz)

```
conda deactivate
source /opt/ros/humble/setup.bash

export TURTLEBOT3_MODEL=waffle_pi

ros2 launch turtlebot3_navigation2 navigation2.launch.py \
use_sim_time:=True \
map:=$HOME/EE4705/maps/house_map_final.yaml
```

Wait for Nav2 and RViz to fully load.

The final map is:

```
maps/house_map_final.yaml
maps/house_map_final.pgm

```

The room waypoint table is:

```
config/room_waypoints.yaml

```

---

## 5. Move the Robot Away From the Starting Wall

[svg](https://github.com/likingood16/EE4705/edit/main/README.md#5-move-the-robot-away-from-the-starting-wall)

The TurtleBot3 may initially spawn close to a wall. Move it slightly into an open area before setting the AMCL pose.

### Terminal 3 – Keyboard Teleoperation

[svg](https://github.com/likingood16/EE4705/edit/main/README.md#terminal-3--keyboard-teleoperation)

```
conda deactivate
source /opt/ros/humble/setup.bash

export TURTLEBOT3_MODEL=waffle_pi

ros2 run turtlebot3_teleop teleop_keyboard
```

Use the keyboard controls to move the robot a short distance away from the wall.

Do not move it too far from the initial area.

---

## 6. Set the Initial Pose in RViz

[svg](https://github.com/likingood16/EE4705/edit/main/README.md#6-set-the-initial-pose-in-rviz)

After moving the robot away from the wall:

1. Return to RViz.
2. Click **2D Pose Estimate**.
3. Click approximately where the robot is located on the map.
4. Drag the arrow so that the heading matches the robot orientation in Gazebo.
5. Wait for the AMCL estimate to settle.
6. Confirm that the robot location in RViz approximately matches Gazebo.

Accurate localization is important before autonomous navigation.

---

## 7. Optional Navigation Check

[svg](https://github.com/likingood16/EE4705/edit/main/README.md#7-optional-navigation-check)

Before launching the language interface, you can verify Nav2 by using **Nav2 Goal** in RViz.

If the robot can navigate correctly to the selected position, localization and Nav2 are ready.

The final system supports six numbered room waypoints stored in:

```
config/room_waypoints.yaml

```

---

## 8. Configure the Gemini API Key

[svg](https://github.com/likingood16/EE4705/edit/main/README.md#8-configure-the-gemini-api-key)

The integrated LLM/VLM system requires a valid Gemini API key.

In the terminal that will run the chat interface:

```
export GEMINI_API_KEY="YOUR_GEMINI_API_KEY"
```

To check that the key is set:

```
echo $GEMINI_API_KEY
```

Do not commit API keys to GitHub.

---

## 9. Launch the Integrated Terminal Assistant

[svg](https://github.com/likingood16/EE4705/edit/main/README.md#9-launch-the-integrated-terminal-assistant)

### Terminal 4 – Integrated LLM/VLM Chat

[svg](https://github.com/likingood16/EE4705/edit/main/README.md#terminal-4--integrated-llmvlm-chat)

```
conda deactivate
cd ~/EE4705/ros2_ws

source /opt/ros/humble/setup.bash
source install/setup.bash

export TURTLEBOT3_MODEL=waffle_pi
export GEMINI_API_KEY="YOUR_GEMINI_API_KEY"

ros2 run ee4705_perception terminal_chat
```

The interface should appear as:

```
==========================================
   EE4705 TurtleBot3 Terminal Assistant
==========================================

Type a command and press ENTER.
Type 'exit' to close the program.

You:

```

---

## 10. Room Navigation

[svg](https://github.com/likingood16/EE4705/edit/main/README.md#10-room-navigation)

Example commands:

```
Go to Room 2

```

```
Could you head over to the fourth room?

```

```
Please check Room 1.

```

The LLM converts the natural-language request into a structured room-navigation command and sends the corresponding waypoint to Nav2.

---

## 11. Scene Description

[svg](https://github.com/likingood16/EE4705/edit/main/README.md#11-scene-description)

After reaching a room, ask:

```
What do you see?

```

The robot captures its current camera frame and sends it to the VLM for scene description.

Example:

```
You: What do you see?

Robot: I can see a humanoid figure and a dark grey wheel near the wall.

```

---

## 12. Visual Follow-Up Questions

[svg](https://github.com/likingood16/EE4705/edit/main/README.md#12-visual-follow-up-questions)

The system supports follow-up visual questions such as:

```
Is there anything on the floor?

```

```
What colour is the object?

```

```
How many objects can you see?

```

The terminal chat maintains conversation history so follow-up questions can refer to previous turns.

---

## 13. Object Search and Approach

[svg](https://github.com/likingood16/EE4705/edit/main/README.md#13-object-search-and-approach)

Example:

```
Approach the fire hydrant

```

or:

```
Find the fire hydrant

```

The object-approach workflow is:

```
Natural-language request
→ object grounding
→ target visible?
    → No: SEARCH
    → Yes: ALIGN
→ MOVE_FORWARD
→ monitor LiDAR distance
→ stop safely near object

```

If the object is not initially visible, the robot rotates in place and repeatedly checks the camera until the target becomes visible or the search fails/times out.

---

## 14. Search, Alignment and Approach Behaviour

[svg](https://github.com/likingood16/EE4705/edit/main/README.md#14-search-alignment-and-approach-behaviour)

If the target is outside the camera field of view, the controller enters search mode.

Example terminal output:

```
[Approach] Looking for fire hydrant...
[Approach] found=False, action=search

```

Once the target becomes visible, the controller uses the target bounding-box centre to decide whether to turn left, turn right, or move forward.

The controller uses separate motion durations for search, fine alignment and forward movement to reduce overshoot.

The LiDAR scanner is used for front-distance safety checks and stopping before collision.

---

## 15. Multi-Turn Interaction

[svg](https://github.com/likingood16/EE4705/edit/main/README.md#15-multi-turn-interaction)

Conversation history is retained.

Example:

```
You: What do you see?

Robot: I can see a humanoid figure and a dark grey wheel.

You: Approach it.

```

The LLM uses previous conversation context to resolve the referenced object when possible.

---

## 16. Example Complete Interaction

[svg](https://github.com/likingood16/EE4705/edit/main/README.md#16-example-complete-interaction)

```
You: Go to Room 1

Robot: Navigating to Room 1...
Robot: I have arrived in Room 1.

You: What do you see?

Robot: I can see a humanoid figure and several objects in the room.

You: Approach the humanoid.

Robot: Approaching the humanoid...

[SEARCH / ALIGN / MOVE FORWARD]

Robot: I have reached the humanoid.

```

---

## 17. Recommended Demo Sequence

[svg](https://github.com/likingood16/EE4705/edit/main/README.md#17-recommended-demo-sequence)

A suitable full-system demonstration is:

```
1. Go to Room 4
2. What do you see?
3. Approach the wheel/it
4. Go to Room 1
5. What do you see?
6. Find the fire hydrant

```

---

## 18. Important Operating Notes

[svg](https://github.com/likingood16/EE4705/edit/main/README.md#18-important-operating-notes)

### Do Not Use Keyboard Control During Autonomous Motion

[svg](https://github.com/likingood16/EE4705/edit/main/README.md#do-not-use-keyboard-control-during-autonomous-motion)

The following systems can command robot motion:

- keyboard teleoperation
- Nav2
- object-approach controller

Do not press movement keys while Nav2 or the object-approach controller is active.

Use keyboard teleoperation only for initial positioning, setup and manual testing.

### Localization Problems

[svg](https://github.com/likingood16/EE4705/edit/main/README.md#localization-problems)

If navigation behaves incorrectly:

1. Compare the robot position in Gazebo and RViz.
2. Re-run **2D Pose Estimate**.
3. Check that the orientation is correct.
4. Retry navigation.

Poor localization can cause Nav2 failure even when the requested waypoint is correct.

### Nav2 Failure

[svg](https://github.com/likingood16/EE4705/edit/main/README.md#nav2-failure)

Navigation may fail because of:

- poor localization
- blocked path
- narrow doorway
- local costmap obstacle
- robot unable to make progress

Check localization before modifying the navigation code.

### Gazebo Physics Instability

[svg](https://github.com/likingood16/EE4705/edit/main/README.md#gazebo-physics-instability)

If the robot suddenly moves unrealistically or is launched by collision forces:

1. Stop Gazebo.
2. Restart the custom world.
3. Restart Nav2.
4. Set the initial pose again.

### Gemini API Availability

[svg](https://github.com/likingood16/EE4705/edit/main/README.md#gemini-api-availability)

Gemini may occasionally return temporary service errors such as:

```
503 UNAVAILABLE
This model is currently experiencing high demand

```

Retry the command after a short delay.

### Always check if the simulation is playing as it can sometimes turn off**

---

## 19. Camera Check

[svg](https://github.com/likingood16/EE4705/edit/main/README.md#19-camera-check)

To verify that the camera topic is active:

```
ros2 topic hz /camera/image_raw
```

To view the onboard camera:

```
ros2 run rqt_image_view rqt_image_view
```

---

## 20. LiDAR Check

[svg](https://github.com/likingood16/EE4705/edit/main/README.md#20-lidar-check)

To verify the LiDAR topic:

```
ros2 topic hz /scan
```

The object-approach controller uses `/scan` for front-distance safety checks.

---

## 21. Final Launch Order

[svg](https://github.com/likingood16/EE4705/edit/main/README.md#21-final-launch-order)

Use this order for normal operation:

```
1. Launch the custom Gazebo world using `ros2 launch ee4705_perception custom_house.launch.py`
2. Launch Nav2 + RViz
3. Launch keyboard teleoperation
4. Move robot slightly away from the starting wall
5. Set 2D Pose Estimate in RViz
6. Confirm localization
7. Set Gemini API key
8. Launch terminal_chat
9. Enter natural-language commands

```

---

## 22. Shutdown

[svg](https://github.com/likingood16/EE4705/edit/main/README.md#22-shutdown)

Exit the terminal assistant using:

```
exit

```

Stop ROS2 processes with:

```
Ctrl+C

```

Recommended shutdown order:

```
1. Terminal Assistant
2. Keyboard Teleoperation
3. Nav2 / RViz
4. Gazebo

```

---

## 23. Final System Summary

[svg](https://github.com/likingood16/EE4705/edit/main/README.md#23-final-system-summary)

The final integrated system supports:

- Natural-language room navigation
- Six room waypoints
- Nav2 autonomous navigation
- Multi-turn terminal interaction
- VLM scene description
- Visual follow-up questions
- Object grounding
- Search when a target is not initially visible
- Camera-based alignment
- Autonomous forward approach
- LiDAR-based stopping
- Natural-language success/failure feedback

The complete workflow is:

```
Natural-Language Instruction
→ LLM Parsing
→ Navigation / Vision / Approach
→ Robot Execution
→ User Feedback

```
