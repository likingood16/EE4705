# EE4705 Project 1.2 - AI Robot in a World Model

This repository contains the group's shared implementation of a language-controlled
TurtleBot3 robot. The final system will accept natural-language instructions,
navigate between rooms, describe camera views, and approach requested objects.

## Project target

```text
User command
  -> command manager
  -> room navigation
  -> scene understanding
  -> object grounding and approach
  -> robot reply
```

The recommended platform from the project brief is:

- Ubuntu 22.04
- ROS2 Humble
- Gazebo
- TurtleBot3 Waffle Pi (the model with a camera)
- Nav2

## Repository layout

| Path | Purpose |
|---|---|
| `ros2_ws/src/` | ROS2 packages and robot source code |
| `config/` | Room waypoints and shared configuration |
| `maps/` | Saved SLAM map files (`.yaml` and `.pgm`) |
| `worlds/` | Modified Gazebo world and model files |
| `evaluation/` | CSV templates and experiment results |
| `docs/` | Assignment brief, research, setup notes, and diagrams |
| `report/` | Group report source and final PDF |
| `demo/` | Demo instructions and the submitted video link |
| `scripts/` | Setup and convenience scripts |

Member allocation is intentionally left open. Record the agreed allocation in
`CONTRIBUTIONS.md` after the group decides.

## Required deliverables

- At least 4 numbered rooms and 3 distinct objects
- At least 20 command-parser tests, including 5 paraphrases and 5 invalid requests
- At least 2 VLMs compared on 10 scenes
- At least 10 object-grounding and approach trials
- At least 20 randomized end-to-end trials
- One uncut 3-5 minute demo covering 2 rooms and 2 object approaches
- One 8-15 page group report
- Source code, README, map, world, waypoints, results, and demo included in submission

Submission deadline: **27 September 2026**.

## First-time setup

Clone the repository inside Ubuntu:

```bash
cd ~
git clone https://github.com/likingood16/EE4705.git
cd EE4705
```

Install the shared dependencies once:

```bash
bash scripts/install_dependencies_ubuntu.sh
```

For every new terminal, activate the project environment:

```bash
source scripts/activate_ubuntu.sh
```

Do not copy or commit `.venv`. GitHub stores the dependency recipe, and each
member creates their own local environment from it. See `environment/README.md`.

After ROS2 packages are added, build the workspace with:

```bash
cd ~/EE4705/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Verify the simulator

Terminal 1:

```bash
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```

Terminal 2:

```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

Terminal 3:

```bash
ros2 topic hz /camera/image_raw
ros2 run rqt_image_view rqt_image_view
```

## Group Git workflow

Do not develop directly on `main`. Create a short-lived branch for one feature:

```bash
git switch main
git pull origin main
git switch -c feature/short-description
```

Save and publish the work:

```bash
git add <files-you-changed>
git commit -m "Describe the completed change"
git push -u origin feature/short-description
```

Open a pull request on GitHub, ask another member to review it, and merge only
after the relevant test works. See `CONTRIBUTING.md` for the complete workflow.

## Security

Never commit API keys, passwords, `.env` files, ROS build outputs, or large video
files. Commit an unlisted video link in `demo/README.md` instead of the video when
it exceeds GitHub's practical file-size limit.
