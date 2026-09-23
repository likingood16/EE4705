# Ubuntu 22.04 command checklist

Every command needed to go from a fresh Ubuntu 22.04 install to a working project.
Run each block in order and wait for it to finish before starting the next.

## 1. Install ROS2 Humble (once)

```bash
sudo apt update && sudo apt install -y locales software-properties-common curl git
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo add-apt-repository -y universe

export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

sudo apt update && sudo apt upgrade -y
sudo apt install -y ros-humble-desktop
```

If any step fails, follow the official guide:
https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

## 2. Git and clone (once)

```bash
git config --global user.name "Your Name"
git config --global user.email "your-github-email@example.com"
cd ~
git clone https://github.com/likingood16/EE4705.git
cd EE4705
```

When Git asks for a password on push, use a GitHub personal access token
(GitHub -> Settings -> Developer settings -> Tokens), not your account password.

## 3. Project dependencies (once)

```bash
bash scripts/install_dependencies_ubuntu.sh
```

## 4. Optional tools (once)

VS Code:

```bash
sudo snap install code --classic
cd ~/EE4705
code .
```

Recommended extensions: Python and ROS (Microsoft).

## 5. Build the workspace (once, and after every pull)

```bash
cd ~/EE4705/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
cd ~/EE4705
source scripts/activate_ubuntu.sh
```

## 6. Every new terminal (including the VS Code terminal)

```bash
cd ~/EE4705 && source scripts/activate_ubuntu.sh
```

## 7. Task 1 system check (2 terminals)

Terminal 1:

```bash
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```

Terminal 2:

```bash
ros2 run ee4705_bringup system_check
```

Expected: `SYSTEM CHECK PASSED`.

Optional:

```bash
ros2 run turtlebot3_teleop teleop_keyboard
ros2 run rqt_image_view rqt_image_view
```

## 8. Vision starter (Task 3)

```bash
pip install -e ros2_ws/src/ee4705_perception
python -m unittest discover ros2_ws/src/ee4705_perception/test -v
vision_demo --image ~/Pictures/room.jpg --provider mock
```

With a real VLM:

```bash
export OPENAI_API_KEY="your-key"
vision_demo --image ~/Pictures/room.jpg --provider openai-compatible --model YOUR_MODEL_NAME \
  --log evaluation/vlm_scene_trials.csv --trial-id prototype-001 --scene-id prototype-room
```

## 9. Build and save the map (3 terminals)

Terminal 1:

```bash
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```

Terminal 2:

```bash
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
```

Terminal 3 - drive through the whole house, then save:

```bash
ros2 run turtlebot3_teleop teleop_keyboard
ros2 run nav2_map_server map_saver_cli -f ~/EE4705/maps/house
```

Test navigation on the saved map (close Cartographer first):

```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/EE4705/maps/house.yaml
```

In RViz use "2D Pose Estimate", then "Nav2 Goal". To read room coordinates for
waypoints, click with "Publish Point" while running:

```bash
ros2 topic echo /clicked_point
```

## 10. Save work to GitHub

```bash
git switch main
git pull origin main
git switch -c feature/short-description
git status
git add <files-you-changed>
git commit -m "Describe the change"
git push -u origin feature/short-description
```

Then open a pull request on GitHub.
