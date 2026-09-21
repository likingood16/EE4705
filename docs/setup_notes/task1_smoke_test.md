# Task 1 simulator smoke test

Use this test before starting navigation or VLM development.

## Terminal 1 - launch Gazebo

```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```

Wait until the house and robot are visible. Keep this terminal open.

## Terminal 2 - build and run the system check

```bash
cd ~/EE4705
git pull origin main
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 run ee4705_bringup system_check
```

Expected final message:

```text
SYSTEM CHECK PASSED: camera, laser, and odometry are available.
```

## If the check fails

List the available topics:

```bash
ros2 topic list
```

Confirm that `TURTLEBOT3_MODEL` is set to `waffle_pi`:

```bash
echo $TURTLEBOT3_MODEL
```

If a topic has a different name, pass it as a ROS parameter. Example:

```bash
ros2 run ee4705_bringup system_check --ros-args \
  -p camera_topic:=/different/camera/topic
```

Record any problem and solution in this folder for the Task 1 report section.
