# ee4705_bringup

This ROS2 package contains shared launch and diagnostic tools. The first tool
checks that the simulated TurtleBot3 publishes camera, laser, and odometry data.

Build from the workspace root:

```bash
cd ~/EE4705/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

With the TurtleBot3 house simulation running, execute:

```bash
ros2 run ee4705_bringup system_check
```

The command exits successfully only after messages arrive on
`/camera/image_raw`, `/scan`, and `/odom`.
