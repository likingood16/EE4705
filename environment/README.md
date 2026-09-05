# Shared development environment

The team shares the files that describe the environment, not one copied virtual
environment. A `.venv` contains machine-specific paths and can be very large, so
every member creates it locally from the same committed files.

## What is shared through GitHub

- `environment/versions.env`: agreed Ubuntu, ROS2, Python, and robot versions
- `requirements.txt`: Python packages installed with pip
- ROS2 `package.xml` files: dependencies for each ROS2 package
- `scripts/install_dependencies_ubuntu.sh`: one-time Ubuntu setup
- `scripts/activate_ubuntu.sh`: environment activation for each terminal

The generated `.venv`, `ros2_ws/build`, `ros2_ws/install`, and `ros2_ws/log`
folders stay local and are ignored by Git.

## First setup on Ubuntu 22.04

Install ROS2 Humble first using the official ROS2 Ubuntu instructions. Then clone
this repository and run:

```bash
cd ~/EE4705
bash scripts/install_dependencies_ubuntu.sh
```

The script installs the shared TurtleBot3, Nav2, Gazebo, camera, build, and Python
dependencies. It then creates `.venv` and asks `rosdep` to install dependencies
declared by packages under `ros2_ws/src`.

## Start work in a new terminal

From the repository root, run:

```bash
source scripts/activate_ubuntu.sh
```

This sources ROS2 Humble, activates `.venv`, selects TurtleBot3 Waffle Pi, and
sources the built ROS2 workspace when one exists.

Build after pulling new ROS2 code:

```bash
cd ros2_ws
colcon build --symlink-install
cd ..
source scripts/activate_ubuntu.sh
```

## Adding a dependency

- Python library: add it to `requirements.txt`.
- ROS2/Ubuntu library used by a package: add its rosdep key to that package's
  `package.xml`.
- Never commit an installed package folder, `.venv`, or an API key.

Make dependency changes on a feature branch and open a pull request so another
member can review them before they reach `main`.
