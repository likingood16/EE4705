#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VERSIONS_FILE="$PROJECT_ROOT/environment/versions.env"

# shellcheck disable=SC1090
source "$VERSIONS_FILE"
ROS_SETUP="/opt/ros/$ROS_DISTRO/setup.bash"

if [[ ! -f /etc/os-release ]]; then
  echo "Cannot identify this operating system. Use Ubuntu 22.04."
  exit 1
fi

# shellcheck disable=SC1091
source /etc/os-release
if [[ "${ID:-}" != "ubuntu" || "${VERSION_ID:-}" != "$UBUNTU_VERSION" ]]; then
  echo "This project expects Ubuntu $UBUNTU_VERSION; detected ${PRETTY_NAME:-unknown}."
  exit 1
fi

if [[ ! -f "$ROS_SETUP" ]]; then
  echo "ROS2 Humble is not installed at $ROS_SETUP."
  echo "Install ROS2 Humble from: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html"
  exit 1
fi

# shellcheck disable=SC1090
source "$ROS_SETUP"

sudo apt-get update
sudo apt-get install -y \
  python3-pip \
  python3-venv \
  python3-colcon-common-extensions \
  python3-rosdep \
  "ros-$ROS_DISTRO-cv-bridge" \
  "ros-$ROS_DISTRO-gazebo-ros-pkgs" \
  "ros-$ROS_DISTRO-nav2-bringup" \
  "ros-$ROS_DISTRO-nav2-simple-commander" \
  "ros-$ROS_DISTRO-rqt-image-view" \
  "ros-$ROS_DISTRO-turtlebot3-cartographer" \
  "ros-$ROS_DISTRO-turtlebot3-gazebo" \
  "ros-$ROS_DISTRO-turtlebot3-navigation2" \
  "ros-$ROS_DISTRO-turtlebot3-teleop"

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi
rosdep update

python3 -m venv --system-site-packages "$PROJECT_ROOT/.venv"
# shellcheck disable=SC1091
source "$PROJECT_ROOT/.venv/bin/activate"
python -m pip install --upgrade pip
python -m pip install -r "$PROJECT_ROOT/requirements.txt"

rosdep install \
  --from-paths "$PROJECT_ROOT/ros2_ws/src" \
  --ignore-src \
  --rosdistro "$ROS_DISTRO" \
  -r -y

echo
echo "Environment installed successfully."
echo "For every new terminal, run: source scripts/activate_ubuntu.sh"
