#!/usr/bin/env bash

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "Run this script with: source scripts/activate_ubuntu.sh"
  exit 1
fi

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_SETUP="/opt/ros/humble/setup.bash"
VENV_ACTIVATE="$PROJECT_ROOT/.venv/bin/activate"

if [[ ! -f "$ROS_SETUP" ]]; then
  echo "ROS2 Humble was not found at $ROS_SETUP."
  return 1
fi

if [[ ! -f "$VENV_ACTIVATE" ]]; then
  echo "The local .venv does not exist. Run: bash scripts/install_dependencies_ubuntu.sh"
  return 1
fi

# shellcheck disable=SC1090
source "$ROS_SETUP"
# shellcheck disable=SC1090
source "$VENV_ACTIVATE"

export TURTLEBOT3_MODEL=waffle_pi

if [[ -f "$PROJECT_ROOT/ros2_ws/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "$PROJECT_ROOT/ros2_ws/install/setup.bash"
fi

cd "$PROJECT_ROOT" || return 1
echo "EE4705 environment active: ROS=$ROS_DISTRO, robot=$TURTLEBOT3_MODEL, Python=$(python --version 2>&1)"

