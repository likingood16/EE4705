#!/usr/bin/env bash
# EE4705 Project 1.2 | Task 1 - simulation bringup
# Contributors (from git history): Alexander Likin (5 commits)

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "Run this script with: source scripts/activate_ubuntu.sh"
  exit 1
fi

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VERSIONS_FILE="$PROJECT_ROOT/environment/versions.env"

# shellcheck disable=SC1090
source "$VERSIONS_FILE"
ROS_SETUP="/opt/ros/$ROS_DISTRO/setup.bash"
VENV_ACTIVATE="$PROJECT_ROOT/.venv/bin/activate"

if [[ ! -f "$ROS_SETUP" ]]; then
  echo "ROS2 Humble was not found at $ROS_SETUP."
  return 1
fi

if [[ ! -f "$VENV_ACTIVATE" ]]; then
  echo "The local .venv does not exist. Run: bash scripts/install_dependencies_ubuntu.sh"
  return 1
fi

# Fast DDS (the Humble default) stalled /tf delivery to Nav2 after a few
# minutes of simulation, freezing the costmap robot pose. CycloneDDS does not.
# Export it before anything ROS starts so every process and the ros2 daemon
# use the same middleware.
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# Loopback-only transport with large fragments, so camera images are not
# dropped (see config/cyclonedds.xml).
export CYCLONEDDS_URI="file://$PROJECT_ROOT/config/cyclonedds.xml"

# shellcheck disable=SC1090
source "$ROS_SETUP"
# shellcheck disable=SC1090
source "$VENV_ACTIVATE"

export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-waffle_pi}"
export EE4705_ROOT="$PROJECT_ROOT"
ENV_FILE="$PROJECT_ROOT/.env"

if [[ -f "$ENV_FILE" ]]; then
  set -a
  source "$ENV_FILE"
  set +a
fi

if [[ -f "$PROJECT_ROOT/ros2_ws/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "$PROJECT_ROOT/ros2_ws/install/setup.bash"
fi

cd "$PROJECT_ROOT" || return 1
echo "EE4705 environment active: ROS=$ROS_DISTRO, RMW=$RMW_IMPLEMENTATION, robot=$TURTLEBOT3_MODEL, Python=$(python --version 2>&1)"
