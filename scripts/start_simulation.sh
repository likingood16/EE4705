#!/usr/bin/env bash
# EE4705 Project 1.2 | Task 1 - simulation bringup
# Contributors (from git history): Alexander Likin (2 commits)

# Start the complete EE4705 Gazebo, Nav2 and localization system.

set -eo pipefail

SCRIPT_DIR="$(
  cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
  pwd
)"

PROJECT_ROOT="$(
  cd -- "$SCRIPT_DIR/.."
  pwd
)"

source "$PROJECT_ROOT/scripts/activate_ubuntu.sh"

WORKSPACE_SETUP="$PROJECT_ROOT/ros2_ws/install/setup.bash"

if [[ ! -f "$WORKSPACE_SETUP" ]]; then
  echo "ROS workspace is not built. Building it now..."

  cd "$PROJECT_ROOT/ros2_ws"

  python3 /usr/bin/colcon build \
    --symlink-install

  cd "$PROJECT_ROOT"
fi

# shellcheck disable=SC1090
source "$WORKSPACE_SETUP"

echo "Starting the complete EE4705 simulation..."
echo "World: $PROJECT_ROOT/worlds/house_with_objects.world"
echo "Map:   $PROJECT_ROOT/maps/house_map_final.yaml"
echo "Pose:  x=0.0, y=-0.05, yaw=0.0"
echo
echo "Press Ctrl+C to stop the complete system."

exec ros2 launch \
  ee4705_bringup \
  simulation.launch.py \
  "$@"