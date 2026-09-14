"""Launch the EE4705 TurtleBot3 sensor system check."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Create the launch description for the sensor check node."""

    timeout = LaunchConfiguration("timeout_sec")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "timeout_sec",
                default_value="20.0",
                description="Seconds to wait for all required TurtleBot3 topics.",
            ),
            Node(
                package="ee4705_bringup",
                executable="system_check",
                name="ee4705_system_check",
                output="screen",
                parameters=[{"timeout_sec": timeout}],
            ),
        ]
    )
