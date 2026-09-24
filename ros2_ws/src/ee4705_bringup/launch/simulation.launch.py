"""Launch the complete EE4705 Gazebo and Nav2 simulation."""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description() -> LaunchDescription:
    """Create the complete EE4705 simulation launch description."""

    project_root = EnvironmentVariable("EE4705_ROOT")

    default_world = PathJoinSubstitution(
        [
            project_root,
            "worlds",
            "house_with_objects.world",
        ]
    )

    default_map = PathJoinSubstitution(
        [
            project_root,
            "maps",
            "house_map_final.yaml",
        ]
    )

    world = LaunchConfiguration("world")
    map_file = LaunchConfiguration("map")
    initial_x = LaunchConfiguration("initial_x")
    initial_y = LaunchConfiguration("initial_y")
    initial_yaw = LaunchConfiguration("initial_yaw")

    gazebo_launch = os.path.join(
        get_package_share_directory("gazebo_ros"),
        "launch",
        "gazebo.launch.py",
    )

    state_publisher_launch = os.path.join(
        get_package_share_directory("turtlebot3_gazebo"),
        "launch",
        "robot_state_publisher.launch.py",
    )

    navigation_launch = os.path.join(
        get_package_share_directory("turtlebot3_navigation2"),
        "launch",
        "navigation2.launch.py",
    )

    navigation_params = os.path.join(
    get_package_share_directory("turtlebot3_navigation2"),
    "param",
    "humble",
    "waffle_pi.yaml",
)

    # Gazebo's diff-drive plugin reports near-perfect odometry, so AMCL can
    # trust it far more than the TurtleBot3 defaults (alpha 0.2). With the
    # defaults the estimate drifted ~0.27 m / 10 deg crossing the open room and
    # the robot missed the 0.85 m doorway north of the old mailbox position.
    amcl_overrides = {
        "alpha1": "0.05",
        "alpha2": "0.05",
        "alpha3": "0.05",
        "alpha4": "0.05",
        "update_min_d": "0.1",
        "update_min_a": "0.1",
        "max_beams": "120",
    }

    tuned_navigation_params = RewrittenYaml(
        source_file=navigation_params,
        param_rewrites=amcl_overrides,
        convert_types=True,
    )

    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch),
        launch_arguments={
            "world": world,
        }.items(),
    )

    start_robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(state_publisher_launch),
    )

    start_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launch),
        launch_arguments={
            "map": map_file,
            "params_file": tuned_navigation_params,
            "use_sim_time": "True",
            "autostart": "True",
        }.items(),
    )

    set_initial_pose = Node(
        package="ee4705_bringup",
        executable="initial_pose_setter",
        name="ee4705_initial_pose_setter",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "x": initial_x,
                "y": initial_y,
                "yaw": initial_yaw,
                "frame_id": "map",
            }
        ],
    )

    run_system_check = Node(
        package="ee4705_bringup",
        executable="system_check",
        name="ee4705_system_check",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "timeout_sec": 30.0,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value=default_world,
                description="Absolute path to the Gazebo world file.",
            ),
            DeclareLaunchArgument(
                "map",
                default_value=default_map,
                description="Absolute path to the saved Nav2 map YAML file.",
            ),
            DeclareLaunchArgument(
                "initial_x",
                default_value="0.0",
                description="Initial robot x coordinate in the map frame.",
            ),
            DeclareLaunchArgument(
                "initial_y",
                default_value="-0.05",
                description="Initial robot y coordinate in the map frame.",
            ),
            DeclareLaunchArgument(
                "initial_yaw",
                default_value="0.0",
                description="Initial robot heading in radians.",
            ),
            start_gazebo,
            start_robot_state_publisher,
            start_navigation,
            TimerAction(
                period=5.0,
                actions=[set_initial_pose],
            ),
            TimerAction(
                period=5.0,
                actions=[run_system_check],
            ),
        ]
    )