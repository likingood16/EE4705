# EE4705 Project 1.2 | Task 1 - simulation bringup
# Contributors (from git history): Alexander Likin (7 commits)
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

    # TurtleBot3's Nav2 parameters with the project's changes (fixed
    # map->odom, 2 x 2 m local costmap, static-map-only global costmap); each
    # change is explained at the top of the file.
    navigation_params = PathJoinSubstitution(
        [
            project_root,
            "config",
            "nav2_waffle_pi.yaml",
        ]
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
            "params_file": navigation_params,
            "use_sim_time": "True",
            "autostart": "True",
        }.items(),
    )

    # Exact localisation: odom == Gazebo world and map == world + (2.0, 0.45),
    # so map->odom is fixed (AMCL no longer broadcasts it; see the Nav2 YAML).
    map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="ee4705_map_to_odom",
        arguments=[
            "--x", "2.0",
            "--y", "0.45",
            "--z", "0.0",
            "--yaw", "0.0",
            "--frame-id", "map",
            "--child-frame-id", "odom",
        ],
    )

    # Feeds the local costmap scans whose transform is already in TF (see
    # scan_relay.py and change 5 in the Nav2 YAML).
    scan_relay = Node(
        package="ee4705_bringup",
        executable="scan_relay",
        name="ee4705_scan_relay",
        output="screen",
        parameters=[{"use_sim_time": True}],
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
            map_to_odom,
            scan_relay,
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