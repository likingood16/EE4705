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

    # Gazebo's diff-drive plugin publishes odometry from the true world pose
    # (odometry_source defaults to WORLD), so odom == Gazebo world frame. The
    # saved map is the world shifted by (+2.00, +0.45) with no rotation (checked
    # by aligning the map image with the wall geometry). A fixed map->odom
    # transform therefore gives exact localisation; AMCL keeps running but no
    # longer publishes map->odom, because its estimate drifted ~0.27 m / 10 deg
    # and made the robot miss the 0.85 m doorway north of the start room.
    #
    # DWB ends its local plan at the first global-plan pose further than half
    # the local costmap width from the robot. With the default 3 x 3 m window,
    # the U-turn through the doorway north of the start room put that local
    # goal behind the wall; GoalDist/GoalAlign score straight-line distance, so
    # the robot turned to face the plain wall and stalled until the progress
    # checker aborted. A 2 x 2 m window keeps the local goal on the near side
    # of the wall. (forward_prune_distance would be the direct knob, but it is
    # not in the TurtleBot3 YAML and RewrittenYaml cannot add new keys.)
    navigation_overrides = {
        "tf_broadcast": "False",
        "local_costmap.local_costmap.ros__parameters.width": "2",
        "local_costmap.local_costmap.ros__parameters.height": "2",
    }

    tuned_navigation_params = RewrittenYaml(
        source_file=navigation_params,
        param_rewrites=navigation_overrides,
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