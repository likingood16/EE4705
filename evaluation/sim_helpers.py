"""Shared helpers for simulation experiments.

Ground truth comes from Gazebo itself (`gz model -p`), which is independent of
ROS, TF and Nav2, so it can be used to check localisation and to measure the
final robot-to-object distance.

One long-lived `SimProbe` node is used per experiment instead of many
short-lived `ros2` CLI processes.
"""

from __future__ import annotations

import math
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path

import yaml

PROJECT_ROOT = Path(__file__).resolve().parents[1]
WAYPOINTS = PROJECT_ROOT / "config" / "room_waypoints.yaml"

# Map frame = Gazebo world frame shifted by this offset (see simulation.launch.py).
WORLD_TO_MAP = (2.0, 0.45)


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float

    def as_dict(self) -> dict:
        return {"x": round(self.x, 3), "y": round(self.y, 3), "yaw": round(self.yaw, 3)}


def gz_model_pose(model: str, timeout_s: float = 10.0) -> Pose2D:
    """Return a Gazebo model's world pose (x, y, yaw) via Gazebo transport."""

    for attempt in range(3):
        try:
            output = subprocess.run(
                ["gz", "model", "-m", model, "-p"],
                capture_output=True, text=True, timeout=timeout_s, check=True,
            ).stdout.split()
            break
        except subprocess.TimeoutExpired:
            # gz can be slow to answer while Gazebo is heavily loaded.
            if attempt == 2:
                raise
    if len(output) != 6:
        raise RuntimeError(f"Unexpected gz output for {model!r}: {output}")
    x, y, _z, _roll, _pitch, yaw = (float(value) for value in output)
    return Pose2D(x, y, yaw)


def world_to_map(pose: Pose2D) -> Pose2D:
    return Pose2D(pose.x + WORLD_TO_MAP[0], pose.y + WORLD_TO_MAP[1], pose.yaw)


def map_to_world(pose: Pose2D) -> Pose2D:
    return Pose2D(pose.x - WORLD_TO_MAP[0], pose.y - WORLD_TO_MAP[1], pose.yaw)


def load_rooms() -> dict[str, Pose2D]:
    data = yaml.safe_load(WAYPOINTS.read_text())
    return {name: Pose2D(float(v["x"]), float(v["y"]), float(v["yaw"]))
            for name, v in data.items()}


def angle_diff(a: float, b: float) -> float:
    return math.atan2(math.sin(a - b), math.cos(a - b))


def set_robot_world_pose(pose: Pose2D, model: str = "waffle_pi") -> None:
    """Teleport the robot in Gazebo (used only to set trial start poses).

    Odometry follows automatically because the diff-drive plugin reports the
    true world pose, and map->odom is a fixed transform.
    """

    subprocess.run(
        ["gz", "model", "-m", model,
         "-x", f"{pose.x}", "-y", f"{pose.y}", "-z", "0.0",
         "-R", "0", "-P", "0", "-Y", f"{pose.yaw}"],
        capture_output=True, text=True, timeout=5.0, check=True,
    )


class SimProbe:
    """One ROS node for navigation goals, TF and costmap pose checks."""

    def __init__(self, name: str = "ee4705_sim_probe") -> None:
        import rclpy
        from geometry_msgs.msg import PolygonStamped
        from nav2_msgs.action import NavigateToPose
        from rclpy.action import ActionClient
        from rclpy.parameter import Parameter
        from tf2_ros import Buffer, TransformListener

        if not rclpy.ok():
            rclpy.init()
        self._rclpy = rclpy
        self.node = rclpy.create_node(
            name, parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)
        self.footprints: dict[str, tuple[float, float, float]] = {}
        for costmap in ("global_costmap", "local_costmap"):
            self.node.create_subscription(
                PolygonStamped, f"/{costmap}/published_footprint",
                self._footprint_callback(costmap), 10)
        self._NavigateToPose = NavigateToPose
        self.nav_client = ActionClient(self.node, NavigateToPose, "navigate_to_pose")

    def _footprint_callback(self, costmap: str):
        def callback(message) -> None:
            points = message.polygon.points
            if not points:
                return
            cx = sum(p.x for p in points) / len(points)
            cy = sum(p.y for p in points) / len(points)
            stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
            self.footprints[costmap] = (cx, cy, stamp)
        return callback

    def spin(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self._rclpy.spin_once(self.node, timeout_sec=min(0.05, max(0.0, end - time.monotonic())))

    def sim_now(self) -> float:
        return self.node.get_clock().now().nanoseconds * 1e-9

    def tf_pose(self, target: str = "map", source: str = "base_footprint") -> Pose2D | None:
        from rclpy.time import Time
        try:
            t = self.tf_buffer.lookup_transform(target, source, Time())
        except Exception:
            return None
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return Pose2D(t.transform.translation.x, t.transform.translation.y, yaw)

    def localisation_report(self) -> dict:
        """Compare the costmap/TF robot pose with Gazebo ground truth (map frame)."""

        truth = world_to_map(gz_model_pose("waffle_pi"))
        now = self.sim_now()
        report = {"truth_map": truth.as_dict(), "sim_time": round(now, 2)}
        glob = self.footprints.get("global_costmap")
        if glob is not None:
            report["global_footprint_err_m"] = round(math.hypot(glob[0] - truth.x, glob[1] - truth.y), 3)
            report["global_footprint_age_s"] = round(now - glob[2], 2)
        loc = self.footprints.get("local_costmap")
        if loc is not None:
            # Local costmap works in odom, which equals the Gazebo world frame.
            world = map_to_world(truth)
            report["local_footprint_err_m"] = round(math.hypot(loc[0] - world.x, loc[1] - world.y), 3)
            report["local_footprint_age_s"] = round(now - loc[2], 2)
        tf = self.tf_pose()
        if tf is not None:
            report["tf_err_m"] = round(math.hypot(tf.x - truth.x, tf.y - truth.y), 3)
            report["tf_yaw_err_deg"] = round(math.degrees(angle_diff(tf.yaw, truth.yaw)), 1)
        return report

    def navigate(self, goal: Pose2D, timeout_s: float = 240.0, monitor=None) -> tuple[bool, float, int]:
        """Send one NavigateToPose goal in the map frame; return (ok, seconds, status)."""

        if not self.nav_client.wait_for_server(timeout_sec=20.0):
            return False, 0.0, -1
        msg = self._NavigateToPose.Goal()
        msg.pose.header.frame_id = "map"
        msg.pose.header.stamp = self.node.get_clock().now().to_msg()
        msg.pose.pose.position.x = goal.x
        msg.pose.pose.position.y = goal.y
        msg.pose.pose.orientation.z = math.sin(goal.yaw / 2)
        msg.pose.pose.orientation.w = math.cos(goal.yaw / 2)
        started = time.monotonic()
        future = self.nav_client.send_goal_async(msg)
        while not future.done():
            self.spin(0.05)
        handle = future.result()
        if not handle.accepted:
            return False, time.monotonic() - started, -2
        result_future = handle.get_result_async()
        next_monitor = time.monotonic()
        while not result_future.done():
            self.spin(0.1)
            if monitor is not None and time.monotonic() >= next_monitor:
                monitor(self)
                next_monitor = time.monotonic() + 2.0
            if time.monotonic() - started > timeout_s:
                cancel = handle.cancel_goal_async()
                while not cancel.done():
                    self.spin(0.05)
                return False, time.monotonic() - started, -3
        status = result_future.result().status
        return status == 4, time.monotonic() - started, status

    def close(self) -> None:
        self.node.destroy_node()
