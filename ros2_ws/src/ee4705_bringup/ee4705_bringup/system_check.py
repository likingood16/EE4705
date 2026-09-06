"""Verify that the simulated TurtleBot3 is publishing its essential sensors."""

from __future__ import annotations

import time
from collections.abc import Callable
from typing import Any

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan


class SystemCheck(Node):
    """Wait for camera, laser scan, and odometry messages and report the result."""

    def __init__(self) -> None:
        super().__init__("ee4705_system_check")

        self.declare_parameter("timeout_sec", 20.0)
        self.declare_parameter("camera_topic", "/camera/image_raw")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("odom_topic", "/odom")

        self.timeout_sec = float(self.get_parameter("timeout_sec").value)
        topics = {
            "camera": str(self.get_parameter("camera_topic").value),
            "laser": str(self.get_parameter("scan_topic").value),
            "odometry": str(self.get_parameter("odom_topic").value),
        }
        self.received = {name: False for name in topics}

        self._subscriptions = [
            self.create_subscription(
                Image,
                topics["camera"],
                self._mark_received("camera", topics["camera"]),
                qos_profile_sensor_data,
            ),
            self.create_subscription(
                LaserScan,
                topics["laser"],
                self._mark_received("laser", topics["laser"]),
                qos_profile_sensor_data,
            ),
            self.create_subscription(
                Odometry,
                topics["odometry"],
                self._mark_received("odometry", topics["odometry"]),
                qos_profile_sensor_data,
            ),
        ]

        topic_list = ", ".join(topics.values())
        self.get_logger().info(
            f"Waiting up to {self.timeout_sec:.1f} seconds for: {topic_list}"
        )

    def _mark_received(self, name: str, topic: str) -> Callable[[Any], None]:
        """Return a callback that records the first message received for a topic."""

        def callback(_message: Any) -> None:
            if not self.received[name]:
                self.received[name] = True
                self.get_logger().info(f"PASS: received {name} data on {topic}")

        return callback

    def wait_for_topics(self) -> bool:
        """Spin until every required topic responds or the timeout expires."""

        deadline = time.monotonic() + self.timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            if all(self.received.values()):
                self.get_logger().info(
                    "SYSTEM CHECK PASSED: camera, laser, and odometry are available."
                )
                return True
            rclpy.spin_once(self, timeout_sec=0.2)

        missing = [name for name, received in self.received.items() if not received]
        self.get_logger().error(
            "SYSTEM CHECK FAILED. Missing data from: " + ", ".join(missing)
        )
        self.get_logger().error(
            "Keep Gazebo running, use TurtleBot3 waffle_pi, and check `ros2 topic list`."
        )
        return False


def main(args: list[str] | None = None) -> int:
    """Run the system check and return a shell-friendly status code."""

    rclpy.init(args=args)
    node = SystemCheck()
    exit_code = 1

    try:
        exit_code = 0 if node.wait_for_topics() else 1
    except KeyboardInterrupt:
        node.get_logger().warning("System check cancelled by the user.")
        exit_code = 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
