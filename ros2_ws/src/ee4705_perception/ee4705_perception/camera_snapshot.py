# EE4705 Project 1.2 | Task 3 - scene description
# Contributors (from git history): Alexander Likin (2 commits)
"""Capture one image from the TurtleBot3 camera."""

from __future__ import annotations
import argparse
import os
import time
from pathlib import Path

import cv2
import rclpy

from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class CameraSnapshot(Node):
    """Subscribe to the camera and save the first frame."""

    def __init__(self, output_path: Path) -> None:
        super().__init__("camera_snapshot")

        self.output_path = output_path
        self.bridge = CvBridge()
        self.saved = False

        self.subscription = self.create_subscription(
            Image,
            "/camera/image_raw",
            self._on_image,
            10,
        )

        self.get_logger().info(
            "Waiting for one image from /camera/image_raw..."
        )


    def _on_image(self, message: Image) -> None:
        """Convert one ROS image message into a JPEG."""

        if self.saved:
            return

        frame = self.bridge.imgmsg_to_cv2(
            message,
            desired_encoding="bgr8",
        )

        self.output_path.parent.mkdir(
            parents=True,
            exist_ok=True,
        )

        saved_successfully = cv2.imwrite(
            str(self.output_path),
            frame,
        )

        if not saved_successfully:
            raise RuntimeError(
                f"Could not save image to {self.output_path}"
            )

        self.saved = True

        self.get_logger().info(
            f"Saved camera image to {self.output_path}"
        )


def capture_one_frame(
    output_path: str | Path,
    timeout_s: float = 10.0,
) -> Path:
    """Capture one frame using an existing ROS context."""

    if not rclpy.ok():
        raise RuntimeError(
            "ROS must be initialized before capturing an image."
        )

    path = Path(output_path)
    node = CameraSnapshot(path)
    deadline = time.monotonic() + timeout_s

    try:
        while (
            rclpy.ok()
            and not node.saved
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(
                node,
                timeout_sec=1.0,
            )

        if not node.saved:
            raise TimeoutError(
                "No camera frame received "
                f"within {timeout_s:.1f} seconds."
            )

        return path

    finally:
        node.destroy_node()


def main() -> int:
    """Capture one frame using a selectable output filename."""

    parser = argparse.ArgumentParser(
        description=(
            "Capture one image from "
            "/camera/image_raw."
        )
    )

    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help=(
            "Output image path. Relative paths are "
            "resolved from the EE4705 project root."
        ),
    )

    parser.add_argument(
        "--timeout",
        type=float,
        default=10.0,
        help="Maximum seconds to wait for a camera frame.",
    )

    arguments = parser.parse_args()

    project_root = Path(
        os.environ.get(
            "EE4705_ROOT",
            Path(__file__).resolve().parents[4],
        )
    )

    if arguments.output is None:
        output_path = (
            project_root
            / "evaluation"
            / "scenes"
            / "current_camera.jpg"
        )

    elif arguments.output.is_absolute():
        output_path = arguments.output

    else:
        output_path = (
            project_root
            / arguments.output
        )

    rclpy.init()

    try:
        saved_path = capture_one_frame(
            output_path,
            timeout_s=arguments.timeout,
        )

        print(
            f"Camera snapshot saved: {saved_path}"
        )

        return 0

    except Exception as error:
        print(
            f"Camera capture failed: {error}"
        )

        return 1

    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    raise SystemExit(main())