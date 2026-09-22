"""Set the simulated robot's initial AMCL pose automatically."""

from __future__ import annotations

import math

import rclpy
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.srv import SetInitialPose
from rclpy.node import Node


class InitialPoseSetter(Node):
    """Wait for AMCL to become active, then provide its initial pose."""

    def __init__(self) -> None:
        super().__init__("ee4705_initial_pose_setter")

        self.declare_parameter("x", 0.0)
        self.declare_parameter("y", 0.0)
        self.declare_parameter("yaw", 0.0)
        self.declare_parameter("frame_id", "map")

        self.amcl_state_client = self.create_client(
            GetState,
            "/amcl/get_state",
        )
        self.initial_pose_client = self.create_client(
            SetInitialPose,
            "/set_initial_pose",
        )

    def wait_for_amcl(self) -> None:
        """Wait until the AMCL lifecycle node reports an active state."""

        self.get_logger().info(
            "Waiting for the AMCL lifecycle service..."
        )

        while rclpy.ok() and not self.amcl_state_client.wait_for_service(
            timeout_sec=1.0
        ):
            self.get_logger().info(
                "AMCL lifecycle service is not available yet."
            )

        while rclpy.ok():
            request = GetState.Request()
            future = self.amcl_state_client.call_async(request)

            rclpy.spin_until_future_complete(
                self,
                future,
                timeout_sec=5.0,
            )

            response = future.result()

            if (
                response is not None
                and response.current_state.id
                == State.PRIMARY_STATE_ACTIVE
            ):
                self.get_logger().info("AMCL is active.")
                return

            self.get_logger().info(
                "Waiting for AMCL to become active..."
            )

    def set_initial_pose(self) -> None:
        """Send the configured initial robot pose to AMCL."""

        self.get_logger().info(
            "Waiting for the initial-pose service..."
        )

        while rclpy.ok() and not self.initial_pose_client.wait_for_service(
            timeout_sec=1.0
        ):
            self.get_logger().info(
                "Initial-pose service is not available yet."
            )

        x = float(self.get_parameter("x").value)
        y = float(self.get_parameter("y").value)
        yaw = float(self.get_parameter("yaw").value)
        frame_id = str(self.get_parameter("frame_id").value)

        request = SetInitialPose.Request()
        request.pose.header.stamp = self.get_clock().now().to_msg()
        request.pose.header.frame_id = frame_id

        request.pose.pose.pose.position.x = x
        request.pose.pose.pose.position.y = y
        request.pose.pose.pose.position.z = 0.0

        request.pose.pose.pose.orientation.z = math.sin(yaw / 2.0)
        request.pose.pose.pose.orientation.w = math.cos(yaw / 2.0)

        # Position uncertainty in x and y.
        request.pose.pose.covariance[0] = 0.25
        request.pose.pose.covariance[7] = 0.25

        # Orientation uncertainty around the vertical axis.
        request.pose.pose.covariance[35] = 0.0685

        self.get_logger().info(
            f"Setting initial pose: x={x:.3f}, "
            f"y={y:.3f}, yaw={yaw:.3f}"
        )

        future = self.initial_pose_client.call_async(request)

        rclpy.spin_until_future_complete(
            self,
            future,
            timeout_sec=10.0,
        )

        if future.result() is None:
            raise RuntimeError(
                "The initial-pose service did not return a response."
            )

        self.get_logger().info(
            "Initial AMCL pose was set successfully."
        )


def main(args: list[str] | None = None) -> None:
    """Run the automatic initial-pose node."""

    rclpy.init(args=args)
    node = InitialPoseSetter()

    try:
        node.wait_for_amcl()
        node.set_initial_pose()
    except KeyboardInterrupt:
        node.get_logger().info("Initial-pose setup interrupted.")
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()