"""ROS controller for Task 4 language-directed object approach."""

from __future__ import annotations

import math
import threading
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from ee4705_perception.approach_policy import ApproachAction
from ee4705_perception.approach_session import evaluate_grounding
from ee4705_perception.approach_velocity import velocity_for_action
from ee4705_perception.camera_snapshot import capture_one_frame
from ee4705_perception.object_grounder import ObjectGrounder
from ee4705_perception.vlm_client import GeminiVLMClient


# ============================================================
# VLM GROUNDING WITH TIMEOUT
# ============================================================

def locate_with_timeout(
    image_path,
    target,
    timeout_s=15.0,
):
    """
    Run one visual-grounding request with a timeout.

    If Gemini takes too long, the controller can retry
    instead of hanging indefinitely.
    """

    result_holder = {}
    error_holder = {}

    def worker():

        try:

            grounder = ObjectGrounder(
                GeminiVLMClient()
            )

            result_holder["result"] = grounder.locate(
                image_path,
                target,
            )

        except Exception as error:

            error_holder["error"] = error

    thread = threading.Thread(
        target=worker,
        daemon=True,
    )

    thread.start()

    thread.join(
        timeout=timeout_s
    )

    if thread.is_alive():

        raise TimeoutError(
            f"Visual grounding exceeded "
            f"{timeout_s:.0f} seconds."
        )

    if "error" in error_holder:

        raise error_holder["error"]

    if "result" not in result_holder:

        raise RuntimeError(
            "Visual grounding returned no result."
        )

    return result_holder["result"]


# ============================================================
# APPROACH CONTROLLER
# ============================================================

class ApproachController(Node):
    """Drive TurtleBot3 toward a language-specified object."""

    def __init__(
        self,
        image_path: str | Path,
    ):

        # Unique name helps avoid repeated rosout-name warnings.
        node_suffix = int(
            time.time() * 1000
        ) % 1_000_000

        super().__init__(
            f"approach_controller_{node_suffix}"
        )

        self.image_path = Path(
            image_path
        )

        # Gazebo TurtleBot3 camera resolution.
        self.image_width = 640
        self.image_height = 480

        # Latest LiDAR information.
        self.front_distance_m = None
        self.last_scan_time = None

        # ----------------------------------------------------
        # Controller settings
        # ----------------------------------------------------

        # Scan older than this is considered unavailable.
        self.scan_timeout_s = 3.0

        # Forward movement duration.
        self.forward_pulse_s = 2.0

        # Shorter pulse for fine left/right alignment.
        self.turn_pulse_s = 0.7

        # Longer rotation when searching for an unseen object.
        self.search_pulse_s = 2.5

        # Maximum duration of one approach request.
        self.max_runtime_s = 300.0

        # Maximum time allowed for one Gemini grounding call.
        self.vision_timeout_s = 15.0
        # ----------------------------------------------------
        # ROS velocity publisher
        # ----------------------------------------------------

        self.velocity_publisher = self.create_publisher(
            Twist,
            "/cmd_vel",
            10,
        )

        # ----------------------------------------------------
        # ROS LiDAR subscriber
        # ----------------------------------------------------

        self.scan_subscription = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            qos_profile_sensor_data,
        )

    # ========================================================
    # LIDAR
    # ========================================================

    def scan_callback(
        self,
        msg: LaserScan,
    ):
        """
        Find the closest valid obstacle approximately
        +/- 15 degrees in front of the robot.
        """

        front_ranges = []

        front_half_angle = math.radians(
            15.0
        )

        for index, distance in enumerate(
            msg.ranges
        ):

            angle = (
                msg.angle_min
                + index * msg.angle_increment
            )

            # Wrap angle into [-pi, pi].
            angle = math.atan2(
                math.sin(angle),
                math.cos(angle),
            )

            if abs(angle) <= front_half_angle:

                if (
                    math.isfinite(distance)
                    and distance >= msg.range_min
                    and distance <= msg.range_max
                ):

                    front_ranges.append(
                        distance
                    )

        if front_ranges:

            self.front_distance_m = min(
                front_ranges
            )

        else:

            # Scan exists, but there is no finite return
            # directly ahead. Treat this as clear space.
            self.front_distance_m = float(
                msg.range_max
            )

        self.last_scan_time = (
            time.monotonic()
        )

    def get_front_distance(
        self,
    ):
        """Return None only when scan data is missing/stale."""

        if self.last_scan_time is None:

            return None

        age = (
            time.monotonic()
            - self.last_scan_time
        )

        if age > self.scan_timeout_s:

            return None

        return self.front_distance_m

    # ========================================================
    # VELOCITY CONTROL
    # ========================================================

    def publish_velocity(
        self,
        linear_x: float,
        angular_z: float,
    ):
        """Publish one Twist command."""

        msg = Twist()

        msg.linear.x = float(
            linear_x
        )

        msg.angular.z = float(
            angular_z
        )

        self.velocity_publisher.publish(
            msg
        )

    def stop_robot(
        self,
    ):
        """Command zero velocity."""

        self.publish_velocity(
            0.0,
            0.0,
        )

    def execute_velocity_pulse(
        self,
        linear_x: float,
        angular_z: float,
        duration_s: float,
    ):
        """
        Continuously publish velocity during the pulse.

        Repeated publishing is required because a single
        /cmd_vel message may expire before useful movement.
        """

        start = time.monotonic()

        while (
            rclpy.ok()
            and time.monotonic() - start
            < duration_s
        ):

            self.publish_velocity(
                linear_x,
                angular_z,
            )

            rclpy.spin_once(
                self,
                timeout_sec=0.1,
            )

        # Stop before taking the next visual observation.
        self.stop_robot()

        # Short settling period.
        settle_start = time.monotonic()

        while (
            rclpy.ok()
            and time.monotonic() - settle_start
            < 0.05
        ):

            rclpy.spin_once(
                self,
                timeout_sec=0.02,
            )

    # ========================================================
    # APPROACH LOOP
    # ========================================================

    def approach(
        self,
        target: str,
    ) -> tuple[bool, str]:
        """Search for and approach one object."""

        target = target.strip()

        if not target:

            return (
                False,
                "No target object was specified.",
            )

        print(
            f"[Approach] Target: {target}"
        )

        # ----------------------------------------------------
        # Wait for initial LiDAR
        # ----------------------------------------------------

        scan_wait_start = (
            time.monotonic()
        )

        while (
            rclpy.ok()
            and self.get_front_distance()
            is None
            and time.monotonic()
            - scan_wait_start
            < 10.0
        ):

            rclpy.spin_once(
                self,
                timeout_sec=0.1,
            )

        if (
            self.get_front_distance()
            is None
        ):

            self.stop_robot()

            return (
                False,
                "I cannot approach the object "
                "because front LiDAR data "
                "is unavailable.",
            )

        started_at = (
            time.monotonic()
        )

        try:

            while rclpy.ok():

                # --------------------------------------------
                # Overall timeout
                # --------------------------------------------

                if (
                    time.monotonic()
                    - started_at
                    >= self.max_runtime_s
                ):

                    self.stop_robot()

                    return (
                        False,
                        f"I could not reach the "
                        f"{target} before the "
                        "approach timed out.",
                    )

                # Process any pending ROS messages.
                rclpy.spin_once(
                    self,
                    timeout_sec=0.1,
                )

                # --------------------------------------------
                # Capture fresh camera frame
                # --------------------------------------------

                print(
                    "[Approach] "
                    "Capturing camera frame..."
                )

                saved_image = capture_one_frame(
                    self.image_path,
                    timeout_s=10.0,
                )

                # --------------------------------------------
                # Visual grounding
                # --------------------------------------------

                print(
                    f"[Approach] "
                    f"Looking for {target}..."
                )

                try:

                    grounding = (
                        locate_with_timeout(
                            saved_image,
                            target,
                            timeout_s=(
                                self.vision_timeout_s
                            ),
                        )
                    )

                except TimeoutError as error:

                    print(
                        "[Approach] "
                        f"Vision timeout: {error}"
                    )

                    print(
                        "[Approach] "
                        "Retrying with a new "
                        "camera frame..."
                    )

                    continue

                except ValueError as error:

                    print(
                        "[Approach] Invalid "
                        f"grounding result: {error}"
                    )

                    print(
                        "[Approach] "
                        "Retrying with a new "
                        "camera frame..."
                    )

                    continue

                except Exception as error:

                    print(
                        "[Approach] Vision "
                        f"request failed: {error}"
                    )

                    print(
                        "[Approach] Retrying..."
                    )

                    continue

                # --------------------------------------------
                # Refresh LiDAR after Gemini request
                # --------------------------------------------

                refresh_start = (
                    time.monotonic()
                )

                while (
                    rclpy.ok()
                    and time.monotonic()
                    - refresh_start
                    < 0.5
                ):

                    rclpy.spin_once(
                        self,
                        timeout_sec=0.05,
                    )

                front_distance = (
                    self.get_front_distance()
                )

                # --------------------------------------------
                # Decide next action
                # --------------------------------------------

                decision = evaluate_grounding(
                    grounding,
                    image_width=(
                        self.image_width
                    ),
                    image_height=(
                        self.image_height
                    ),
                    front_distance_m=(
                        front_distance
                    ),
                )

                print(
                    "[Approach] "
                    f"found={grounding.found}, "
                    f"action="
                    f"{decision.action.value}, "
                    f"front={front_distance}"
                )

                # --------------------------------------------
                # TARGET REACHED
                # --------------------------------------------

                if (
                    decision.action
                    == ApproachAction
                    .STOP_TARGET_REACHED
                ):

                    self.stop_robot()

                    return (
                        True,
                        f"I have reached the "
                        f"{target}.",
                    )

                # --------------------------------------------
                # SENSOR FAILURE
                # --------------------------------------------

                if (
                    decision.action
                    == ApproachAction
                    .STOP_SENSOR_UNAVAILABLE
                ):

                    self.stop_robot()

                    return (
                        False,
                        "I stopped because the "
                        "LiDAR reading became "
                        "unavailable.",
                    )

                # --------------------------------------------
                # OBSTACLE SAFETY STOP
                # --------------------------------------------

                if (
                    decision.action
                    == ApproachAction
                    .STOP_OBSTACLE
                ):

                    self.stop_robot()

                    return (
                        False,
                        f"I stopped while "
                        f"approaching the {target} "
                        "because it is not safe "
                        "to continue forward.",
                    )

                # --------------------------------------------
                # Convert policy action into velocity
                # --------------------------------------------

                velocity = velocity_for_action(
                    decision.action,
                    motion_permitted=True,
                )

                print(
                    "[Approach] "
                    f"linear="
                    f"{velocity.linear_x:.2f}, "
                    f"angular="
                    f"{velocity.angular_z:.2f}"
                )

                # --------------------------------------------
                # SEARCH
                # --------------------------------------------

                if (
                    decision.action
                    == ApproachAction.SEARCH
                ):

                    self.execute_velocity_pulse(
                        velocity.linear_x,
                        velocity.angular_z,
                        self.search_pulse_s,
                    )

                # --------------------------------------------
                # ALIGN / MOVE FORWARD
                # --------------------------------------------

                else:
                    if decision.action in (
                       ApproachAction.TURN_LEFT,
                       ApproachAction.TURN_RIGHT,
                    ):
                      pulse_s = self.turn_pulse_s
                    else:
                     pulse_s = self.forward_pulse_s

                    self.execute_velocity_pulse(
                      velocity.linear_x,
                      velocity.angular_z,
                      pulse_s,
                    )
                    

        finally:

            # Safety guarantee:
            # every exit path commands zero velocity.
            self.stop_robot()


# ============================================================
# TERMINAL CHAT WRAPPER
# ============================================================

def approach_object(
    target: str,
    image_path: str | Path,
) -> tuple[bool, str]:
    """Entry point used by terminal_chat.py."""

    node = ApproachController(
        image_path
    )

    try:

        return node.approach(
            target
        )

    finally:

        node.stop_robot()
        node.destroy_node()
