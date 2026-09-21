"""Experimental simulation-only Task 4 ROS adapter.

Prepared with AI assistance for Student C; review and Gazebo validation required.
The standalone command is observation-only unless --enable-motion is supplied.
Do not use on physical hardware. This is not a certified emergency-stop system.
"""

from __future__ import annotations

import argparse
import json
import math
import tempfile
import time
from dataclasses import asdict
from pathlib import Path

from .approach_policy import ApproachAction, ApproachConfig
from .approach_result_logger import ApproachTrial, append_approach_trial
from .approach_runtime import (
    ApproachAbort, ApproachOutcome, fresh, scan_clearance,
    stamp_seconds, wait_for_grounding, yaw_from_quaternion,
)
from .approach_session import evaluate_grounding
from .approach_velocity import velocity_for_action
from .object_grounder import ObjectGrounder
from .search_tracker import SearchStatus, SearchTracker


def run_approach(target, client, *, enable_motion=False, exclusive_control=False,
                 evidence_dir="evaluation/task4_evidence", trial_id=None,
                 camera_topic="/camera/image_raw", scan_topic="/scan",
                 odom_topic="/odom", cmd_topic="/cmd_vel",
                 inf_is_clear=False, timeout_s=180.0, vlm_timeout_s=20.0,
                 max_calls=20, close_height=0.55):
    """Run one attempt in an already initialized ROS context.

    Caller must ensure Nav2/teleop have relinquished velocity control. Returns
    a candidate, never an automatically certified approach success. Stop via
    /task4/stop or Ctrl+C. Blocks chat input but spins the stop service.
    """
    # Lazy ROS imports keep the existing macOS offline tests importable.
    import cv2
    import rclpy
    from cv_bridge import CvBridge
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image, LaserScan
    from std_srvs.srv import Trigger

    if not rclpy.ok():
        raise RuntimeError("Caller must initialize ROS before run_approach().")
    if not target.strip():
        raise ValueError("Target cannot be empty.")
    if enable_motion and not exclusive_control:
        raise ValueError("Motion requires explicit exclusive-control confirmation.")
    if not all(math.isfinite(v) and v > 0 for v in (timeout_s, vlm_timeout_s)):
        raise ValueError("Timeouts must be positive and finite.")
    if isinstance(max_calls, bool) or not isinstance(max_calls, int) or max_calls < 1:
        raise ValueError("max_calls must be a positive integer.")
    config = ApproachConfig(close_height_fraction=close_height)
    base = Path(evidence_dir)
    base.mkdir(parents=True, exist_ok=True)
    # Unique directory; never overwrite earlier experiment images.
    folder = Path(tempfile.mkdtemp(prefix="attempt-", dir=base))
    trial_id = trial_id or folder.name
    started = time.monotonic()
    calls = 0
    search = None
    total_search_rotation = 0.0
    initially_visible = None
    active_search = False

    class ApproachNode(Node):
        def __init__(self):
            super().__init__("task4_approach")
            self.bridge = CvBridge()
            self.cancelled = False
            self.fault = None
            self.frame = None
            self.yaw = None
            self.front = None
            self.around = None
            self.pose_xy = None
            self.received = {}
            self.stamps = {}
            self.frame_sequence = 0
            self.search_tracker = None
            self.publisher = self.create_publisher(Twist, cmd_topic, 10)
            self.create_subscription(Image, camera_topic, self.on_image, qos_profile_sensor_data)
            self.create_subscription(LaserScan, scan_topic, self.on_scan, qos_profile_sensor_data)
            self.create_subscription(Odometry, odom_topic, self.on_odom, qos_profile_sensor_data)
            self.create_service(Trigger, "/task4/stop", self.on_stop)

        def accept_stamp(self, name, message):
            stamp = stamp_seconds(message.header.stamp)
            clock = self.get_clock().now().nanoseconds * 1e-9
            if not -0.1 <= clock - stamp <= 1.0:
                return False
            previous = self.stamps.get(name)
            if previous is not None and stamp <= previous:
                return False
            self.stamps[name] = stamp
            self.received[name] = time.monotonic()
            return True

        def on_image(self, message):
            if not self.accept_stamp("camera", message):
                return
            try:
                self.frame = self.bridge.imgmsg_to_cv2(message, "bgr8")
                self.frame_sequence += 1
            except Exception:
                self.fault = "camera_conversion_failed"

        def on_scan(self, message):
            if not self.accept_stamp("scan", message):
                return
            args = (message.ranges, message.angle_min, message.angle_increment,
                    message.range_min, message.range_max)
            self.front = scan_clearance(*args, inf_is_clear=inf_is_clear)
            self.around = scan_clearance(*args, all_around=True, inf_is_clear=inf_is_clear)

        def on_odom(self, message):
            if not self.accept_stamp("odom", message):
                return
            pose = message.pose.pose
            q = pose.orientation
            try:
                self.yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
                self.pose_xy = (pose.position.x, pose.position.y)
                if not all(math.isfinite(v) for v in self.pose_xy):
                    raise ValueError("Invalid position")
                if self.search_tracker is not None:
                    self.search_tracker.update(yaw_rad=self.yaw,
                                               now_s=time.monotonic(), target_found=False)
            except ValueError:
                self.fault = "invalid_odometry"

        def on_stop(self, request, response):
            self.cancelled = True
            self.stop()
            response.success = True
            response.message = "Task 4 cancellation latched; no further motion permitted."
            return response

        def stop(self):
            # Observation mode must not interfere with another controller.
            if enable_motion and rclpy.ok():
                self.publisher.publish(Twist())

        def poll(self):
            if not rclpy.ok():
                raise ApproachAbort("ros_shutdown")
            rclpy.spin_once(self, timeout_sec=0.02)
            if self.cancelled:
                raise ApproachAbort("cancelled")
            if self.fault:
                raise ApproachAbort(self.fault)
            if time.monotonic() - started >= timeout_s:
                raise ApproachAbort("operation_timeout")

        def sensors_ready(self):
            now = time.monotonic()
            return (all(fresh(self.received.get(name), now, 0.75)
                        for name in ("camera", "scan", "odom"))
                    and self.frame is not None and self.yaw is not None
                    and self.front is not None)

        def check_motion(self, turning):
            if not self.sensors_ready():
                raise ApproachAbort("sensor_missing_invalid_or_stale")
            distance = self.around if turning else self.front
            if distance is None:
                raise ApproachAbort("rotation_scan_unavailable")
            if distance <= config.minimum_front_distance_m:
                raise ApproachAbort("obstacle")

        def pulse(self, action):
            request = velocity_for_action(action, motion_permitted=enable_motion)
            # At most ~4 cm advance or ~7 degrees turn per perception cycle.
            deadline = time.monotonic() + 0.5
            if action == ApproachAction.SEARCH:
                deadline = time.monotonic() + 1.75  # ~20 degrees per search view
            try:
                while time.monotonic() < deadline:
                    self.poll()
                    self.check_motion(request.angular_z != 0)
                    if (action == ApproachAction.SEARCH and self.search_tracker is not None
                            and self.search_tracker.status != SearchStatus.SEARCHING):
                        break
                    msg = Twist()
                    msg.linear.x = request.linear_x
                    msg.angular.z = request.angular_z
                    self.publisher.publish(msg)
            finally:
                self.stop()

    node = ApproachNode()
    grounder = ObjectGrounder(client)
    reason = "not_started"
    candidate = False
    start_pose = ""
    try:
        # Explicitly stop before startup. Poll using wall time even if /clock pauses.
        deadline = time.monotonic() + 10.0
        while not node.sensors_ready():
            node.stop()
            node.poll()
            if time.monotonic() >= deadline:
                raise ApproachAbort("startup_sensor_timeout")
        start_pose = json.dumps({"x": node.pose_xy[0], "y": node.pose_xy[1],
                                 "yaw": node.yaw, "frame": "odom"})
        for index in range(max_calls):
            # Stop, settle, then demand a newly received image.
            node.stop()
            settle_until = time.monotonic() + 0.3
            while time.monotonic() < settle_until:
                node.poll()
                node.stop()
            sequence = node.frame_sequence
            deadline = time.monotonic() + 3.0
            while node.frame_sequence <= sequence or not node.sensors_ready():
                node.poll()
                node.stop()
                if time.monotonic() >= deadline:
                    raise ApproachAbort("fresh_frame_timeout")
            if active_search and search is not None:
                search.update(yaw_rad=node.yaw, now_s=time.monotonic(), target_found=False)
                if search.status == SearchStatus.TIMED_OUT:
                    raise ApproachAbort("search_timed_out")
            image_path = folder / f"frame-{index:03d}.jpg"
            frame = node.frame.copy()
            if not cv2.imwrite(str(image_path), frame):
                raise ApproachAbort("image_save_failed")
            pose_before = node.pose_xy
            yaw_before = node.yaw
            calls += 1

            def waiting_poll():
                node.poll()
                if active_search and search is not None:
                    search.update(yaw_rad=node.yaw, now_s=time.monotonic(), target_found=False)
                    if search.status == SearchStatus.TIMED_OUT:
                        raise ApproachAbort("search_timed_out")

            grounding = wait_for_grounding(
                lambda: grounder.locate(image_path, target), poll=waiting_poll,
                stop=node.stop, now=time.monotonic, timeout_s=vlm_timeout_s,
            )
            if initially_visible is None:
                initially_visible = grounding.found
            # Never act on a view taken before appreciable uncommanded movement.
            if not node.sensors_ready():
                raise ApproachAbort("sensor_missing_invalid_or_stale")
            dyaw = math.atan2(math.sin(node.yaw-yaw_before), math.cos(node.yaw-yaw_before))
            if math.dist(node.pose_xy, pose_before) > 0.03 or abs(dyaw) > 0.05:
                raise ApproachAbort("robot_moved_during_perception")
            height, width = frame.shape[:2]
            decision = evaluate_grounding(grounding, image_width=width,
                                         image_height=height, front_distance_m=node.front,
                                         config=config)
            record = {"grounding": asdict(grounding), "action": decision.action.value,
                      "front_clearance_m": node.front, "image_width": width,
                      "image_height": height}
            (folder / f"frame-{index:03d}.json").write_text(
                json.dumps(record, indent=2), encoding="utf-8")
            if decision.measurement is not None:
                box = decision.measurement.bbox_pixels
                cv2.rectangle(frame, (int(box[0]), int(box[1])),
                              (min(width-1, int(box[2])), min(height-1, int(box[3]))),
                              (0, 255, 0), 2)
                if not cv2.imwrite(str(folder / f"box-{index:03d}.jpg"), frame):
                    raise ApproachAbort("annotation_save_failed")
            node.get_logger().info(f"{target}: {decision.action.value}")
            if not enable_motion:
                reason = "observation_only"
                break
            if decision.action == ApproachAction.STOP_TARGET_REACHED:
                candidate = True
                reason = "visual_arrival_candidate"
                break
            if decision.action in (ApproachAction.STOP_OBSTACLE,
                                   ApproachAction.STOP_SENSOR_UNAVAILABLE):
                raise ApproachAbort(decision.action.value)
            if grounding.found:
                if active_search and search is not None:
                    total_search_rotation += search.rotation_rad
                active_search = False
                node.search_tracker = None
                search = None
            else:
                if not active_search:
                    search = SearchTracker(initial_yaw_rad=node.yaw,
                                           started_at_s=time.monotonic(), timeout_s=90.0)
                    active_search = True
                    node.search_tracker = search
                if search.status != SearchStatus.SEARCHING:
                    raise ApproachAbort("object_not_found_search_limit")
            node.pulse(decision.action)
        else:
            reason = "api_call_limit"
    except ApproachAbort as error:
        reason = str(error)
    except KeyboardInterrupt:
        reason = "keyboard_cancelled"
    except Exception as error:
        reason = "controller_error"
        node.get_logger().error(f"Controller exception type: {type(error).__name__}")
    finally:
        # Best effort graceful stop; external process kill/network failure still
        # requires a verified base command watchdog or hardware stop.
        for _ in range(5):
            node.stop()
            time.sleep(0.02)
        node.destroy_node()
    if active_search and search is not None:
        total_search_rotation += search.rotation_rad
    outcome = ApproachOutcome(reason, calls, time.monotonic()-started,
                              math.degrees(total_search_rotation), candidate)
    (folder / "outcome.json").write_text(json.dumps(asdict(outcome), indent=2), encoding="utf-8")
    append_approach_trial(folder / "trial.csv", ApproachTrial(
        trial_id=trial_id, target=target, mode="simulation", start_pose=start_pose,
        model=getattr(client, "model", "unknown"), image_path=str(folder),
        initially_visible=initially_visible, elapsed_s=outcome.elapsed_s,
        search_rotation_deg=outcome.search_rotation_deg, api_calls=calls,
        failure_reason="" if candidate or reason == "observation_only" else reason,
        notes=f"{reason}; manually score grounding, success and final distance. "
              "Laser clearance is not target distance; cost unknown.",
    ))
    print(f"Evidence: {folder}")
    return outcome


def main(argv=None):
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.utilities import remove_ros_args
    from .vlm_client import GeminiVLMClient
    import sys

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--target", required=True)
    parser.add_argument("--model", required=True, help="Exact working Gemini vision model ID")
    parser.add_argument("--enable-motion", action="store_true")
    parser.add_argument("--exclusive-control", action="store_true",
                        help="Confirm Nav2/teleop have relinquished velocity control")
    parser.add_argument("--inf-is-clear", action="store_true",
                        help="Only after verifying +inf means no return in this simulator")
    parser.add_argument("--evidence-dir", default="evaluation/task4_evidence")
    parser.add_argument("--trial-id")
    parser.add_argument("--timeout", type=float, default=180.0)
    parser.add_argument("--vlm-timeout", type=float, default=20.0)
    parser.add_argument("--max-calls", type=int, default=20)
    parser.add_argument("--close-height", type=float, default=0.55)
    for name, default in (("camera", "/camera/image_raw"), ("scan", "/scan"),
                          ("odom", "/odom"), ("cmd", "/cmd_vel")):
        parser.add_argument(f"--{name}-topic", default=default)
    ros_argv = sys.argv if argv is None else ["approach_robot", *argv]
    args = parser.parse_args(remove_ros_args(args=ros_argv)[1:])
    # Disable ROS SIGINT shutdown so Python Ctrl+C can publish zero first.
    rclpy.init(args=ros_argv[1:], signal_handler_options=SignalHandlerOptions.NO)
    try:
        client = GeminiVLMClient(model=args.model)
        result = run_approach(
            args.target, client, enable_motion=args.enable_motion,
            exclusive_control=args.exclusive_control, inf_is_clear=args.inf_is_clear,
            evidence_dir=args.evidence_dir, trial_id=args.trial_id,
            timeout_s=args.timeout, vlm_timeout_s=args.vlm_timeout, max_calls=args.max_calls,
            close_height=args.close_height, camera_topic=args.camera_topic,
            scan_topic=args.scan_topic, odom_topic=args.odom_topic, cmd_topic=args.cmd_topic,
        )
        print(result.reply)
        return 0 if result.visual_candidate or result.reason == "observation_only" else 1
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
