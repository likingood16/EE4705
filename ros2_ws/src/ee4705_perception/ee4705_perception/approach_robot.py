"""Simulation-only Task 4 object approach controller.

Prepared with AI assistance for Student C; validated in Gazebo (see
evaluation/object_approach_trials.csv). The standalone command is
observation-only unless --enable-motion is supplied. Do not use on physical
hardware. This is not a certified emergency-stop system.

Each cycle: stop -> settle -> fresh frame -> VLM bounding box (robot stopped)
-> bearing from the box centre, range from the laser along that bearing (or
from the box's bottom edge on the floor when the laser passes over the
object) -> closed-loop turn to face it -> closed-loop drive of up to 1.2 m,
guarded by the laser corridor in front of the robot -> repeat. When the
target is not visible the robot turns 45 degrees and looks again, and gives
up after a full turn.
"""

from __future__ import annotations

import argparse
import json
import math
import tempfile
import time
from dataclasses import asdict
from pathlib import Path

from .approach_geometry import (
    ROBOT_FRONT_X_M, SCAN_X_M, camera_to_scan, ground_distance,
    measure_target, pixel_bearing,
)
from .approach_policy import StepConfig, StepKind, plan_step
from .approach_result_logger import ApproachTrial, append_approach_trial
from .approach_runtime import (
    ApproachAbort, ApproachOutcome, corridor_clearance, fresh,
    range_at_bearing, scan_clearance, stamp_seconds, wait_for_grounding,
    yaw_from_quaternion,
)
from .object_grounder import ObjectGrounder

# The laser sees the robot's path as a strip this wide either side of centre
# (Waffle Pi body half-width 0.133 m, wheels at 0.144 m, plus margin).
CORRIDOR_HALF_WIDTH_M = 0.19
# Stop driving when anything in that strip is this close to the laser
# (bumper ~0.3 m from it).
STOP_CLEARANCE_M = 0.43
# Turning in place sweeps a 0.25 m radius around base_footprint; the laser is
# 0.064 m behind it, so every return must be further than this.
TURN_CLEARANCE_M = 0.32
MAX_FORWARD_MPS = 0.18
MAX_TURN_RADPS = 0.6
TURN_TOLERANCE_RAD = math.radians(1.5)
FULL_TURN_RAD = 2 * math.pi


def estimate_target_range(front_laser, box_bottom_range):
    """Pick the target range from the laser and the floor-contact estimate.

    The laser window around the bearing sees the target unless it is lower
    than the scan plane (0.13 m); then the laser reads something far behind
    it, and the box's floor contact point is the better estimate.
    """
    if front_laser is not None and math.isfinite(front_laser):
        if box_bottom_range is None or front_laser <= 1.5 * box_bottom_range + 0.3:
            return front_laser, "laser"
    if box_bottom_range is not None:
        return box_bottom_range, "floor"
    return None, "unknown"


def run_approach(target, client, *, enable_motion=False, exclusive_control=False,
                 evidence_dir="evaluation/task4_evidence", trial_id=None,
                 camera_topic="/camera/image_raw", scan_topic="/scan",
                 odom_topic="/odom", cmd_topic="/cmd_vel",
                 inf_is_clear=False, timeout_s=300.0, vlm_timeout_s=30.0,
                 max_calls=25, step_config=StepConfig(), use_sim_time=True):
    """Run one attempt in an already initialized ROS context.

    Caller must ensure Nav2/teleop have relinquished velocity control. Stop
    via /task4/stop or Ctrl+C. Blocks chat input but spins the stop service.
    """
    # Lazy ROS imports keep the offline tests importable without ROS.
    import cv2
    import rclpy
    from cv_bridge import CvBridge
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
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
    base = Path(evidence_dir)
    base.mkdir(parents=True, exist_ok=True)
    # Unique directory; never overwrite earlier experiment images.
    folder = Path(tempfile.mkdtemp(prefix="attempt-", dir=base))
    trial_id = trial_id or folder.name
    started = time.monotonic()
    calls = 0
    tokens = [0, 0]
    search_rotation = 0.0
    search_start_yaw = None
    search_turns = 0
    initially_visible = None
    final_range = None

    class ApproachNode(Node):
        def __init__(self):
            # Sensor stamps are simulation time, so the freshness check must be too.
            super().__init__("task4_approach", parameter_overrides=[
                Parameter("use_sim_time", value=use_sim_time)])
            self.bridge = CvBridge()
            self.cancelled = False
            self.fault = None
            self.frame = None
            self.yaw = None
            self.scan = None
            self.around = None
            self.pose_xy = None
            self.received = {}
            self.stamps = {}
            self.frame_sequence = 0
            self.publisher = self.create_publisher(Twist, cmd_topic, 10)
            # Best-effort images lose fragments under CycloneDDS (config/cyclonedds.xml).
            self.create_subscription(Image, camera_topic, self.on_image,
                                     QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
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
            self.scan = (message.ranges, message.angle_min, message.angle_increment,
                         message.range_min, message.range_max)
            self.around = scan_clearance(*self.scan, all_around=True, inf_is_clear=inf_is_clear)

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

        def stale_sensors(self):
            now = time.monotonic()
            return [name for name in ("camera", "scan", "odom")
                    if not fresh(self.received.get(name), now, 0.75)]

        def sensors_ready(self):
            return (not self.stale_sensors() and self.frame is not None
                    and self.yaw is not None and self.scan is not None)

        def require_sensors(self):
            if not self.sensors_ready():
                stale = self.stale_sensors()
                self.get_logger().warning(f"Stale or missing sensors: {stale}")
                raise ApproachAbort("sensor_missing_invalid_or_stale")

        def corridor(self):
            return corridor_clearance(*self.scan, half_width=CORRIDOR_HALF_WIDTH_M,
                                      inf_is_clear=inf_is_clear)

        def publish(self, linear, angular):
            msg = Twist()
            msg.linear.x = float(linear)
            msg.angular.z = float(angular)
            self.publisher.publish(msg)

        def turn_by(self, angle, limit_s=25.0):
            """Closed-loop turn on odometry; returns the rotation achieved."""
            goal = self.yaw + angle
            previous = self.yaw
            turned = 0.0
            deadline = time.monotonic() + limit_s
            try:
                while True:
                    self.poll()
                    self.require_sensors()
                    if self.around is None:
                        raise ApproachAbort("rotation_scan_unavailable")
                    if self.around <= TURN_CLEARANCE_M:
                        raise ApproachAbort("obstacle_during_turn")
                    turned += abs(math.atan2(math.sin(self.yaw - previous),
                                             math.cos(self.yaw - previous)))
                    previous = self.yaw
                    error = math.atan2(math.sin(goal - self.yaw), math.cos(goal - self.yaw))
                    if abs(error) <= TURN_TOLERANCE_RAD:
                        break
                    if time.monotonic() >= deadline:
                        raise ApproachAbort("turn_timeout")
                    speed = max(0.15, min(MAX_TURN_RADPS, 1.5 * abs(error)))
                    self.publish(0.0, math.copysign(speed, error))
            finally:
                self.stop()
            return turned

        def drive_forward(self, distance, limit_s=30.0):
            """Drive straight on odometry; stops early for anything in the path.

            Returns the distance driven and whether the path blocked it.
            """
            x0, y0 = self.pose_xy
            heading = self.yaw
            deadline = time.monotonic() + limit_s
            travelled = 0.0
            blocked = False
            try:
                while True:
                    self.poll()
                    self.require_sensors()
                    clearance = self.corridor()
                    if clearance is None:
                        raise ApproachAbort("sensor_missing_invalid_or_stale")
                    travelled = math.dist(self.pose_xy, (x0, y0))
                    remaining = distance - travelled
                    if clearance <= STOP_CLEARANCE_M:
                        blocked = True
                        break
                    if remaining <= 0.01:
                        break
                    if time.monotonic() >= deadline:
                        raise ApproachAbort("drive_timeout")
                    speed = min(MAX_FORWARD_MPS, 0.8 * remaining + 0.04,
                                0.6 * (clearance - STOP_CLEARANCE_M) + 0.03)
                    drift = math.atan2(math.sin(heading - self.yaw), math.cos(heading - self.yaw))
                    self.publish(speed, max(-0.3, min(0.3, 2.0 * drift)))
            finally:
                self.stop()
            return travelled, blocked

    node = ApproachNode()
    grounder = ObjectGrounder(client)
    reason = "not_started"
    arrived = False
    start_pose = ""
    last_blocked = False
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
            settle_until = time.monotonic() + 0.4
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
            image_path = folder / f"frame-{index:03d}.jpg"
            frame = node.frame.copy()
            scan = node.scan
            if not cv2.imwrite(str(image_path), frame):
                raise ApproachAbort("image_save_failed")
            pose_before = node.pose_xy
            yaw_before = node.yaw
            calls += 1
            grounding = wait_for_grounding(
                lambda: grounder.locate(image_path, target), poll=node.poll,
                stop=node.stop, now=time.monotonic, timeout_s=vlm_timeout_s,
            )
            calls += grounding.retries
            tokens[0] += grounding.input_tokens or 0
            tokens[1] += grounding.output_tokens or 0
            if initially_visible is None:
                initially_visible = grounding.found
            # Never act on a view taken before appreciable uncommanded movement.
            node.require_sensors()
            dyaw = math.atan2(math.sin(node.yaw-yaw_before), math.cos(node.yaw-yaw_before))
            if math.dist(node.pose_xy, pose_before) > 0.03 or abs(dyaw) > 0.05:
                raise ApproachAbort("robot_moved_during_perception")

            height, width = frame.shape[:2]
            record = {"grounding": asdict(grounding), "image_width": width,
                      "image_height": height}
            bearing = target_range = None
            source = "not_found"
            if grounding.found:
                box = measure_target(grounding.bbox, image_width=width, image_height=height)
                bearing = pixel_bearing(box.center_x_pixels, width)
                floor = ground_distance(box.bbox_pixels[3], width, height)
                floor_range = camera_to_scan(floor, bearing)[0] if floor is not None else None
                # Laser window: the box's angular half-width, at least 3 degrees.
                half = abs(pixel_bearing(box.bbox_pixels[0], width)
                           - pixel_bearing(box.bbox_pixels[2], width)) / 2
                laser = range_at_bearing(*scan, bearing=bearing,
                                         half_window=min(max(half, math.radians(3)), math.radians(12)),
                                         inf_is_clear=inf_is_clear)
                target_range, source = estimate_target_range(laser, floor_range)
                record.update(bbox_pixels=box.bbox_pixels, bearing_deg=math.degrees(bearing),
                              laser_range_m=laser, floor_range_m=floor_range,
                              target_range_m=target_range, range_source=source)
                cv2.rectangle(frame, (int(box.bbox_pixels[0]), int(box.bbox_pixels[1])),
                              (min(width-1, int(box.bbox_pixels[2])),
                               min(height-1, int(box.bbox_pixels[3]))), (0, 255, 0), 2)
                label = (f"{grounding.label} {math.degrees(bearing):+.0f}deg "
                         + (f"{target_range:.2f}m ({source})" if target_range else "range ?"))
                cv2.putText(frame, label, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            (0, 255, 0), 2)
                if not cv2.imwrite(str(folder / f"box-{index:03d}.jpg"), frame):
                    raise ApproachAbort("annotation_save_failed")
            step = plan_step(target_found=grounding.found, bearing_rad=bearing,
                             target_range_m=target_range, config=step_config)
            record["step"] = {"kind": step.kind.value, "turn_deg": math.degrees(step.turn_rad),
                              "forward_m": step.forward_m}
            (folder / f"frame-{index:03d}.json").write_text(
                json.dumps(record, indent=2), encoding="utf-8")
            node.get_logger().info(
                f"{target}: {step.kind.value} turn={math.degrees(step.turn_rad):+.0f}deg "
                f"forward={step.forward_m:.2f}m range={target_range} ({source})")
            if grounding.found:
                final_range = target_range
            if not enable_motion:
                reason = "observation_only"
                break

            if step.kind == StepKind.SEARCH:
                if search_start_yaw is None:
                    search_start_yaw, search_turns = node.yaw, 0
                search_turns += 1
                # Aim at absolute headings from the search start so the small
                # stopping error of each turn does not accumulate.
                goal = search_start_yaw + search_turns * step.turn_rad
                node.turn_by(math.atan2(math.sin(goal - node.yaw), math.cos(goal - node.yaw)))
                if search_turns * step.turn_rad >= FULL_TURN_RAD - 1e-6:
                    # Back at the first view, which was already checked.
                    search_rotation += search_turns * step.turn_rad + math.atan2(
                        math.sin(node.yaw - goal), math.cos(node.yaw - goal))
                    search_start_yaw = None
                    raise ApproachAbort("object_not_found_search_limit")
                continue
            if search_start_yaw is not None:
                # Target found mid-search: count the rotation it took, then a
                # new search starts if the target is lost again.
                search_rotation += search_turns * step_config.search_step_rad
                search_start_yaw = None
            if step.turn_rad:
                node.turn_by(step.turn_rad)
            if step.kind == StepKind.ARRIVED:
                arrived = True
                reason = "arrived"
                break
            if step.forward_m >= 0.05:
                _, blocked = node.drive_forward(step.forward_m)
                # Blocked twice in a row while the target is still far: something
                # other than the target is in the way.
                if blocked and last_blocked:
                    raise ApproachAbort("path_blocked")
                last_blocked = blocked
        else:
            reason = "api_call_limit"
    except ApproachAbort as error:
        reason = str(error)
    except KeyboardInterrupt:
        reason = "keyboard_cancelled"
    except Exception as error:
        reason = "controller_error"
        node.get_logger().error(f"Controller exception: {type(error).__name__}: {error}")
    finally:
        if search_start_yaw is not None:
            search_rotation += search_turns * step_config.search_step_rad
        # Best effort graceful stop; external process kill/network failure still
        # requires a verified base command watchdog or hardware stop.
        for _ in range(5):
            node.stop()
            time.sleep(0.02)
        node.destroy_node()
    outcome = ApproachOutcome(
        reason, calls, time.monotonic()-started, math.degrees(search_rotation), arrived,
        target=target.strip(), initially_visible=initially_visible,
        final_range_m=final_range, evidence_dir=str(folder),
        input_tokens=tokens[0], output_tokens=tokens[1])
    (folder / "outcome.json").write_text(json.dumps(asdict(outcome), indent=2), encoding="utf-8")
    append_approach_trial(folder / "trial.csv", ApproachTrial(
        trial_id=trial_id, target=target, mode="simulation", start_pose=start_pose,
        model=getattr(client, "model", "unknown"), image_path=str(folder),
        initially_visible=initially_visible, elapsed_s=outcome.elapsed_s,
        search_rotation_deg=outcome.search_rotation_deg, api_calls=calls,
        failure_reason="" if arrived or reason == "observation_only" else reason,
        notes=f"{reason}; final laser range {final_range} m (bumper is "
              f"{ROBOT_FRONT_X_M - SCAN_X_M:.2f} m ahead of the laser). "
              "Score grounding and final distance against Gazebo.",
    ))
    print(f"Evidence: {folder}")
    return outcome


def make_vlm_client(provider, model=None):
    from .vlm_client import GeminiVLMClient, QwenVLMClient

    if provider == "gemini":
        return GeminiVLMClient(model=model)
    if provider == "qwen":
        return QwenVLMClient(model=model)
    raise ValueError(f"Unknown provider: {provider}")


def main(argv=None):
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.utilities import remove_ros_args
    import sys

    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--target", required=True)
    parser.add_argument("--provider", choices=["qwen", "gemini"], default="qwen")
    parser.add_argument("--model", help="Model ID (default: the provider's configured model)")
    parser.add_argument("--enable-motion", action="store_true")
    parser.add_argument("--exclusive-control", action="store_true",
                        help="Confirm Nav2/teleop have relinquished velocity control")
    parser.add_argument("--inf-is-clear", action="store_true",
                        help="Only after verifying +inf means no return in this simulator")
    parser.add_argument("--evidence-dir", default="evaluation/task4_evidence")
    parser.add_argument("--trial-id")
    parser.add_argument("--timeout", type=float, default=300.0)
    parser.add_argument("--vlm-timeout", type=float, default=30.0)
    parser.add_argument("--max-calls", type=int, default=25)
    for name, default in (("camera", "/camera/image_raw"), ("scan", "/scan"),
                          ("odom", "/odom"), ("cmd", "/cmd_vel")):
        parser.add_argument(f"--{name}-topic", default=default)
    ros_argv = sys.argv if argv is None else ["approach_robot", *argv]
    args = parser.parse_args(remove_ros_args(args=ros_argv)[1:])
    # Disable ROS SIGINT shutdown so Python Ctrl+C can publish zero first.
    rclpy.init(args=ros_argv[1:], signal_handler_options=SignalHandlerOptions.NO)
    try:
        client = make_vlm_client(args.provider, args.model)
        result = run_approach(
            args.target, client, enable_motion=args.enable_motion,
            exclusive_control=args.exclusive_control, inf_is_clear=args.inf_is_clear,
            evidence_dir=args.evidence_dir, trial_id=args.trial_id,
            timeout_s=args.timeout, vlm_timeout_s=args.vlm_timeout, max_calls=args.max_calls,
            camera_topic=args.camera_topic, scan_topic=args.scan_topic,
            odom_topic=args.odom_topic, cmd_topic=args.cmd_topic,
        )
        print(result.reply)
        return 0 if result.visual_candidate or result.reason == "observation_only" else 1
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
