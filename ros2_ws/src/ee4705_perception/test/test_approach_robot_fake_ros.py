"""Exercise the real adapter against a fake ROS transport, not Gazebo."""

import contextlib
import io
import json
import math
import tempfile
import threading
import time as real_time
import types
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch

from ee4705_perception import approach_robot
from ee4705_perception.vlm_client import VLMResponse


class FakeFrame:
    shape = (480, 640, 3)
    def copy(self):
        return FakeFrame()


class Transport:
    def __init__(self):
        self.t = 100.0
        self.yaw = 0.0
        self.x = 0.0
        self.node = None
        self.published = []
        self.last = (0.0, 0.0)
        self.drop_scan = False
        self.obstacle = False
        self.cancel = False
        self.frozen_stamp = False
        self.fail_image = False
        self.api_active = False
        self.motion_during_api = False
        self.hook = lambda: None

    def modules(self):
        transport = self
        class Twist:
            def __init__(self):
                self.linear = NS(x=0.0)
                self.angular = NS(z=0.0)
        class Publisher:
            def publish(self, msg):
                value = (msg.linear.x, msg.angular.z)
                if transport.api_active and value != (0.0, 0.0):
                    transport.motion_during_api = True
                transport.last = value
                transport.published.append(value)
        class Node:
            def __init__(self, name):
                transport.node = self
                self.callbacks = {}
            def create_publisher(self, *args):
                return Publisher()
            def create_subscription(self, kind, topic, cb, qos):
                self.callbacks[topic] = cb
            def create_service(self, kind, name, cb):
                self.service = cb
            def get_clock(self):
                return NS(now=lambda: NS(nanoseconds=int(transport.t*1e9)))
            def get_logger(self):
                return NS(info=lambda *a: None, error=lambda *a: None)
            def destroy_node(self):
                pass
        def save(path, frame):
            if transport.fail_image:
                return False
            Path(path).write_bytes(b"fake image for adapter test only")
            return True
        modules = {}
        for name in ("rclpy", "rclpy.node", "rclpy.qos", "cv_bridge", "cv2",
                     "geometry_msgs", "geometry_msgs.msg", "nav_msgs", "nav_msgs.msg",
                     "sensor_msgs", "sensor_msgs.msg", "std_srvs", "std_srvs.srv"):
            modules[name] = types.ModuleType(name)
        modules["rclpy"].ok = lambda: True
        modules["rclpy"].spin_once = lambda node, timeout_sec: self.spin()
        modules["rclpy.node"].Node = Node
        modules["rclpy.qos"].qos_profile_sensor_data = object()
        modules["cv_bridge"].CvBridge = lambda: NS(imgmsg_to_cv2=lambda *a: FakeFrame())
        modules["cv2"].imwrite = save
        modules["cv2"].rectangle = lambda *a: None
        modules["geometry_msgs.msg"].Twist = Twist
        modules["nav_msgs.msg"].Odometry = object
        modules["sensor_msgs.msg"].Image = object
        modules["sensor_msgs.msg"].LaserScan = object
        modules["std_srvs.srv"].Trigger = object
        return modules

    def spin(self):
        self.t += 0.05
        self.x += self.last[0]*0.05
        self.yaw += self.last[1]*0.05
        self.hook()
        if self.cancel:
            self.node.service(NS(), NS())
        stamp_t = 100.0 if self.frozen_stamp else self.t
        sec = int(stamp_t)
        header = NS(stamp=NS(sec=sec, nanosec=int((stamp_t-sec)*1e9)))
        self.node.callbacks["/camera/image_raw"](NS(header=header))
        if not self.drop_scan:
            self.node.callbacks["/scan"](NS(
                header=header, ranges=[0.2 if self.obstacle else 2.0]*360,
                angle_min=0.0, angle_increment=math.pi/180,
                range_min=0.1, range_max=5.0))
        self.node.callbacks["/odom"](NS(header=header, pose=NS(pose=NS(
            orientation=NS(x=0, y=0, z=math.sin(self.yaw/2), w=math.cos(self.yaw/2)),
            position=NS(x=self.x, y=0.0)))))
        real_time.sleep(0.0001)  # Allow the API worker to run.


class Client:
    model = "fake-gemini"
    def __init__(self, transport, responses=None):
        self.transport = transport
        self.responses = iter(responses or ["forward"])
        self.calls = 0
        self.release = None
        self.error = False
    def ask(self, path, prompt):
        self.calls += 1
        self.transport.api_active = True
        try:
            if self.release:
                self.release.wait(2)
            if self.error:
                raise RuntimeError("mock API error")
            kind = next(self.responses, "forward")
            data = {"found": kind != "missing", "label": "cup",
                    "bbox": None if kind == "missing" else
                    ([350, 100, 650, 800] if kind == "close" else [400, 200, 600, 500])}
            return VLMResponse(json.dumps(data), self.model, 0.1)
        finally:
            self.transport.api_active = False


class AdapterTests(unittest.TestCase):
    def setUp(self):
        self.transport = Transport()
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.client = Client(self.transport)

    def run_attempt(self, **kwargs):
        fake_time = NS(monotonic=lambda: self.transport.t,
                       sleep=lambda seconds: None)
        with patch.dict("sys.modules", self.transport.modules()), \
                patch.object(approach_robot, "time", fake_time), \
                contextlib.redirect_stdout(io.StringIO()):
            return approach_robot.run_approach(
                "cup", self.client, evidence_dir=self.directory.name,
                **kwargs)

    def test_observation_publishes_nothing(self):
        result = self.run_attempt()
        self.assertEqual(result.reason, "observation_only")
        self.assertEqual(self.transport.published, [])

    def test_motion_requires_exclusive_confirmation(self):
        with self.assertRaises(ValueError):
            self.run_attempt(enable_motion=True)

    def test_move_then_candidate_stops_without_certified_success(self):
        self.client = Client(self.transport, ["forward", "close"])
        result = self.run_attempt(enable_motion=True, exclusive_control=True)
        self.assertTrue(result.visual_candidate)
        self.assertTrue(any(v[0] > 0 for v in self.transport.published))
        self.assertEqual(self.transport.published[-1], (0, 0))
        self.assertFalse(self.transport.motion_during_api)
        import csv
        csv_path = next(Path(self.directory.name).glob("*/trial.csv"))
        with csv_path.open() as f:
            row = next(csv.DictReader(f))
        self.assertEqual(row["approach_success"], "")
        self.assertEqual(row["final_distance_m"], "")

    def test_obstacle_interrupts_pulse(self):
        self.transport.hook = lambda: setattr(self.transport, "obstacle", self.transport.last[0] > 0)
        result = self.run_attempt(enable_motion=True, exclusive_control=True)
        self.assertEqual(result.reason, "obstacle")
        self.assertEqual(self.transport.published[-1], (0, 0))

    def test_stale_scan_interrupts_pulse(self):
        def hook():
            if self.transport.last[0] > 0:
                self.transport.drop_scan = True
        self.transport.hook = hook
        result = self.run_attempt(enable_motion=True, exclusive_control=True)
        self.assertIn(result.reason, ("sensor_missing_invalid_or_stale", "fresh_frame_timeout"))
        self.assertEqual(self.transport.published[-1], (0, 0))

    def test_cancel_during_vlm_wait(self):
        self.client.release = threading.Event()
        self.transport.hook = lambda: setattr(self.transport, "cancel", self.client.calls > 0)
        try:
            result = self.run_attempt(enable_motion=True, exclusive_control=True)
            self.assertEqual(result.reason, "cancelled")
            self.assertTrue(all(v == (0, 0) for v in self.transport.published))
        finally:
            self.client.release.set()

    def test_vlm_timeout(self):
        self.client.release = threading.Event()
        try:
            result = self.run_attempt(enable_motion=True, exclusive_control=True, vlm_timeout_s=0.2)
            self.assertEqual(result.reason, "vlm_timeout")
            self.assertTrue(all(v == (0, 0) for v in self.transport.published))
        finally:
            self.client.release.set()

    def test_api_error_stops(self):
        self.client.error = True
        result = self.run_attempt(enable_motion=True, exclusive_control=True)
        self.assertEqual(result.reason, "grounding_failed")
        self.assertEqual(self.transport.published[-1], (0, 0))

    def test_call_limit_stops(self):
        result = self.run_attempt(enable_motion=True, exclusive_control=True, max_calls=1)
        self.assertEqual(result.reason, "api_call_limit")
        self.assertEqual(self.client.calls, 1)
        self.assertEqual(self.transport.published[-1], (0, 0))

    def test_total_timeout_stops(self):
        result = self.run_attempt(enable_motion=True, exclusive_control=True, timeout_s=0.2)
        self.assertEqual(result.reason, "operation_timeout")

    def test_repeated_stamps_do_not_refresh_sensors(self):
        self.transport.frozen_stamp = True
        result = self.run_attempt(enable_motion=True, exclusive_control=True)
        self.assertEqual(result.reason, "fresh_frame_timeout")
        self.assertTrue(all(v == (0, 0) for v in self.transport.published))

    def test_image_save_failure_stops(self):
        self.transport.fail_image = True
        result = self.run_attempt(enable_motion=True, exclusive_control=True)
        self.assertEqual(result.reason, "image_save_failed")
        self.assertEqual(self.transport.published[-1], (0, 0))

    def test_full_turn_search_stops(self):
        self.client = Client(self.transport, ["missing"]*50)
        result = self.run_attempt(enable_motion=True, exclusive_control=True, max_calls=40)
        self.assertEqual(result.reason, "object_not_found_search_limit")
        self.assertGreaterEqual(result.search_rotation_deg, 359.0)
        self.assertLess(result.search_rotation_deg, 362.0)
        self.assertEqual(self.transport.published[-1], (0, 0))
