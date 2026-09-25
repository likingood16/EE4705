# EE4705 Project 1.2 | Task 1 - simulation bringup
# Contributors (from git history): Alexander Likin (1 commit)
"""Forward /scan to the local costmap only once its transform is available.

Nav2's obstacle layer passes scans through a tf2_ros MessageFilter. When a
scan arrives before the odometry transform for its timestamp (common when
the machine is loaded), the filter registers a wait request with a timeout
timer, and the costmap's TF listener thread later runs the filter's
callbacks. Under load this path occasionally hung that listener for good:
the costmap's robot pose froze while the rest of Nav2 kept running, plans
started from the stale pose and every goal failed. It happened with both
Fast DDS and CycloneDDS, and a lifecycle reset could not recover it.

Holding each scan until the transform at its stamp (plus a small margin) is
already in TF means the costmap's filter always finds the transform
immediately, so the wait path is never used. The cost is a few tens of
milliseconds of extra scan latency.
"""

from __future__ import annotations

from collections import deque

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


class ScanRelay(Node):
    """Republish scans on /scan_costmap once TF can transform them."""

    def __init__(self) -> None:
        super().__init__("ee4705_scan_relay")

        self.declare_parameter("input_topic", "/scan")
        self.declare_parameter("output_topic", "/scan_costmap")
        self.declare_parameter("fixed_frame", "odom")
        # TF must extend this far past the scan stamp, so the costmap's own
        # (separately updated) TF buffer very likely has it too.
        self.declare_parameter("margin_s", 0.05)
        self.declare_parameter("max_age_s", 1.0)

        self.fixed_frame = str(self.get_parameter("fixed_frame").value)
        self.margin = Duration(seconds=float(self.get_parameter("margin_s").value))
        self.max_age = Duration(seconds=float(self.get_parameter("max_age_s").value))

        self.tf_buffer = Buffer()
        # No dedicated thread: TF callbacks run in this node's executor.
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)
        self.pending: deque[LaserScan] = deque(maxlen=20)
        self.dropped = 0

        self.publisher = self.create_publisher(
            LaserScan, str(self.get_parameter("output_topic").value), 10)
        self.create_subscription(
            LaserScan, str(self.get_parameter("input_topic").value),
            self.pending.append, qos_profile_sensor_data)
        # Steady clock, so the relay keeps checking even if /clock stalls.
        self.create_timer(0.01, self.flush, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def flush(self) -> None:
        while self.pending:
            scan = self.pending[0]
            stamp = Time.from_msg(scan.header.stamp)
            if self.tf_buffer.can_transform(
                self.fixed_frame, scan.header.frame_id, stamp + self.margin
            ):
                self.publisher.publish(self.pending.popleft())
                continue
            if self.get_clock().now() - stamp > self.max_age:
                self.pending.popleft()
                self.dropped += 1
                self.get_logger().warning(
                    f"Dropped a scan with no transform after {self.max_age.nanoseconds / 1e9:.1f} s "
                    f"({self.dropped} so far)", throttle_duration_sec=10.0)
                continue
            break


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ScanRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
