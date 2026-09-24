"""Reverse a short distance when something is close in front of the robot.

After an object approach the robot stops ~0.4 m from the object, but the
SLAM map draws objects larger than they are (the fire hydrant's occupied
cells reach 0.29 m from its centre), so the robot's centre can end up inside
the costmap's inscribed radius. Nav2's planner then refuses to plan from
there and its backup recovery reports "Collision Ahead". Backing away first
puts the robot back in free space before a navigation goal is sent.
"""

from __future__ import annotations

import math
import time


def _sector_min(scan, centre, half_width):
    nearest = math.inf
    for i, value in enumerate(scan.ranges):
        angle = scan.angle_min + i * scan.angle_increment
        offset = math.atan2(math.sin(angle - centre), math.cos(angle - centre))
        if abs(offset) <= half_width and math.isfinite(value) and value >= scan.range_min:
            nearest = min(nearest, value)
    return nearest


def back_away_if_blocked(front_threshold_m=0.75, distance_m=0.35,
                         rear_clearance_m=0.35, speed_mps=0.1, timeout_s=8.0):
    """Reverse up to distance_m if anything within +/-45 deg ahead is close.

    Uses an already-initialised ROS context. Returns the distance reversed.
    """
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan

    node = rclpy.create_node("ee4705_back_away")
    state = {"scan": None, "xy": None}
    node.create_subscription(LaserScan, "/scan",
                             lambda m: state.update(scan=m), qos_profile_sensor_data)
    node.create_subscription(
        Odometry, "/odom",
        lambda m: state.update(xy=(m.pose.pose.position.x, m.pose.pose.position.y)),
        qos_profile_sensor_data)
    publisher = node.create_publisher(Twist, "/cmd_vel", 10)
    reversed_m = 0.0
    try:
        deadline = time.monotonic() + 3.0
        while (state["scan"] is None or state["xy"] is None) and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        if state["scan"] is None or state["xy"] is None:
            return 0.0
        if _sector_min(state["scan"], 0.0, math.radians(45)) >= front_threshold_m:
            return 0.0
        start = state["xy"]
        deadline = time.monotonic() + timeout_s
        command = Twist()
        command.linear.x = -speed_mps
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
            reversed_m = math.dist(state["xy"], start)
            if reversed_m >= distance_m:
                break
            # The laser sits behind the robot's centre; keep the tail clear.
            if _sector_min(state["scan"], math.pi, math.radians(40)) <= rear_clearance_m:
                break
            publisher.publish(command)
        return reversed_m
    finally:
        for _ in range(3):
            publisher.publish(Twist())
            time.sleep(0.02)
        node.destroy_node()
