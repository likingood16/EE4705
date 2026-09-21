import sys
import math
import yaml
import rclpy
import os

from rclpy.node import Node
from rclpy.action import ActionClient
from pathlib import Path
from nav2_msgs.action import NavigateToPose


class GotoRoom(Node):

    def __init__(self, room_name):
        super().__init__('goto_room')

        self.room_name = room_name

        # Track navigation result so another program
        # such as terminal_chat.py can reuse this class
        self.finished = False
        self.success = False

        self.client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

        self.send_goal()


    def send_goal(self):
        project_root = Path(
            os.environ.get(
                'EE4705_ROOT',
                Path(__file__).resolve().parents[4]
            )
        )
        yaml_path = project_root / 'config' / 'room_waypoints.yaml'


        try:
            with open(yaml_path, 'r') as file:
                rooms = yaml.safe_load(file)

        except Exception as e:
            self.get_logger().error(
                f"Failed to load waypoint file: {e}"
            )

            self.finished = True
            self.success = False
            return


        if self.room_name not in rooms:

            self.get_logger().error(
                f"Room '{self.room_name}' not found"
            )

            self.finished = True
            self.success = False
            return


        room = rooms[self.room_name]

        x = room['x']
        y = room['y']
        yaw = room['yaw']


        # Convert yaw angle to quaternion
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)


        goal = NavigateToPose.Goal()

        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()

        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.position.z = 0.0

        goal.pose.pose.orientation.x = 0.0
        goal.pose.pose.orientation.y = 0.0
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw


        self.get_logger().info(
            f"Going to {self.room_name}: "
            f"x={x}, y={y}, yaw={yaw}"
        )


        self.get_logger().info(
            "Waiting for Nav2 action server..."
        )

        self.client.wait_for_server()


        self.goal_future = self.client.send_goal_async(
            goal
        )

        self.goal_future.add_done_callback(
            self.goal_response_callback
        )


    def goal_response_callback(self, future):

        goal_handle = future.result()

        if not goal_handle.accepted:

            self.get_logger().error(
                "Navigation goal rejected"
            )

            self.finished = True
            self.success = False
            return


        self.get_logger().info(
            "Navigation goal accepted"
        )


        self.result_future = goal_handle.get_result_async()

        self.result_future.add_done_callback(
            self.result_callback
        )


    def result_callback(self, future):

        self.get_logger().info(
            "Navigation result received"
        )

        result = future.result()

        status = result.status


        self.get_logger().info(
            f"Nav2 finished with status: {status}"
        )


        # ROS2 action status:
        # 4 = STATUS_SUCCEEDED
        if status == 4:

            self.success = True

            self.get_logger().info(
                f"{self.room_name} reached successfully!"
            )

        else:

            self.success = False

            self.get_logger().error(
                f"Navigation failed with status: {status}"
            )


        self.finished = True


def main():

    rclpy.init()


    if len(sys.argv) < 2:

        print(
            "Usage: ros2 run ee4705_perception "
            "goto_room room_name"
        )

        rclpy.shutdown()
        return


    room_name = sys.argv[1]


    node = GotoRoom(room_name)


    # Spin until navigation finishes
    while rclpy.ok() and not node.finished:

        rclpy.spin_once(
            node,
            timeout_sec=0.1
        )


    if node.success:

        print(
            f"Robot: I have arrived at {room_name}."
        )

    else:

        print(
            f"Robot: Sorry, I could not reach {room_name}."
        )


    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
