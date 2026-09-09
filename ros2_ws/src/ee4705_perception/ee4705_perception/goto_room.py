import sys
import math
import yaml
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient


class GotoRoom(Node):

    def __init__(self, room_name):
        super().__init__('goto_room')

        self.room_name = room_name

        self.client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

        self.send_goal()


    def send_goal(self):

        yaml_path = '/home/charanprogram/EE4705/config/room_waypoints.yaml'

        with open(yaml_path, 'r') as file:
            rooms = yaml.safe_load(file)

        if self.room_name not in rooms:
            self.get_logger().error(
                f"Room {self.room_name} not found"
            )
            return

        room = rooms[self.room_name]

        x = room['x']
        y = room['y']
        yaw = room['yaw']

        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)

        goal = NavigateToPose.Goal()

        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()

        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = 0.0

        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        self.get_logger().info(
            f"Going to {self.room_name}: x={x}, y={y}, yaw={yaw}"
        )

        self.client.wait_for_server()

        self.future = self.client.send_goal_async(
            goal
        )

        self.future.add_done_callback(
            self.goal_response_callback
        )


    def goal_response_callback(self, future):

     goal_handle = future.result()

     if not goal_handle.accepted:
        self.get_logger().error("Goal rejected")
        return

     self.get_logger().info("Goal accepted")

     self.result_future = goal_handle.get_result_async()

     self.result_future.add_done_callback(
        self.result_callback
     )


    def result_callback(self, future):

     self.get_logger().info("RESULT CALLBACK TRIGGERED")

     result = future.result()

     status = result.status

     self.get_logger().info(
        f"Nav2 finished with status: {status}"
     )

     if status == 4:
        self.get_logger().info(
            f"{self.room_name} reached successfully!"
         )

     else:
        self.get_logger().error(
            f"Navigation failed with status: {status}"
        )
def main():

    rclpy.init()

    if len(sys.argv) < 2:
        print("Usage: ros2 run ee4705_perception goto_room room_name")
        return

    room_name = sys.argv[1]

    node = GotoRoom(room_name)

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
