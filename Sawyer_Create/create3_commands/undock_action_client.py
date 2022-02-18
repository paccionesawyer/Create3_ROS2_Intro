import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from irobot_create_msgs.action import Undock


class UndockingActionClient(Node):

    def __init__(self):
        super().__init__('undocking_action_client')
        self._action_client = ActionClient(self, Undock, 'undock')

    def send_goal(self, order):
        goal_msg = Undock.Goal()
        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result))
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    action_client = UndockingActionClient()

    action_client.send_goal("{}")

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()