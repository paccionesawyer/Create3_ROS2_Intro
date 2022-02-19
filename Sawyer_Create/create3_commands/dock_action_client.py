import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from irobot_create_msgs.action import DockServo


class DockServoActionClient(Node):

    def __init__(self):
        super().__init__('dockservo_action_client')
        self._action_client = ActionClient(self, DockServo, 'dock')

    def send_goal(self, order):
        self._action_client.wait_for_server()
        goal_msg = DockServo.Goal()

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


def main(args=None):
    rclpy.init(args=args)

    action_client = DockServoActionClient()

    action_client.send_goal("{}")

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()