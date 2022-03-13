import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from irobot_create_msgs.action import DockServo

class DockServoActionClient2(Node):

    def __init__(self):
        super().__init__('dockservo_action_client')
        self._action_client = ActionClient(self, DockServo, 'dock')

    def send_goal(self, order):
        goal_msg = DockServo.Goal()

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

        return self._send_goal_future

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

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.partial_sequence))


def main(args=None):
    rclpy.init(args=args)

    action_client = DockServoActionClient2()
    future = action_client.send_goal("{}")
    rclpy.spin_until_future_complete(action_client, future)

    # action_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()