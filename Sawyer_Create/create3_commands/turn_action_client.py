import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from irobot_create_msgs.action import RotateAngle

class RotateActionClient(Node):

    def __init__(self):
        super().__init__('rotate_action_client')
        self._action_client = ActionClient(self, RotateAngle, 'rotate_angle')

    def send_goal(self, angle=1.57, max_rotation_speed=0.5):
        goal_msg = RotateAngle.Goal()
        goal_msg.angle = angle 
        goal_msg.max_rotation_speed = max_rotation_speed

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

    action_client = RotateActionClient()

    angle = 1.57
    speed = 0.5 # Max 1.9

    action_client.send_goal(angle, speed)
    rclpy.spin(action_client)


if __name__ == '__main__':
    main()