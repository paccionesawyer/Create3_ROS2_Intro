import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from irobot_create_msgs.action import DriveDistance


class DriveDistanceActionClient(Node):
    '''
    An example of an action client that will cause the iRobot 
    Create3 to drive a specific distance. Subclass of Node.
    '''

    def __init__(self):
        '''
        Purpose
        -------
        initialized by calling the Node constructor, naming our node 
        'drive_distance_action_client'
        '''
        super().__init__('drive_distance_action_client')
        self._action_client = ActionClient(
            self, DriveDistance, 'drive_distance')

    def send_goal(self, distance=0.5, max_translation_speed=0.15):
        '''
        Purpose
        -------
        This method waits for the action server to be available, then sends a 
        goal to the server. It returns a future that we can later wait on.
        '''
        goal_msg = DriveDistance.Goal()
        goal_msg.distance = distance
        goal_msg.max_translation_speed = max_translation_speed

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        '''
        Purpose
        -------
        A callback that is executed when the future is complete.
        The future is completed when an action server accepts or rejects the goal request.
        '''
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        '''
        Purpose
        -------
        Similar to sending the goal, we will get a future that will complete when the result is ready.
        '''
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result))
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    action_client = DriveDistanceActionClient()

    dist  = 0.5
    speed = 0.15

    action_client.send_goal(dist, speed)
    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
