import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from irobot_create_msgs.action import Undock

class UndockingActionClient2(Node):
    '''
    A more complicated example of an action client that will cause the iRobot 
    Create3 to dock if it is currently undocked. Subclass of Node.
    '''

    def __init__(self):
        '''
        Purpose
        -------
        initialized by calling the Node constructor, naming our node 
        'undocking_action_client'
        '''
        super().__init__('undocking_action_client')
        self._action_client = ActionClient(self, Undock, 'undock')

    def send_goal(self, order):
        '''
        Purpose
        -------
        This method waits for the action server to be available, then sends a 
        goal to the server. It returns a future that we can later wait on.
        '''
        goal_msg = Undock.Goal()

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

        return self._send_goal_future

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

    action_client = UndockingActionClient2()
    future = action_client.send_goal("{}")
    rclpy.spin(action_client)

    # action_client.destroy_node()
    


if __name__ == '__main__':
    main()