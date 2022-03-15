import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from irobot_create_msgs.action import Undock


class UndockingActionClient(Node):
    '''
    A simple action client to communicate to the /undock action, this will
    only work if the Create3 is currently docked.
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

    def send_goal(self):
        '''
        Purpose
        -------
        This method waits for the action server to be available, then sends a 
        goal to the server. It returns a future that we can later wait on.
        '''
        goal_msg = Undock.Goal()
        print(goal_msg)

        self._action_client.wait_for_server()

        return self._action_client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)

    undock_client = UndockingActionClient()
    future = undock_client.send_goal()

    # The future is completed when an action server accepts or rejects the goal.
    rclpy.spin_until_future_complete(undock_client, future)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
