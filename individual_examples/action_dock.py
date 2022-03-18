import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from irobot_create_msgs.action import DockServo

class DockServoActionClient(Node):
    '''
    A simple example of an action client that will cause the iRobot Create3
    to dock if it is currently undocked. Subclass of Node.
    '''

    def __init__(self):
        '''
        Purpose
        -------
        initialized by calling the Node constructor, naming our node 
        'dockservo_action_client'
        '''
        super().__init__('dockservo_action_client')
        self._action_client = ActionClient(self, DockServo, 'dock')

    def send_goal(self):
        '''
        Purpose
        -------
        This method waits for the action server to be available, then sends a 
        goal to the server. It returns a future that we can later wait on.
        '''
        goal_msg = DockServo.Goal()
        print(goal_msg)

        self._action_client.wait_for_server()
        return self._action_client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)

    dock_client = DockServoActionClient()
    future = dock_client.send_goal()

    # The future is completed when an action server accepts or rejects the goal.
    rclpy.spin_until_future_complete(dock_client, future)
    rclpy.shutdown()

if __name__ == '__main__':
    main()