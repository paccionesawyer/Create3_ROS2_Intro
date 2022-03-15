import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from irobot_create_msgs.msg import HazardDetectionVector


from rclpy.action import ActionClient
from irobot_create_msgs.action import RotateAngle


class BumperTurn(Node):
    def __init__(self):
        super().__init__('bumper_turn')
        self.subscription = self.create_subscription(
            HazardDetectionVector, '/hazard_detection', self.listener_callback, qos_profile_sensor_data)
        self._action_client = ActionClient(self, RotateAngle, '/rotate_angle')

    def listener_callback(self, msg):
        '''
        This function is called every time self.subscription gets a message
        from the Robot. It then turns based on that message.
        '''
        for detection in msg.detections:
            det = detection.header.frame_id
            if det != "base_link":
                print(det)
                if det == "bump_right":
                    self.send_goal(angle=-1.57)
                elif det == "bump_left":
                    self.send_goal(angle=1.57)
                elif det == "bump_front_left":
                    self.send_goal(angle=1.57)
                elif det == "bump_front_right":
                    self.send_goal(angle=-1.57)
                elif det == "bump_front_center":
                    pass

    def send_goal(self, angle=1.57, max_rotation_speed=0.5):
        '''
        This method waits for the action server to be available, then sends a 
        goal to the server. It returns a future that we can later wait on.
        '''
        goal_msg = RotateAngle.Goal()
        goal_msg.angle = angle
        goal_msg.max_rotation_speed = max_rotation_speed

        self._action_client.wait_for_server()
        return self._action_client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)

    bumper_turn = BumperTurn()
    try:
        rclpy.spin(bumper_turn)
    except KeyboardInterrupt:
        print('\nCaught keyboard interrupt')
    finally:
        print("Done")
        rclpy.shutdown()


if __name__ == '__main__':
    main()
