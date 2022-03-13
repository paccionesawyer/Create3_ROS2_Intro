import sys,rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from irobot_create_msgs.msg import HazardDetectionVector


from rclpy.action import ActionClient
from irobot_create_msgs.action import RotateAngle

class BumperTurn(Node):
    def __init__(self):
        super().__init__('bumper_turn')
        self.subscription = self.create_subscription(
            HazardDetectionVector,'/hazard_detection',self.listener_callback,qos_profile_sensor_data)        
        self._action_client = ActionClient(self, RotateAngle, '/rotate_angle')

    def listener_callback(self, msg):
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
        # self.get_logger().info('Result: {0}'.format(result))


def main(args=None):
    rclpy.init(args=args)

    bumper_turn = BumperTurn()
    try: 
        rclpy.spin(bumper_turn)
    except KeyboardInterrupt:
        print('Caught keyboard interrupt')
    except BaseException:
        print('Exception:', file=sys.stderr)
    finally:
        print("done")
        bumper_turn.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
