import sys,rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from irobot_create_msgs.msg import HazardDetectionVector

class HazardSubscriber(Node):

    def __init__(self):
        super().__init__('hazard_subscriber')
        self.subscription = self.create_subscription(
            HazardDetectionVector,'/pacman/hazard_detection',self.listener_callback,qos_profile_sensor_data)        
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        for detection in msg.detections:
            print(detection.header.frame_id)
            # self.get_logger().info('I heard: "%s"' % msg)


def main(args=None):
    rclpy.init(args=args)

    hazard_subscriber = HazardSubscriber()
    try: 
        rclpy.spin(hazard_subscriber)
    except KeyboardInterrupt:
        print('Caught keyboard interrupt')
    except BaseException:
        print('Exception:', file=sys.stderr)
    finally:
        print("done")
        hazard_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
