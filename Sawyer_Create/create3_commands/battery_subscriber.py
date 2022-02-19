import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState


class BatterySubscriber(Node):

    def __init__(self):
        super().__init__('battery_subscriber')
        self.subscription = self.create_subscription(
            BatteryState, '/battery_state', self.listener_callback, qos_profile_sensor_data)
        self.subscription  # prevent unused variable warning
        self.state = None

    def listener_callback(self, msg):
        # self.get_logger().info('I heard: "%s"' % msg)
        self.state = msg


def main(args=None):
    rclpy.init(args=args)

    battery_subscriber = BatterySubscriber()
    try:
        rclpy.spin(battery_subscriber)
    except KeyboardInterrupt:
        print('Caught keyboard interrupt')
    except BaseException:
        print('Exception:', file=sys.stderr)
    finally:
        print("done")
        battery_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
