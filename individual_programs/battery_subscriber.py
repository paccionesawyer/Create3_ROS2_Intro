import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState


class BatterySubscriber(Node):
    '''
    A Node listening to the /battery_state topic.
    '''

    def __init__(self, namespace: str = ""):
        super().__init__('battery_subscriber')
        self.subscription = self.create_subscription(
            BatteryState, namespace + '/battery_state', self.listener_callback,
            qos_profile_sensor_data)

    def listener_callback(self, msg: BatteryState):
        """
        :type msg: BatteryState
        :rtype: None
        """
        self.get_logger().info('I heard: "%s"' % msg)
        self.printBattery(msg)

    def printBattery(self, msg):
        """
        :type msg: BatteryState
        :rtype: None
        """
        # We can get components of the message by using the '.' dot operator
        print("Battery Percentage:", msg.percentage)


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
