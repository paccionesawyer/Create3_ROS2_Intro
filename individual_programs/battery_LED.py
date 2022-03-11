import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState

from irobot_create_msgs.msg import LedColor
from irobot_create_msgs.msg import LightringLeds


class ColorPalette():
    ''' 
    Helper Class to define frequently used colors
    '''

    def __init__(self):
        self.red = LedColor(red=255, green=0, blue=0)
        self.green = LedColor(red=0, green=255, blue=0)
        self.blue = LedColor(red=0, green=0, blue=255)
        self.yellow = LedColor(red=255, green=255, blue=0)
        self.pink = LedColor(red=255, green=0, blue=255)
        self.cyan = LedColor(red=0, green=255, blue=255)
        self.purple = LedColor(red=127, green=0, blue=255)
        self.white = LedColor(red=255, green=255, blue=255)
        self.grey = LedColor(red=189, green=189, blue=189)
        self.tufts_blue = LedColor(red=98, green=166, blue=10)
        self.tufts_brown = LedColor(red=94, green=75, blue=60)


class BatteryPercentageLED(Node):
    '''
    A Node to change the LEDs based on the battery percentage.
    '''

    def __init__(self, namespace: str = ""):
        super().__init__('battery_percentage')

        # Create a subscriber to the battery state topic
        self.subscription = self.create_subscription(
            BatteryState, namespace + '/battery_state', self.listener_callback, 
            qos_profile_sensor_data)

        # Create a publisher to the LED topic
        self.lights_publisher = self.create_publisher(
            LightringLeds, namespace + '/cmd_lightring', 10)

        # Initialize ColorPallete so we have simple access to RGB values
        self.cp = ColorPalette()

        # Initialize the structure of the message we are publishing to the LED
        # topic
        self.lightring = LightringLeds()
        self.lightring.override_system = True

    def listener_callback(self, msg: BatteryState):
        '''
        Parameters
        ----------
        :type msg: BatteryState
        :rtype: None

        Purpose
        -------
        Whenever our subscriber (listener) get's a message this function is 
        'called back' to and ran.
        '''

        # Uncomment this line to see the full message recieved /from
        # battery_state
        # self.get_logger().info('I heard: "%s"' % msg)
        self.changeLED(msg.percentage)

    def changeLED(self, percentage: float):
        '''
        Parameters
        ----------
        :type percentage: float
        :rtype: None

        Purpose
        -------
        Change the LEDs to correspond to the current battery percentage.
        0-33 -> Red
        33-67 -> Yellow
        67-100 -> Green
        '''

        print("Battery Percentage:", percentage)

        # Initialize to white
        to_publish = [self.cp.white, self.cp.white, self.cp.white,
                      self.cp.white, self.cp.white, self.cp.white]

        if percentage <= 0.33:
            # Change ALL 6 LEDs to Red
            to_publish = [self.cp.red, self.cp.red, self.cp.red,
                          self.cp.red, self.cp.red, self.cp.red]
        elif percentage <= 0.67:
            # Change ALL 6 LEDs to Yellow
            to_publish = [self.cp.yellow, self.cp.yellow, self.cp.yellow,
                          self.cp.yellow, self.cp.yellow, self.cp.yellow]
        elif percentage <= 1.0:
            # Change ALL 6 LEDs to Green
            to_publish = [self.cp.green, self.cp.green, self.cp.green,
                          self.cp.green, self.cp.green, self.cp.green]

        current_time = self.get_clock().now()
        self.lightring.header.stamp = current_time.to_msg()
        self.lightring.leds = to_publish
        self.lights_publisher.publish(self.lightring)

    def reset(self):
        '''
        Purpose
        -------
        Release control of the LEDs back to the Create3.
        '''

        self.lightring.override_system = False
        white = [self.cp.white, self.cp.white, self.cp.white,
                 self.cp.white, self.cp.white, self.cp.white]
        self.lightring.leds = white
        self.lights_publisher.publish(self.lightring)


def main(args=None):
    rclpy.init(args=args)

    battery_percentage_LED = BatteryPercentageLED()

    try:
        rclpy.spin(battery_percentage_LED)
    except KeyboardInterrupt:
        print('\nCaught Keyboard Interrupt')
    except BaseException:
        print('Exception:', file=sys.stderr)
    finally:
        print("Done")
        battery_percentage_LED.reset()
        battery_percentage_LED.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
