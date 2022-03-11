import sys
import rclpy
from rclpy.node import Node

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


class LEDPublisher(Node):
    '''
    Publish a custom color to each LED on the Create3
    '''

    def __init__(self, namespace: str = ""):
        super().__init__('led_publisher')

        self.cp = ColorPalette()
        self.lights_publisher = self.create_publisher(
            LightringLeds, namespace + '/cmd_lightring', 10)
    
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.lightring = LightringLeds()
        self.lightring.override_system = True

    def timer_callback(self):
        '''
        Purpose
        -------
        This function will be called every 0.5 seconds, right now it's 
        publishing the same colors every time. 
        Try and see if you can get the colors to change each time this function 
        is called!
        '''
        # NOTE Edit this line to change color
        led_colors = [self.cp.purple, self.cp.cyan, self.cp.pink,
                      self.cp.blue, self.cp.red, self.cp.green]
        current_time = self.get_clock().now()

        self.lightring.leds = led_colors
        self.lightring.header.stamp = current_time.to_msg()
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

    led_publisher = LEDPublisher()
    try:
        rclpy.spin(led_publisher)
    except KeyboardInterrupt:
        print('\nCaught Keyboard Interrupt')
    except BaseException:
        print('Exception:', file=sys.stderr)
    finally:
        print("Done")
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        led_publisher.reset()
        led_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
