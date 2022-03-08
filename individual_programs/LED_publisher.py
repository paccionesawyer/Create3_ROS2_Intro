import rclpy
from rclpy.node import Node

from irobot_create_msgs.msg import LedColor
from irobot_create_msgs.msg import LightringLeds

class ColorPalette():
    """ Helper Class to define frequently used colors"""
    def __init__(self):
        self.red = LedColor(red=255,green=0,blue=0)
        self.green = LedColor(red=0,green=255,blue=0)
        self.blue = LedColor(red=0,green=0,blue=255)
        self.yellow = LedColor(red=255,green=255,blue=0)
        self.pink = LedColor(red=255,green=0,blue=255)
        self.cyan = LedColor(red=0,green=255,blue=255)
        self.purple = LedColor(red=127,green=0,blue=255)
        self.white = LedColor(red=255,green=255,blue=255)
        self.grey = LedColor(red=189,green=189,blue=189)
        self.tufts_blue = LedColor(red=98,green=166,blue=10)
        self.tufts_brown = LedColor(red=94,green=75,blue=60)

class Lights():
    """ Class to tell the robot to set lightring lights as part of dance sequence"""
    def __init__(self, led_colors):
        """
        Parameters
        ----------
        led_colors : list of LedColor
            The list of 6 LedColors corresponding to the 6 LED lights on the lightring
        """
        self.led_colors = led_colors

class LEDPublisher(Node):

    def __init__(self):
        super().__init__('led_publisher')
        self.cp = ColorPalette()
        self.lights_publisher = self.create_publisher(LightringLeds, 'cmd_lightring', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        current_time = self.get_clock().now()
        test = Lights([self.cp.blue, self.cp.green, self.cp.blue, self.cp.green, self.cp.blue, self.cp.green]) ## NOTE EDIT THIS LINE (Can get ride of Lights Object definition?)
        lightring = LightringLeds()
        lightring.override_system = True
        lightring.leds = test.led_colors

        self.get_logger().info('Time %f New lights action, first led (%d,%d,%d)' % (current_time.nanoseconds / float(1e9), lightring.leds[0].red, lightring.leds[0].green, lightring.leds[0].blue))

        lightring.header.stamp = current_time.to_msg()

        self.lights_publisher.publish(lightring)


def main(args=None):
    rclpy.init(args=args)

    led_publisher = LEDPublisher()

    rclpy.spin(led_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    lightring = LightringLeds()
    lightring.override_system = False
    
    led_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()