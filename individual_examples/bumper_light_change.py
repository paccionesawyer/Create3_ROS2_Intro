import sys,rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from irobot_create_msgs.msg import HazardDetectionVector

from irobot_create_msgs.msg import LedColor
from irobot_create_msgs.msg import LightringLeds

"""
TODO CHANGE THE TOPIC NAMES TO MATCH NAMESPACE
"""

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

class BumperLightChange(Node):

    def __init__(self):
        super().__init__('bumper_light_change')
        self.cp = ColorPalette()
        self.lights_publisher = self.create_publisher(LightringLeds, '/cmd_lightring', 10)
        self.subscription = self.create_subscription(
            HazardDetectionVector,'/hazard_detection',self.listener_callback,qos_profile_sensor_data)        

    def listener_callback(self, msg):
        for detection in msg.detections:
            det = detection.header.frame_id
            lightring = LightringLeds()
            lightring.override_system = True
            
            if det != "base_link":
                print(det)
                if det == "bump_right":
                    light_list = Lights([self.cp.blue, self.cp.blue, self.cp.blue, self.cp.blue, self.cp.blue, self.cp.blue]) ## NOTE EDIT THIS LINE (Can get ride of Lights Object definition?)
                elif det == "bump_left":
                    light_list = Lights([self.cp.red, self.cp.red, self.cp.red, self.cp.red, self.cp.red, self.cp.red]) ## NOTE EDIT THIS LINE (Can get ride of Lights Object definition?)
                elif det == "bump_front_left":
                    light_list = Lights([self.cp.pink, self.cp.pink, self.cp.pink, self.cp.pink, self.cp.pink, self.cp.pink]) ## NOTE EDIT THIS LINE (Can get ride of Lights Object definition?)
                elif det == "bump_front_right":
                    light_list = Lights([self.cp.cyan, self.cp.cyan, self.cp.cyan, self.cp.cyan, self.cp.cyan, self.cp.cyan]) ## NOTE EDIT THIS LINE (Can get ride of Lights Object definition?)
                elif det == "bump_front_center":
                    light_list = Lights([self.cp.white, self.cp.white, self.cp.white, self.cp.white, self.cp.white, self.cp.white]) ## NOTE EDIT THIS LINE (Can get ride of Lights Object definition?)

                lightring.leds = light_list.led_colors
                self.lights_publisher.publish(lightring)
            # self.get_logger().info('I heard: "%s"' % msg)


def main(args=None):
    rclpy.init(args=args)

    hazard_subscriber = BumperLightChange()
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
