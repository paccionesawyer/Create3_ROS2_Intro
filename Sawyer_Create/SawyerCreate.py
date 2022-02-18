# Tufts CEEO, Spring 2022
# Sawyer_Create.py
# By: Sawyer Paccione
# Completed: TBD

# Description: A Class to easily control the Create3 via python commands
# Purpose:    ""
# TODO : Figure out why action server is not found 
# TODO : Choose what type of messages to return
# TODO : Comment Functions
# TODO : Change Name of Class

import rclpy, time, math
from create3_commands.battery_subscriber import BatterySubscriber
from create3_commands.LED_publisher import LEDPublisher
from create3_commands.undock_action_client import UndockingActionClient
from create3_commands.dock_action_client import DockServoActionClient
from create3_commands.drive_distance_action_client import DriveDistanceActionClient 
from create3_commands.turn_action_client import RotateActionClient

class SawyerCreate:
    '''
    A Class to streamline the calling of Create3 Commands in Python
    '''
    def __init__(self):
        '''
        Initialize the possible commands you can call from this class
        '''
        rclpy.init(args=None)
        self.battery = BatterySubscriber()
        self.led = LEDPublisher()
        self.undock_client = UndockingActionClient()
        self.dock_client = DockServoActionClient()
        self.drive_client = DriveDistanceActionClient()
        self.turn_client = RotateActionClient()

    def get_battery(self):
        rclpy.spin_once(self.battery)
        return self.battery.state

    def change_leds(self, color):
        self.led.set_colors(color)
        rclpy.spin_once(self.led)

    def undock(self):
        self.undock_client.send_goal("{}")
        rclpy.spin(self.undock_client)

    def dock(self):
        self.dock_client.send_goal("{}")
        rclpy.spin(self.dock_client)

    def drive_dist(self, dist=0.5, speed=0.15):
        self.drive_client.send_goal(dist, speed)
        rclpy.spin(self.drive_client)

    def turn_angle(self, angle=(1.571), speed=0.5):
        self.turn_client.send_goal(angle, speed)
        rclpy.spin(self.turn_client)


def main():
    myCreate = SawyerCreate()

    print("Getting Battery State")
    myCreate.get_battery()
    print(myCreate.battery.state)
    
    color = "pink"
    print("Changing LED Colors to", color)
    myCreate.change_leds(color)

    print("Undocking Create3")
    myCreate.undock() ## This is blocking until the undocking action is complete

    print("Driving 0.5")
    myCreate.drive_dist() ## Blocking
    
    print("Turning")
    angle = math.pi/2
    myCreate.turn_angle(angle=angle)
    
    print("Docking Create3")
    myCreate.dock() # Blocking




if __name__ == '__main__':
    main()