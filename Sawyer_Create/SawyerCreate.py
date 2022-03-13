# Tufts CEEO, Spring 2022
# Sawyer_Create.py
# By: Sawyer Paccione
# Completed: TBD

# Description: A Class to easily control the Create3 via python commands
# Purpose:    ""
# 
# TODO : Choose what type of messages to return
# TODO : Comment Functions
# TODO : Change Name of Class

import rclpy
import time
import math
from create3_commands.battery_subscriber import BatterySubscriber
from create3_commands.LED_publisher import LEDPublisher
from create3_commands.undock_action_client import UndockingActionClient
from create3_commands.dock_action_client import DockServoActionClient
from create3_commands.drive_distance_action_client import DriveDistanceActionClient
from create3_commands.rotate_action_client import RotateActionClient

class SawyerCreate:
    """
    A Class to streamline the calling of Create3 Commands in Python
    """

    def __init__(self):
        """
        Initialize the possible commands you can call from this class
        """
        rclpy.init(args=None)
        self.battery = BatterySubscriber()
        self.led = LEDPublisher()
        self.undock_client = UndockingActionClient()
        self.dock_client = DockServoActionClient()
        self.drive_client = DriveDistanceActionClient()
        self.rotate_client = RotateActionClient()

    def get_battery(self):
        """
        Returns
        -------
        dictionary:
            The state of the battery of the connected Create3

        ROS2
        ----
        ros2 topic echo /battery_state
        """
        rclpy.spin_once(self.battery)
        return self.battery.state

    def change_leds(self, color: str = "green"):
        """
        Parameters
        ----------
        color : string of a color name

        ROS2
        ----
        ros2 topic pub /cmd_lightring irobot_create_msgs/msg/LightringLeds "{override_system: true, leds: [{red: 255, green: 0, blue: 0}, {red: 0, green: 255, blue: 0}, {red: 0, green: 0, blue: 255}, {red: 255, green: 255, blue: 0}, {red: 255, green: 0, blue: 255}, {red: 0, green: 255, blue: 255}]}"
        
        Change the color of the LED's of the Create3's power button  
        """
        self.led.set_colors(color)
        rclpy.spin_once(self.led)

    def undock(self):
        """
        ROS2
        ----
        ros2 action send_goal /undock irobot_create_msgs/action/Undock "{}"

        Purpose
        -------
        Undock the Create3. This is a blocking function.
        """
        self.undock_client.send_goal("{}")
        rclpy.spin_once(self.undock_client)

    def dock(self):
        """
        ROS2
        ----
        ros2 action send_goal /dock irobot_create_msgs/action/DockServo "{}"

        Purpose
        -------
        Dock the Create3. This is a blocking function.
        """
        self.dock_client.send_goal("{}")
        rclpy.spin_once(self.dock_client)

    def drive_dist(self, dist: float = 0.5, speed: float = 0.15):
        """
        Parameters
        ----------
        dist: float
            The distance you want the create to travel in meters.
        speed: float
            This maximum speed that is allowed while it's moving 

        ROS2
        ----
        ros2 action send_goal /drive_distance irobot_create_msgs/action/DriveDistance "{distance: 0.5,max_translation_speed: 0.15}"
        """
        self.drive_client.send_goal(dist, speed)
        rclpy.spin_once(self.drive_client)

    def rotate_angle(self, angle: float = 1.571, speed: float = 0.5):
        """
        Parameters
        ----------
        angle: float
            The angle in radians that you want the Create3 to turn
        speed: float
            This maximum speed that is allowed while it's moving 

        ROS2
        ----
        ros2 action send_goal /rotate_angle irobot_create_msgs/action/RotateAngle "{angle: 1.57,max_rotation_speed: 0.5}"
        """
        self.rotate_client.send_goal(angle, speed)
        rclpy.spin_once(self.rotate_client)
    
    def __del__(self):
        # self.battery.destroy_node()
        # self.led.destroy_node()
        # self.undock_client.destroy_node()
        # self.dock_client.destroy_node()
        # self.drive_client.destroy_node()
        # self.rotate_client.destroy_node()
        rclpy.shutdown()


def main():
    myCreate = SawyerCreate()

    print("Getting Battery State")
    myCreate.get_battery()
    print(myCreate.battery.state)

    color = "green"
    print("Changing LED Colors to", color)
    myCreate.change_leds(color)

    print("Undocking Create3")
    myCreate.undock()  

    print("Driving 0.5")
    myCreate.drive_dist()  # Blocking

    print("Turning")
    angle = math.pi/2
    myCreate.rotate_angle(angle=angle)

    print("Docking Create3")
    myCreate.dock()  # Blocking


if __name__ == '__main__':
    main()


# from Sawyer_Create.SawyerCreate import SawyerCreate

# def main():
#     myCreate3 = SawyerCreate()
    

# if __name__ == '__main__':
#     main()