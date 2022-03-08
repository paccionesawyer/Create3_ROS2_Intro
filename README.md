# Create3 Ros2 Introduction

## Section 1 - Coding Using Blockly

iRobot Coding teaches key skills by separating learning to code into 3 developmental Learning Levels. The first uses drag-and-drop, graphical blocks to teach the fundamental logic skills of coding, no reading skills required. When you're ready, use the platform's auto-level converter to instantly translate your code to the next level and create a framework of knowledge you can build upon. [Learn More](https://edu.irobot.com/what-we-offer/irobot-coding)

[Click Here to Begin Coding!](https://code.irobot.com/#/)

This setup uses bluetooth to communicate directly from your computer to your Create3, so little to no setup is required. Once on the homepage, there are a ton of great examples, click around. The goal in this section should be to undock your robot, spin around and then dock again. If you're struggling the link below will bring you to a solution.

[Example Code](https://code.irobot.com/?project=D5TJU)

## Section 2 - Coding In Python

Build upon existing knowledge by connecting new information back to past projects. The ability to flip between learning levels helps you build your way up to Learning Level 3, which uses full-text code to teach the structure and syntax of professional coding languages. [Source](https://edu.irobot.com/what-we-offer/irobot-coding)

## Section 3 - Coding with ROS2 Command Line

Now that you have begun coding you're robot we can switch to ROS 2. [This is a link to the current documentation for the Create3](https://iroboteducation.github.io/create3_docs/setup/pi4/). Navigate to the Setup tab and follow the instructions there, they also have a tutorial for setting up on a Raspberry Pi.

### Tufts Wifi

If you are on Tufts WiFi, or any network that does not support multi-casting, there are a couple extra steps to complete. Only do this once you know ROS2 has been setup correctly.

What you need to do is the following:

1. make sure that `rmw_fastrtps_cpp` is the selected RMW implementation on the robot by selecting it in the webserver configuration tab (more details here [https://iroboteducation.github.io/create3_docs/setup/provision/](https://iroboteducation.github.io/create3_docs/setup/provision/))
2. make sure that Fast-DDS is installed and it's the selected RMW implementation on your laptop (or Raspberry Pi).

```
sudo apt update && sudo apt install ros-galactic-rmw-fastrtps-cpp
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

3. Copy this XML configuration file to your laptop (or Raspberry Pi) and replace `ROBOT_IP` with the actual IP of your Create 3 at line 12

```
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
   <participant profile_name="unicast_connection" is_default_profile="true">
       <rtps>
           <builtin>
               <metatrafficUnicastLocatorList>
                   <locator/>
               </metatrafficUnicastLocatorList>
               <initialPeersList>
                   <locator>
                       <udpv4>
                           <address>ROBOT_IP</address>
                       </udpv4>
                   </locator>
               </initialPeersList>
           </builtin>
       </rtps>
   </participant>
</profiles>
```

4. Tell ROS where to find this XML file by doing: **Repeat this step on reboot**

```
export FASTRTPS_DEFAULT_PROFILES_FILE=/path/to/the/file
```

5. Kill the ROS daemon to make sure that discovery information are refreshed (**This step may need to be repeated periodically**)

```
pkill -9 _ros2_daemon
```

6. Try to communicate with the robot!

```
ros2 topic list
```

**NOTE: if Fast-DDS was not already installed on your laptop, you will also have to rebuild the Create 3 ROS 2 messages [https://github.com/iRobotEducation/irobot_create_msgs](https://github.com/iRobotEducation/irobot_create_msgs)**

Credit To [@alsora](https://github.com/alsora)

Now you can test some ROS2 Commands! Copy and past the following into the command line.

Get the battery state!
```
ros2 topic echo /battery_state
```

Change the LED Color!
```
ros2 topic pub /cmd_lightring irobot_create_msgs/msg/LightringLeds "{override_system: true, leds: [{red: 255, green: 0, blue: 0}, {red: 0, green: 255, blue: 0}, {red: 0, green: 0, blue: 255}, {red: 255, green: 255, blue: 0}, {red: 255, green: 0, blue: 255}, {red: 0, green: 255, blue: 255}]}"
```

Undock your Create3!
```
ros2 action send_goal /undock irobot_create_msgs/action/Undock "{}"
```

Dock your Create3!
```
ros2 action send_goal /dock irobot_create_msgs/action/DockServo "{}"
```

## Section 4 - Coding with ROS2 Custom Library

Clone this Github Repository onto your Pi.

```
git clone https://github.com/paccionesawyer/Create3_Python.git
```

I've created a boiler plate file for coding with this library. Edit it with your preferred text editor. For example, nano.

```
cd Create3_Python
nano Example_File.py
```

Start Coding! All the function descriptions can be found in the class definition.

## Section 5 - Creating your own Clients

To perform more complicated actions you will need to create your own client node. In order to do this, take a peak at the `SawyerCreate.py` file and the files under `SawyerCreate/create3_commands`.

<!-- IP Address: 10.245.82.16

Name: ubuntu

Password: iRobot -->
