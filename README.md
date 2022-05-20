
# iRobot Create3 ROS2 Introduction

This repository has a Jupyter Notebook that is designed to serve as a brief introduction to ROS2. For more information on ROS2 please check out the official website. The functionality of this notebook relies on you already setting up communication between your computer and the Create3 previously.

(This is has been tested running on a Raspberry Pi 4, running Ubuntu 20.04.4 LTS, Python 3.8.10, and ROS2 Galactic. Connected to the Create3 via USB0 and WiFi).

## Installation

Before completing this installation pleasae see the offical [Create3 Docs](https://iroboteducation.github.io/create3_docs/). As long as you can use ros2 from the command line, continue. Specifically, [Raspberry Pi Setup](https://iroboteducation.github.io/create3_docs/setup/pi4/), and [Connect To Wifi](https://iroboteducation.github.io/create3_docs/setup/provision/).

1. Clone this repository on to your ROS2 capable device:
```bash
git clone https://github.com/paccionesawyer/Create3_ROS2_Intro.git
```
2. Install the required python package using pip or conda. For pip, use the following command:
```bash
pip3 install -r requirements.txt
```
3. (Optional) Setup Jupyter Notebook as a server so you can run the notebook remotely. [Example](https://www.digitalocean.com/community/tutorials/how-to-install-run-connect-to-jupyter-notebook-on-remote-server)

## Usage/Examples

Navigate to this repository on your local machine, and start a jupyter notebook. If you are unfamiliar with jupyter notebooks I suggest familiarizing yourself before preceding. [Link](https://docs.jupyter.org/en/latest/start/index.html)

```bash
cd Create3_ROS2_Intro
jupyter notebook
```

This should provide a URL that you can then connect to, and navigate through the tutorial. If you followed the optional step of setting up a host server, you can connect to the Jupyter Notebook from any computer on the same system.

### Making Custom Files

Another component of this repository is the program called `custom_ROS2_program.py`. This program should be run from the command line and will prompt users to input topics, and from that information build a skeletal program that the user can then improve on.

## Authors

- [@paccionesawyer](https://github.com/paccionesawyer)

## Acknowledgements

- [Create3 Documentation and Team](https://iroboteducation.github.io/create3_docs/)
- [Offical ROS2 Tutorials](https://docs.ros.org/en/galactic/Tutorials.html)
