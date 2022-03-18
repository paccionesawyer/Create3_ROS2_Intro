# Create3 ROS2 Introduction

This repository has a Jupyter Notebook that is designed to serve as a brief introduction to ROS2. For more information on ROS2 please check out the [official website](https://docs.ros.org/en/galactic/index.html). This page gives a broader description of ROS2, how to write your own code, and links to further tutorials.

As this is primarily used as a jupyter notebook it requires a desktop, and therefore a monitor, keyboard and mouse. However, there is an alternative for those who want to avoid doing that. Below is a set of instructions that follows the jupyter notebook exactly, and replaces every code that is run with a python file call.

## 00 Create3 ROS2 Introduction

### Table Of Contents

1. [Getting Data From the Robot (Subscriptions)](./01_Getting_Data.ipynb)
2. [Sending Data To the Robot (Publishing)](./02_Sending_Data.ipynb)
3. [Sending Actions to the Robot (Actions)](./03_Sending_Actions.ipynb)
4. [Combining Subscriptions and Publishing](./04_Combining_Sub_and_Pub.ipynb)
5. [Combining Subscriptions and Actions](./05_Combine_Sub_Actions.ipynb)
6. [Combining Subscriptions, Publishing, and Actions](./06_Combining_Sub_Pub_and_Actions.ipynb)
7. [Writing Your Own Code](./07_Writing_New_Code.ipynb)

### Purpose

This notebook is designed to serve as a brief introduction to ROS2. For more information on ROS2 please check out the last page of this notebook [Writing Your Own Code](./07_Writing_New_Code.ipynb). This page gives a broader description of ROS2, how to write your own code, and links to further tutorials.

### [ROS 2](https://docs.ros.org/en/galactic/index.html)

### More Examples

This jupyter notebook is part of a larger [repository](https://github.com/paccionesawyer/Create3_Python). More examples are found in the subfolder, [`individual_examples`](./individual_examples).

## 01 Getting Data

In ROS2, we get information from a Robot by subscribing to a topic. For this topic let's get the battery information from the Create3.

To do this from the command-line, we use the command `ros2 topic echo <topic_name>`. For example, let's get information from the topic named /battery_state.

```bash
    ros2 topic echo /battery_state
```

From this message you should have a print-out of battery information corresponding to your Create3.

However, in the command-line this isn't much help to us, so next let's see how to get this information in a python file.

### Battery Subscriber Example Code

The full code can be found in this repository under `./individual_examples/sub_battery.py` and assuming you are in the root of this repository can be run as follows.

```bash
python3 ./individual_examples/sub_battery.py
```

I would continue the same way for the remaining notebooks.
