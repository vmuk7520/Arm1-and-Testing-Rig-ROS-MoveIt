# Arm1-and-Testing-Rig-ROS

1. [Introduction](#introduction)
1. [Before we start](#before-we-start)
1. [Setup](#setup)
1. [Week 2](#week-2)
	* [Assignment 2](#assignment-2)
1. [Week 3](#week-3)
	* [Assignment 3a](#assignment-3a)
	* [Assignment 3b](#assignment-3b)
1. [Week 4](#week-4)

## Introduction: 
This guide provides a comprehensive overview of using ROS and MoveIt to control ANT61's Arm1 and the associated testing rig. By following these instructions, you'll be able to set up, configure, and operate the system effectively. This guide is intended for robotics who are familiar with ROS and MoveIt.

## Before we start:
* __Ubuntu Installation__ :
For using ROS framework Ubuntu is necessary, if you already have a Linux system you can skip to [setup](#setup). It's Preferable that you install Ubuntu 20.04)
Follow this [Tutorial](https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview) to install Ubuntu onto your device. 

<span style="color:red">[WARNING], Do at your own risk! We will not be responsible if you lose your data. __Follow instructions carefully and make backups before you start!__</span>

* __ROS Installation/setup__: For this guide we will be using the ROS noetic distro
	- Ubuntu 20.04: [ROS Noetic](https://wiki.ros.org/noetic/Installation/Ubuntu)

## Setup:
To start, navigate to the appropriate directory:

```bash
cd Arm1-and-Testing-Rig-ROS/ANT61_ws
```
To compile run:

```
catkin make
source devel/setup.bash
```
`source devel/setup.bash` adds the *ANT61_ws* path to the current terminal session. Add this to your `.bashrc` file to ensure it is added to any future terminal sessions.

## Arm1
Everything you need to know for the Arm1 controller.

### arm_description
This package provides the complete physical description of Arm1, including the essential URDF and 3D mesh files for accurate visualization. You can run the following command to check if these files are imported correctly.

```bash
roslaunch arm_description display.launch
```
![Screenshot 2024-08-08 210441](https://github.com/user-attachments/assets/1f21b0d1-daf3-4c70-8b87-295491abbb93)

### can_interface
Communication with Arm1 utilizes a CAN bus via your computer's serial port. This package provides two scripts, CAN_rx.py and CAN_tx.py, which are both variations of the [Waveshare usb_can_adapter_v1.py](https://github.com/RajithaRanasinghe/Python-Class-for-Waveshare-USB-CAN-A/blob/main/usb_can_adapter_v1.py) script. These scripts handle CAN message exchange between your software and Arm1's hardware.

Before operating,, plug Arm1 into your device and ensure that the `tty_device` in the main function for each script matches your serial port. To find the correct serial device run the following:
```bash
ls /dev/ttyUSB*
```

### arm_control
