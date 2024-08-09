# Arm1-and-Testing-Rig-ROS

1. [Introduction](#introduction)
1. [Before we start](#before-we-start)
1. [Setup](#setup)
1. [Arm1](#arm1)
	* [arm_description](#arm_description)
	* [can_interface](#can_interface)
	* [arm_control](#arm_control)
	* [arm_moveit](#arm_moveit)
1. [Testing Rig](#testing-rig)
   	* [Motor Configuration](#motor-configuration)

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

Before operating, connect Arm1 to your device and ensure the `tty_device` parameter in each script's main function matches your serial port. To identify the correct serial port, run the following command in your terminal:
```bash
ls /dev/ttyUSB*
```
### arm_control
This package includes two primary components:
* __Hardware Interface Node__: Bridges Arm1's CAN communication with MoveIt.
* __test_movement.py__: A standalone Python script for verifying MoveIt controller functionality and demonstrating system capabilities.

### arm_moveit
This MoveIt package was generated using the `moveit_setup_assistant`. It calculates the inverse-kinematics and loads controllers for the arm. To start a simulated environment, run:

```bash
roslaunch arm_moveit demo_gazebo.launch
```
![Screenshot 2024-08-08 213145](https://github.com/user-attachments/assets/ef3631ed-2682-4be6-b490-0e53a68a90f7)

Once connected to physical hardware, execute the following command to launch all necessary nodes and establish a secure connection with the arm:

```bash
roslaunch arm_moveit bringup.launch
```
Once the MoveIt controllers are operational, run `test_movement.py` to verify smooth system operation!
```bash
rosrun arm_control test_movement.py
```

## Testing Rig
Everything you need to know for the Testing Rig controller.

### Motor Configuration
Before using this software, ensure your Dynamixel motors are properly configured. We recommend following the steps outlined in this helpful [tutorial](#https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/)) 

This tutorial will walk you through customizing the motor settings to your specific needs, however the following points must be considered:
* __Motor IDs__: The 3 motors must have IDs 0, 1, 2 for starting from the most outer motor.
* __Motor speed__: Each motor should have a PWM limit of approximately 150 to reduce extreme RPM
* __Motor PID__: Set the P constant to 600, the I constant to 0, and the D constant to 2200
