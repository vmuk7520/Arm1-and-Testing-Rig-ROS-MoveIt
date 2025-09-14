
# Arm1 and Testing Rig ROS Workspace

## Table of Contents
1. [Project Overview](#project-overview)
2. [Workspace Structure](#workspace-structure)
3. [Before You Start](#before-you-start)
4. [Setup & Build Instructions](#setup--build-instructions)
5. [Arm1 System](#arm1-system)
	- [arm_description](#arm_description)
	- [can_interface](#can_interface)
	- [arm_control](#arm_control)
	- [arm_moveit](#arm_moveit)
6. [Testing Rig System](#testing-rig-system)
	- [Motor Configuration](#motor-configuration)
	- [testing_rig_description](#testing_rig_description)
	- [dynamixel_sdk](#dynamixel_sdk)
	- [testing_rig_control](#testing_rig_control)
	- [testing_rig_moveit](#testing_rig_moveit)
7. [Troubleshooting & Tips](#troubleshooting--tips)

---

## Project Overview

This workspace provides a complete ROS Noetic and MoveIt-based solution for controlling the ANT61 Arm1 and a Dynamixel-powered Testing Rig. It includes:
- Physical and kinematic descriptions (URDF, meshes)
- Hardware interface nodes for CAN and Dynamixel
- MoveIt integration for motion planning and control
- Example scripts for testing and demonstration

This guide is intended for robotics engineers familiar with ROS and MoveIt.

---

## Workspace Structure

```
ANT61_ws/
├── build/           # Catkin build artifacts
├── devel/           # Catkin development environment
├── logs/            # Build and runtime logs
├── src/             # Source code for all packages
│   ├── arm1/        # Custom code for Arm1
│   ├── testing_rig/ # Custom code for Testing Rig
│   ├── ...          # Other packages (see below)
├── .catkin_workspace
├── .catkin_tools/
└── ...
```

### Main Packages in `src/`

| Package                | Description |
|------------------------|-------------|
| `arm_description`      | URDF & meshes for Arm1 |
| `can_interface`        | CAN bus communication scripts |
| `arm_control`          | Hardware interface node & test script |
| `arm_moveit`           | MoveIt config for Arm1 |
| `testing_rig_description` | URDF & meshes for Testing Rig |
| `dynamixel_sdk`        | Dynamixel motor SDK (from Robotis) |
| `dynamixel_sdk_examples` | Example nodes for Dynamixel SDK |
| `testing_rig_control`  | Hardware interface node & test script |
| `testing_rig_moveit`   | MoveIt config for Testing Rig |

---

## Before You Start

### 1. Ubuntu Installation
ROS Noetic requires Ubuntu 20.04. If you already have a compatible Linux system, skip to [Setup](#setup--build-instructions).

**[WARNING]**: Proceed at your own risk! Back up your data before making system changes.

- [Ubuntu Installation Tutorial](https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview)

### 2. ROS Noetic Installation
- [ROS Noetic Installation Guide](https://wiki.ros.org/noetic/Installation/Ubuntu)

---

## Setup & Build Instructions

1. **Clone the repository** (if not already done):
	```bash
	git clone <repo_url>
	cd Arm1-and-Testing-Rig-ROS-MoveIt/ANT61_ws
	```
2. **Build the workspace:**
	```bash
	catkin_make
	source devel/setup.bash
	```
	Add `source /path/to/ANT61_ws/devel/setup.bash` to your `~/.bashrc` for convenience.

---

## Arm1 System

### arm_description
Contains the URDF and mesh files for Arm1. For visualization, run:
```bash
roslaunch arm_description display.launch
```
![Arm1 Visualization](https://github.com/user-attachments/assets/1f21b0d1-daf3-4c70-8b87-295491abbb93)

### can_interface
Provides CAN bus communication scripts (`CAN_rx.py`, `CAN_tx.py`). These are based on [Waveshare usb_can_adapter_v1.py](https://github.com/RajithaRanasinghe/Python-Class-for-Waveshare-USB-CAN-A/blob/main/usb_can_adapter_v1.py).

**Setup:**
- Connect Arm1 to your device.
- Edit the `tty_device` parameter in each script to match your serial port.
- Find your serial port:
	```bash
	ls /dev/ttyUSB*
	```

### arm_control
Contains:
- **Hardware Interface Node**: Bridges CAN communication with MoveIt.
- **test_movement.py**: Standalone script to verify MoveIt controller and demonstrate system capabilities.

### arm_moveit
MoveIt configuration for Arm1 (generated via `moveit_setup_assistant`).

**Simulated environment:**
```bash
roslaunch arm_moveit demo_gazebo.launch
```
![Arm1 MoveIt Demo](https://github.com/user-attachments/assets/ef3631ed-2682-4be6-b490-0e53a68a90f7)

**Physical hardware:**
```bash
roslaunch arm_moveit bringup.launch
```
Once controllers are running, verify with:
```bash
rosrun arm_control test_movement.py
```

---

## Testing Rig System

### Motor Configuration
Before using the software, configure your Dynamixel motors. Follow the [Dynamixel Wizard2 Tutorial](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/)).

<img width="424" alt="Dynamixel Wizard2" src="https://github.com/user-attachments/assets/24aa3c79-afe7-4b96-a472-df31d37fa73b">

**Required settings:**
- **Motor IDs**: 0, 1, 2 (outermost to innermost)
- **PWM limit**: ~150
- **PID**: P=600, I=0, D=2200

### testing_rig_description
Contains URDF and mesh files for the Testing Rig. For visualization, run:
```bash
roslaunch testing_rig_description display.launch
```
![Testing Rig Visualization](https://github.com/user-attachments/assets/1678362f-3e7b-450d-b176-4dc2c8fdace9)

### dynamixel_sdk
Official [ROBOTIS DynamixelSDK](https://github.com/ROBOTIS-GIT/DynamixelSDK) for motor control.

**Setup:**
- Plug in the rig.
- Edit `DEVICE_NAME` in `dynamixel_sdk_examples/src/sync_read_write_node.cpp` to match your serial port.
- Find your serial port:
	```bash
	ls /dev/ttyUSB*
	```

### testing_rig_control
Contains:
- **Hardware Interface Node**: Bridges DynamixelSDK nodes with MoveIt.
- **test_movement.py**: Standalone script to verify MoveIt controller and demonstrate system capabilities.

### testing_rig_moveit
MoveIt configuration for the Testing Rig.

**Simulated environment:**
```bash
roslaunch testing_rig_moveit demo_gazebo.launch
```
![Testing Rig MoveIt Demo](https://github.com/user-attachments/assets/ef3631ed-2682-4be6-b490-0e53a68a90f7)

**Physical hardware:**
```bash
roslaunch testing_rig_moveit bringup.launch
```
Once controllers are running, verify with:
```bash
rosrun testing_rig_control test_movement.py
```

---

## Troubleshooting & Tips

- **Source your workspace**: Always run `source devel/setup.bash` in each new terminal.
- **Serial port issues**: Use `ls /dev/ttyUSB*` to find your device. Check permissions if you can't access it.
- **Gazebo/MoveIt errors**: Ensure all dependencies are installed (`rosdep install --from-paths src --ignore-src -r -y`).
- **Rebuild after changes**: Run `catkin_make` after modifying source code or configuration files.
- **Check logs**: See `logs/` for build/runtime errors.
- **Documentation**: Refer to package-level README files (if present) for more details.

---

## Contributing

Pull requests and issues are welcome! Please follow ROS and MoveIt best practices.

---

## License

See individual package folders for license information.

