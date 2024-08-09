# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "controller_manager;hardware_interface;roscpp;std_msgs;rospy;dynamixel_sdk".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-ltesting_rig_control".split(';') if "-ltesting_rig_control" != "" else []
PROJECT_NAME = "testing_rig_control"
PROJECT_SPACE_DIR = "/home/victor/Arm1-and-Testing-Rig-ROS/ANT61_ws/install"
PROJECT_VERSION = "0.0.0"
