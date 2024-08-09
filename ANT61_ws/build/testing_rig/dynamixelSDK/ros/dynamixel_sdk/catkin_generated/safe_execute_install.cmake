execute_process(COMMAND "/home/victor/Arm1-and-Testing-Rig-ROS/ANT61_ws/build/testing_rig/dynamixelSDK/ros/dynamixel_sdk/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/victor/Arm1-and-Testing-Rig-ROS/ANT61_ws/build/testing_rig/dynamixelSDK/ros/dynamixel_sdk/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
