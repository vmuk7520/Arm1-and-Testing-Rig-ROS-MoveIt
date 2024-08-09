# Install script for directory: /home/victor/Arm1-and-Testing-Rig-ROS/ANT61_ws/src/testing_rig/testing_rig_control

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/victor/Arm1-and-Testing-Rig-ROS/ANT61_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/testing_rig_control/msg" TYPE FILE FILES
    "/home/victor/Arm1-and-Testing-Rig-ROS/ANT61_ws/src/testing_rig/testing_rig_control/msg/SetPosition.msg"
    "/home/victor/Arm1-and-Testing-Rig-ROS/ANT61_ws/src/testing_rig/testing_rig_control/msg/SyncSetPosition.msg"
    "/home/victor/Arm1-and-Testing-Rig-ROS/ANT61_ws/src/testing_rig/testing_rig_control/msg/BulkSetItem.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/testing_rig_control/srv" TYPE FILE FILES
    "/home/victor/Arm1-and-Testing-Rig-ROS/ANT61_ws/src/testing_rig/testing_rig_control/srv/GetPosition.srv"
    "/home/victor/Arm1-and-Testing-Rig-ROS/ANT61_ws/src/testing_rig/testing_rig_control/srv/SyncGetPosition.srv"
    "/home/victor/Arm1-and-Testing-Rig-ROS/ANT61_ws/src/testing_rig/testing_rig_control/srv/BulkGetItem.srv"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/testing_rig_control/cmake" TYPE FILE FILES "/home/victor/Arm1-and-Testing-Rig-ROS/ANT61_ws/build/testing_rig/testing_rig_control/catkin_generated/installspace/testing_rig_control-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/victor/Arm1-and-Testing-Rig-ROS/ANT61_ws/devel/include/testing_rig_control")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/victor/Arm1-and-Testing-Rig-ROS/ANT61_ws/devel/share/roseus/ros/testing_rig_control")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/victor/Arm1-and-Testing-Rig-ROS/ANT61_ws/devel/share/common-lisp/ros/testing_rig_control")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/victor/Arm1-and-Testing-Rig-ROS/ANT61_ws/devel/share/gennodejs/ros/testing_rig_control")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/victor/Arm1-and-Testing-Rig-ROS/ANT61_ws/devel/lib/python3/dist-packages/testing_rig_control")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/victor/Arm1-and-Testing-Rig-ROS/ANT61_ws/devel/lib/python3/dist-packages/testing_rig_control")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/victor/Arm1-and-Testing-Rig-ROS/ANT61_ws/build/testing_rig/testing_rig_control/catkin_generated/installspace/testing_rig_control.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/testing_rig_control/cmake" TYPE FILE FILES "/home/victor/Arm1-and-Testing-Rig-ROS/ANT61_ws/build/testing_rig/testing_rig_control/catkin_generated/installspace/testing_rig_control-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/testing_rig_control/cmake" TYPE FILE FILES
    "/home/victor/Arm1-and-Testing-Rig-ROS/ANT61_ws/build/testing_rig/testing_rig_control/catkin_generated/installspace/testing_rig_controlConfig.cmake"
    "/home/victor/Arm1-and-Testing-Rig-ROS/ANT61_ws/build/testing_rig/testing_rig_control/catkin_generated/installspace/testing_rig_controlConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/testing_rig_control" TYPE FILE FILES "/home/victor/Arm1-and-Testing-Rig-ROS/ANT61_ws/src/testing_rig/testing_rig_control/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/testing_rig_control" TYPE PROGRAM FILES "/home/victor/Arm1-and-Testing-Rig-ROS/ANT61_ws/build/testing_rig/testing_rig_control/catkin_generated/installspace/test_movement.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/testing_rig_control/testing_rig_control" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/testing_rig_control/testing_rig_control")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/testing_rig_control/testing_rig_control"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/testing_rig_control" TYPE EXECUTABLE FILES "/home/victor/Arm1-and-Testing-Rig-ROS/ANT61_ws/devel/lib/testing_rig_control/testing_rig_control")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/testing_rig_control/testing_rig_control" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/testing_rig_control/testing_rig_control")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/testing_rig_control/testing_rig_control"
         OLD_RPATH "/home/victor/Arm1-and-Testing-Rig-ROS/ANT61_ws/devel/lib:/opt/ros/noetic/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/testing_rig_control/testing_rig_control")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/testing_rig_control" TYPE DIRECTORY FILES "/home/victor/Arm1-and-Testing-Rig-ROS/ANT61_ws/src/testing_rig/testing_rig_control/include/testing_rig_control/")
endif()

