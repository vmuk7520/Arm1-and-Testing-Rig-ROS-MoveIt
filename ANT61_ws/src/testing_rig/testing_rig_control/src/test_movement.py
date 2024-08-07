#!/usr/bin/env python3
"""
Node for testing the movement of a arm1 by specifying either joint positions or 
a pose goal
"""

import sys
import copy
import rospy
import time
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import numpy as np
import time
import math
import random


class RigController():
    """ Class to send control commands to Arm1 """

    def __init__(self):
        """ initialise the object """

        self.robot = moveit_commander.RobotCommander()

        # MoveGroupCommander object to plan and execute arm commands
        groupName = "testing_rig"
        self.moveGroup = moveit_commander.MoveGroupCommander(groupName)

        # MoveGroupCommander object to build collision scene
        self.scene = moveit_commander.PlanningSceneInterface()

        self.success = False

    def goToJointState(self, jointGoal):
        """ send the rig to a given set of joint states """
        if(len(jointGoal) != 3):
            print("Incorrect number of joint angles provided")
            return -1

        # send the robot to the given joint positions
        self.success = self.moveGroup.go(jointGoal, wait=True)

        # ensure there is no residual movement
        self.moveGroup.stop()

    def goToPoseGoal(self, poseGoal):
        """ send the rig to a given pose """
        self.moveGroup.set_pose_target(poseGoal)

        # send the robot to the given pose
        self.success = self.moveGroup.go(wait=True)

        # ensure there is no residual movement
        self.moveGroup.stop()

        self.moveGroup.clear_pose_targets()

    def plan_cartesian_path(self, waypoints):
        ''' Plan a Cartesian path directly by specifying a list of waypoints
        for the end-effector to go through.'''

        (plan, fraction) = self.moveGroup.compute_cartesian_path(
            waypoints, 0.001, 50  # waypoints to follow  # eef_step
        )  # jump_threshold

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction
    
    def execute_plan(self, plan):

        move_group = self.moveGroup

        '''execute if you would like the robot to follow
        the plan that has already been computed:'''
        move_group.execute(plan, wait=True)

    def euler_to_quaternion(self, eulerAngles):

        roll = eulerAngles[0]
        pitch = eulerAngles[1]
        yaw = eulerAngles[2]

        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)

        quaternionMessage = geometry_msgs.msg.Quaternion()

        quaternionMessage.w = cy * cr * cp + sy * sr * sp
        quaternionMessage.x = cy * sr * cp - sy * cr * sp
        quaternionMessage.y = cy * cr * sp + sy * sr * cp
        quaternionMessage.z = sy * cr * cp - cy * sr * sp

        return quaternionMessage
    
    def quaternion_to_euler(self, quaternionMessage):
        
        w = quaternionMessage.w
        x = quaternionMessage.x
        y = quaternionMessage.y
        z = quaternionMessage.z

        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (w * y - z * x)
        if math.fabs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        eulerAngles = [roll,pitch,yaw]

        return eulerAngles


def main():
    rospy.init_node("RigController")

    # Initiate ArmController Object
    rigController = RigController()

    x = 0
    y = 0
    z = 0

    while not rospy.is_shutdown():
        try:
            # joints = np.radians([0,0,0])
            # rigController.goToJointState(joints)
            # exit()
            
            joints = np.radians([x,y,z])
            rigController.goToJointState(joints)
            current = rigController.moveGroup.get_current_pose()
            current_angles = rigController.quaternion_to_euler(current.pose.orientation)
            print(f'\nBEACON STATE:\nRoll: {current_angles[0]}\nPitch: {current_angles[1]}\nYaw: {current_angles[2]}\n')
            time.sleep(3)

            x=random.uniform(-360, 360)
            y=random.uniform(-360, 360)
            z=random.uniform(-360, 360)

            print(f'Motor Bearings: {x}, {y}, {z}')

        except rospy.ROSInterruptException:
            return
        except KeyboardInterrupt:
            return

if __name__ == "__main__":
    main()
