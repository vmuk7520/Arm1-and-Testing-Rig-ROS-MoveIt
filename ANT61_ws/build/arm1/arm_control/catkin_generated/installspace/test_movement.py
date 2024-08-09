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

class ArmController():
    """ Class to send control commands to Arm1 """

    def __init__(self):
        """ initialise the object """

        self.robot = moveit_commander.RobotCommander()

        # MoveGroupCommander object to plan and execute arm commands
        groupName = "arm1"
        self.moveGroup = moveit_commander.MoveGroupCommander(groupName)

        # MoveGroupCommander object to build collision scene
        self.scene = moveit_commander.PlanningSceneInterface()

        self.success = True

    def goToJointState(self, jointGoal):
        """ send the arm to a given set of joint states """
        if(len(jointGoal) != 4):
            print("Incorrect number of joint angles provided")
            return -1

        # send the robot to the given joint positions
        self.moveGroup.go(jointGoal, wait=True)

        # ensure there is no residual movement
        self.moveGroup.stop()

    def goToPoseGoal(self, poseGoal):
        """ send the arm to a given pose """
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

def main():
    rospy.init_node("ArmController")

    # Initiate ArmController Object
    armController = ArmController()

    # sequentially rotate between different positions
    while True:
        try:
            joints = np.radians([0,50,60,20])
            armController.goToJointState(joints)

            rospy.sleep(5)
            
            joints = np.radians([-20,50,-90,0])
            armController.goToJointState(joints)
            
            rospy.sleep(5)

        except rospy.ROSInterruptException:
            return
        except KeyboardInterrupt:
            return

if __name__ == "__main__":
    main()
