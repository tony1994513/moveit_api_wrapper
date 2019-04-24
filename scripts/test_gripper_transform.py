#!/usr/bin/env python

import rospy
import sys
from moveit_api_warpper import  arm_utils 
import moveit_commander
from moveit_commander import MoveGroupCommander


def main():
    rospy.init_node('test_1', anonymous=True)
    # Initialize the move_group API
    moveit_commander.roscpp_initialize(sys.argv) 
    # Initialize the MoveIt! commander for the right arm
    right_arm = MoveGroupCommander('tx90_arm')
    # Get the end-effector link
    end_effector_link = right_arm.get_end_effector_link()
    # Get the end-effector pose
    ee_pose = right_arm.get_current_pose(end_effector_link)

    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        gripper_pose = arm_utils.tool0TgripperTransform(ee_pose.pose) 
        r.sleep()





if __name__ == '__main__':
    sys.exit(main()) 