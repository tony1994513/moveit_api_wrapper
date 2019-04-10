#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import String
from moveit_api_warpper import arm_utils

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('robot_move_test', anonymous=True)
robot = moveit_commander.RobotCommander()
group = moveit_commander.MoveGroupCommander("tx90_arm")
home_position = [1.414566993713379, 0.33568114042282104, 1.8443951606750488, -0.11257880926132202, 1.197943091392517, -2.097616672515869]
plan = arm_utils.fK_calculate(group,home_position)
# plan = arm_utils.set_trajectory_speed(plan, 0.8)
arm_utils.execute_plan(group,plan)
rospy.sleep(10)

JointAngle = [0.726246178150177, 1.018491268157959, 1.3517783880233765, -0.10556681454181671, 0.7094179391860962, 2.743661880493164]
plan = arm_utils.fK_calculate(group,JointAngle)
plan = arm_utils.set_trajectory_speed(plan, 0.8)
arm_utils.execute_plan(group,plan)
rospy.sleep(10)
# Shut down MoveIt cleanly
moveit_commander.roscpp_shutdown()
# Exit MoveIt
moveit_commander.os._exit(0)

