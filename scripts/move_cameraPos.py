#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import String
from moveit_api_warpper import arm_utils
from moveit_api_warpper import _Constant

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('robot_move_test', anonymous=True)
robot = moveit_commander.RobotCommander()
group = moveit_commander.MoveGroupCommander("tx90_arm")
plan = arm_utils.fK_calculate(group,_Constant.memoeryChip_camera_detection)
plan.joint_trajectory.points[-1].time_from_start.secs = 100
arm_utils.execute_plan(group,plan)
print "Done"
# Shut down MoveIt cleanly
moveit_commander.roscpp_shutdown()
# Exit MoveIt
moveit_commander.os._exit(0)

