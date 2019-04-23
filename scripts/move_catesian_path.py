#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import String
from moveit_api_warpper import arm_utils
import pdb

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('robot_catestain_move', anonymous=True)
robot = moveit_commander.RobotCommander()
group = moveit_commander.MoveGroupCommander("tx90_arm")
position = [0.702692985534668, 0.7728180885314941, 1.2635552883148193, -0.017287401482462883, 1.093550443649292, 1.5272510051727295]
home_pose = arm_utils.fk_compute_service(position)
current_pose = group.get_current_pose()
# pdb.set_trace()
plan = arm_utils.IK_cartesian_plan(group,current_pose,home_pose,0.01)
# plan = arm_utils.set_trajectory_speed(plan, 0.8)
arm_utils.execute_plan(group,plan)
# rospy.sleep(10)

# JointAngle = [0.726246178150177, 1.018491268157959, 1.3517783880233765, -0.10556681454181671, 0.7094179391860962, 2.743661880493164]
# plan = arm_utils.fK_calculate(group,JointAngle)
# plan = arm_utils.set_trajectory_speed(plan, 0.8)
# arm_utils.execute_plan(group,plan)
# rospy.sleep(10)
# Shut down MoveIt cleanly
moveit_commander.roscpp_shutdown()
# Exit MoveIt
moveit_commander.os._exit(0)

