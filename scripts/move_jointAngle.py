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
home_position = [1.414566993713379, 0.33568114042282104, 1.8443951606750488, -0.11257880926132202, 1.197943091392517, -1.097616672515869]
plan = arm_utils.fK_calculate(group,home_position)
plan.joint_trajectory.points[-1].time_from_start.secs = 10
# plan = arm_utils.set_trajectory_speed(plan, 0.8)
arm_utils.execute_plan(group,plan)
# rospy.sleep(40)

JointAngle = [1.414566993713379, 0.33568114042282104, 1.8443951606750488, -0.11257880926132202, 1.197943091392517, -0.05097616672515869]
plan = arm_utils.fK_calculate(group,JointAngle)
plan = arm_utils.set_trajectory_speed(plan, 0.8)
plan.joint_trajectory.points[-1].time_from_start.secs = 10
arm_utils.execute_plan(group,plan)

JointAngle = [0.7026929259300232, 0.7008180618286133, 1.197399377822876, -0.016282839700579643, 1.2316861152648926, 1.5247265100479126]
plan = arm_utils.fK_calculate(group,JointAngle)
plan = arm_utils.set_trajectory_speed(plan, 0.8)
plan.joint_trajectory.points[-1].time_from_start.secs = 100
arm_utils.execute_plan(group,plan)

print "Done"
# Shut down MoveIt cleanly
moveit_commander.roscpp_shutdown()
# Exit MoveIt
moveit_commander.os._exit(0)

