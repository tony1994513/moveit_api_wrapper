#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import String
from moveit_api_warpper import arm_utils

DEBUG = True


moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('robot_move_test',
                anonymous=True)
robot = moveit_commander.RobotCommander()
group = moveit_commander.MoveGroupCommander("tx90_arm")
# Increase the planning time since constraint planning can take a while
robot.set_planning_time(15)    
# Allow replanning to increase the odds of a solution
robot.allow_replanning(True)
# Allow some leeway in position(meters) and orientation (radians)
robot.set_goal_position_tolerance(0.05)
robot.set_goal_orientation_tolerance(0.1)

if DEBUG:
    print "============ Reference frame for this robot: %s" % group.get_planning_frame()
    print "============ End-effector link for this group: %s" % group.get_end_effector_link()
    print "============ Robot Groups:"
    print robot.get_group_names()
    print "============ Printing robot state"
    print arm_utils.print_robot_Jointstate(robot)
    print "============"

print "============ Generating plan 1"
pose_target = Pose()
pose_target.orientation.w = 1.0
pose_target.position.x = 0.7
pose_target.position.y = -0.05
pose_target.position.z = 1.1
plan = arm_utils.iK_calculate(group, pose_target)     
arm_utils.execute_plan(group,plan)

print arm_utils.print_robot_Jointstate(robot)

target_joint = [-0.14257286131927796, 1.0905119122949902, -0.59253489336999264, -3.1406088825859366, 0.6986560818776872, -2.999702977027948]
plan = arm_utils.fK_calculate(group,target_joint)
arm_utils.execute_plan(group,plan)


# Shut down MoveIt cleanly
moveit_commander.roscpp_shutdown()
# Exit MoveIt
moveit_commander.os._exit(0)

