#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import String
from moveit_api_warpper import arm_utils
from moveit_msg.srv import GetPositionFK, GetPositionFKRequest, GetPositionFKResponse


moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('robot_test_fk', anonymous=True)
robot = moveit_commander.RobotCommander()
group = moveit_commander.MoveGroupCommander("tx90_arm")

rospy.wait_for_service('compute_fk',timeout=3)
try:
    req = GetPositionFKRequest()
    req.flag = True
    client = rospy.ServiceProxy('compute_fk', GetPositionFK)
    res = client(req)
    rospy.loginfo("res is %s", res)
except rospy.ServiceException, e:
    rospy.loginfo("Service call failed: %s"%e)



home_position = [1.414566993713379, 0.33568114042282104, 1.8443951606750488, -0.11257880926132202, 1.197943091392517, -2.097616672515869]
r
