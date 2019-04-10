#!/usr/bin/env python

import rospy
import sys
from linemod_pose_estimation.srv import linemod_pose, linemod_poseRequest, linemod_poseResponse
from moveit_api_warpper import  arm_utils 


def main():
    rospy.init_node('client_1', anonymous=True)
    object_id = 0 # 0: memory chip; 1: cpu
    arm_utils.object_pose(object_id)


if __name__ == '__main__':
    sys.exit(main()) 
