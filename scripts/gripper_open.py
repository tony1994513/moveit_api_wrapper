#!/usr/bin/env python

import rospy
import sys
from staubli_val3_driver.srv import IOCommand,IOCommandRequest,IOCommandResponse
from moveit_api_warpper import  arm_utils 


def main():
    rospy.init_node('client_', anonymous=True)
    state = "open"
    arm_utils.gripper_control(state)


if __name__ == '__main__':
    sys.exit(main()) 