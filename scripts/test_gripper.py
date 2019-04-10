#!/usr/bin/env python

import rospy
import sys
from staubli_val3_driver.srv import IOCommand,IOCommandRequest,IOCommandResponse
from moveit_api_warpper import  arm_utils 


def main():
    rospy.init_node('client', anonymous=True)
    state = "chip_close"
    arm_utils.gripper_control(state)


if __name__ == '__main__':
    sys.exit(main()) 