 #!/usr/bin/env python

import numpy 
from geometry_msgs.msg import Pose
import tf
from tf.transformations import (
    translation_matrix,
    quaternion_matrix,
    translation_from_matrix,
    quaternion_from_matrix,
)
import sys, os
import rospy

tool0Pose = Pose()
tool0Pose.position.x = 0.59862821679
tool0Pose.position.y = 0.402944355186
tool0Pose.position.z = 0.362606529708
tool0Pose.orientation.x = -0.919450735294
tool0Pose.orientation.y = 0.393131291518
tool0Pose.orientation.z = 0.00758573336227
tool0Pose.orientation.w = 0.000767884174764

def main():
    rospy.init_node('test_1', anonymous=True)
    pos = tool0Pose.position
    ori = tool0Pose.orientation
    transform_mat = numpy.dot(translation_matrix((pos.x, pos.y, pos.z)), quaternion_matrix((ori.x, ori.y, ori.z,ori.w)))

    rot_1 = numpy.array((
        [[1, 0, 0, 0.07],
        [0, 1, 0, 0],
        [0, 0, 1, 0.132],
        [0, 0, 0, 1]]
        ), dtype=numpy.float64)

    # rotate z axis 45 degree and move z_offset
    aa = (2**0.5)/2
    rot_2 = numpy.array((
        [[-aa, -aa, 0, 0],
        [aa, -aa, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]]
        ), dtype=numpy.float64)

    new_mat = numpy.dot(numpy.dot(transform_mat,rot_1), rot_2)
    new_trans = translation_from_matrix(new_mat)
    new_quat = quaternion_from_matrix(new_mat)   
        
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        broadcaster = tf.TransformBroadcaster()
        broadcaster.sendTransform(
                new_trans,
                new_quat,
                rospy.Time.now(),
                'hole_pose',
                'base_link', 
                )
        # broadcaster.sendTransform(
        #         ,
        #         new_quat,
        #         rospy.Time.now(),
        #         'hole_pose',
        #         'base_link', 
        # )        
        r.sleep()

if __name__ == '__main__':
    sys.exit(main()) 