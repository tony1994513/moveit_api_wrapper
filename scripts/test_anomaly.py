#!/usr/bin/env python
import rospy
import sys
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Int32,Bool

Force = None
threshold = 20

def callback(data):
    global Force
    Force = data

def main():
    rospy.init_node('anomaly_detetion_test', anonymous=True)
    rospy.wait_for_message('wrench/filtered',WrenchStamped, timeout=3)
    rospy.Subscriber("wrench/filtered", WrenchStamped, callback)
    threshold_pub = rospy.Publisher('anomaly_threshold', Int32, queue_size=10)
    anomaly_detetion_pub = rospy.Publisher('anomaly_detection_pub', Bool, queue_size=10)
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        detection_msg = Bool()
        if Force:
            if Force > threshold:
                detection_msg.data = True
            else:
                detection_msg.data = False
            anomaly_detetion_pub.publish(detection_msg)
        
        threshold_msg = Int32()
        threshold_msg.data = threshold
        threshold_pub.publish(threshold_msg)
        rate.sleep()


if __name__ == '__main__':
    sys.exit(main()) 