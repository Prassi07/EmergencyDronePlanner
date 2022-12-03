#!/usr/bin/env python

import os
import rospy
import numpy as np
from std_msgs.msg import String


def battery_sim():
    pub = rospy.Publisher('status', String, queue_size = 10)
    rospy.init_node('battery_sim', anonymous = True)
    rate = rospy.Rate(10) # 10 hz

    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()


def listener():
    rospy.init_node('listener', anonymous=True)
    # rospy.Subscriber("chatter", String, callback)

if __name__ == '__main__':
    try:
        battery_sim()
    except rospy.ROSInterruptException:
        pass 


