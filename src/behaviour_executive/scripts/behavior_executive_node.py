#!/usr/bin/env python3

import rospy
from behavior_executive.sim_interface import SimInterface


if __name__ == "__main__":
    rospy.init_node("dummy_planner")
    node_ = SimInterface()
    rate = rospy.Rate(0.3)

    while not rospy.is_shutdown():
        node_.send_plan(None, None, None)
        rate.sleep()

    rospy.spin()
