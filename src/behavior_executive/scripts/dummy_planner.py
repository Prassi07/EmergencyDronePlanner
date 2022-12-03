#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from simple_drone_sim.msg import Plan, Waypoint, PoseStampedArray


class WaypointPub(object):
    def __init__(self):
        self._waypoint_pub = rospy.Publisher("/planning/global", Plan, queue_size=10)

        x_points = [300.0, 500.0, 500.0, 100.0]
        y_points = [500.0, 500.0, 100.0, 100.0]
        z_points = [20.0, 25.0, 35.0, 25.0]

        self.plan = Plan()

        self.sent = 0

        self.veh_id = 0

        self.plan.vehicle_id = self.veh_id

        for x, y, z in zip(x_points, y_points, z_points):
            wp = Waypoint()
            wp.position.position.x = x
            wp.position.position.y = y
            wp.position.position.z = z
            wp.waypoint_type = 1
            self.plan.plan.append(wp)

    def run(self):
        if self.sent != 2:
            self._waypoint_pub.publish(self.plan)
            print("sending path!")
            self.sent = self.sent + 1


if __name__ == "__main__":
    rospy.init_node("dummy_planner")
    node_ = WaypointPub()
    rate = rospy.Rate(1.0)

    while not rospy.is_shutdown():
        node_.run()
        rate.sleep()

    rospy.spin()
