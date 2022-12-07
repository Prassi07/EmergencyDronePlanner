import math
from enum import Enum
import rospy

# msgs
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
from simple_drone_sim.msg import Plan

# application
from behavior_executive.sim_interface import SimInterface


class BehaviorStates(Enum):
    INIT = 0
    FOLLOW_GLOBAL = 1
    GO_TO_LZ = 2
    LAND = 3


class BehaviorExecutive(object):
    def __init__(self):
        self.sim_interface = SimInterface()

        self.global_plan = None
        self.has_new_plan = False
        self.wp_num = 0

        self.lz_plans = None
        self.selected_lz_plan = None
        self.has_new_lz = False
        self.lz_wp_num = 0
        self.sent_lz = False
        self.at_lz = False

        self.state = BehaviorStates.INIT

        # meters to consider a wp reached
        self.reached_thresh = 10.0

        self._waypoint_dist_remain_pub = rospy.Publisher(
            "/behavior_executive/distance_to_wp", Float32, queue_size=10
        )

        self._battery_remain_pub = rospy.Publisher(
            "/behavior_executive/battery", Float32, queue_size=10
        )

        self._wp_number_pub = rospy.Publisher(
            "/behavior_executive/waypoint_number", Float32, queue_size=10
        )

        self.lz_path_pub = rospy.Publisher(
            "/behavior_executive/landing_zone_path", Path, queue_size=10, latch=True
        )

        self._global_plan_sub = rospy.Subscriber(
            "/planning/global", Plan, self.global_path_callback
        )

        self._landing_zone_plans = rospy.Subscriber(
            "/planning/landing_zones", Plan, self.lz_path_callback
        )

    def publish_lz_path(self, plan):
        msg = Path()
        msg.header = plan.header

        for wp in plan.plan:
            pose = PoseStamped()
            pose.pose = wp.position
            msg.poses.append(pose)

        self.lz_path_pub.publish(msg)

    def global_path_callback(self, msg):
        self.global_plan = msg
        self.has_new_plan = True
        rospy.loginfo("BehaviorExecutive: Received new plan!")

    def lz_path_callback(self, msg):
        if not self.sent_lz:
            self.lz_plans = msg
            self.publish_lz_path(msg)
            self.has_new_lz = True

    def has_reached(self, wp, odom):
        if wp is None or odom is None:
            return False
        diff_x = abs(wp.position.position.x - odom.pose.position.x)
        diff_y = abs(wp.position.position.y - odom.pose.position.y)
        diff_z = abs(wp.position.position.z - odom.pose.position.z)

        dist = math.sqrt(diff_x * diff_x + diff_y * diff_y + diff_z * diff_z)

        self._waypoint_dist_remain_pub.publish(Float32(dist))

        return dist <= self.reached_thresh

    def should_land(self):
        # TODO: add battery smarts
        return False

    def command_land(self):
        self.sim_interface.command_land()

    def update_viz(self):
        bat = self.sim_interface.get_vehicle_battery()

        if bat is not None:
            self._battery_remain_pub.publish(Float32(bat.percent))

        if self.global_plan is not None:
            self._wp_number_pub.publish(
                Float32(data=self.wp_num % len(self.global_plan.plan))
            )

    def send_lz_plan(self):
        if self.lz_plans is not None:
            if not self.sent_lz:
                self.sim_interface.send_plan(self.lz_plans)
                self.sent_lz = True

            rospy.loginfo_once("BehaviorExecutive: Sending LZ plan!")
            if self.has_reached(
                self.lz_plans.plan[0], self.sim_interface.get_vehicle_odom()
            ):
                self.lz_wp_num = self.lz_wp_num + 1
                if self.lz_wp_num == len(self.lz_plans.plan):
                    rospy.logwarn("BehaviorExecutive: Reached end of LZ plan!")
                    self.at_lz = True
                    return

                # go to next wp
                first = self.lz_plans.plan[0]
                self.lz_plans.plan = self.lz_plans.plan[1:]
                self.lz_plans.plan.append(first)
                self.sim_interface.send_plan(self.lz_plans)
                rospy.loginfo(
                    "BehaviorExecutive: Sending next LZ waypoint! Sent total {}".format(
                        self.lz_wp_num
                    )
                )

    def send_global_plan(self):
        # new global plan received, reset
        if self.has_new_plan:
            self.wp_num = 0
            self.sim_interface.send_plan(self.global_plan)
            self.has_new_plan = False

        if self.global_plan is not None:
            if self.has_reached(
                self.global_plan.plan[0], self.sim_interface.get_vehicle_odom()
            ):
                self.wp_num = self.wp_num + 1
                if self.wp_num == len(self.global_plan.plan):
                    rospy.logwarn("BehaviorExecutive: Reached end of plan!")
                    return True
                else:
                    # go to next wp
                    first = self.global_plan.plan[0]
                    self.global_plan.plan = self.global_plan.plan[1:]
                    self.global_plan.plan.append(first)
                    self.sim_interface.send_plan(self.global_plan)
                    rospy.loginfo(
                        "BehaviorExecutive: Sending next waypoint! Sent total {}".format(
                            self.wp_num
                        )
                    )
        return False

    def publish_next_waypoint(self):
        if self.state == BehaviorStates.INIT:
            if self.has_new_plan:
                self.state = BehaviorStates.FOLLOW_GLOBAL
        elif self.state == BehaviorStates.FOLLOW_GLOBAL:
            if self.should_land():
                self.state = BehaviorStates.GO_TO_LZ
            else:
                done = self.send_global_plan()
                if done:
                    self.state = BehaviorStates.GO_TO_LZ
        elif self.state == BehaviorStates.GO_TO_LZ:
            if self.at_lz:
                self.state = BehaviorStates.LAND
            else:
                self.send_lz_plan()
        elif self.state == BehaviorStates.LAND:
            self.command_land()
        else:
            rospy.logerr(
                "BehaviorExecutive: Invalid state reached! State: {}".format(self.state)
            )
        self.update_viz()

    def run(self):
        self.publish_next_waypoint()
