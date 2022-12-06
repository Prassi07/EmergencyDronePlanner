import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import UInt8
from simple_drone_sim.msg import Plan, Waypoint, PoseStampedArray, BatteryArray


class SimInterface(object):
    def __init__(self):
        self._waypoint_pub = rospy.Publisher("/global_path", Plan, queue_size=10)
        self.landing_pub = rospy.Publisher(
            "/drone_sim/command_land", UInt8, queue_size=10
        )

        self.battery = None
        self.odom = None

        self._odom_sub = rospy.Subscriber(
            "/drone_sim/vehicle_poses", PoseStampedArray, self.vehicle_odom_callback
        )

        self._battery_sub = rospy.Subscriber(
            "/drone_sim/vehicle_battery", BatteryArray, self.vehicle_battery_callback
        )

        # only controlling one drone at a time
        self.veh_id = 0

    def get_vehicle_odom(self):
        return self.odom

    def get_vehicle_battery(self):
        return self.battery

    def command_land(self):
        self.landing_pub.publish(UInt8(data=self.veh_id))

    def send_plan(self, plan):
        plan.vehicle_id = self.veh_id
        self._waypoint_pub.publish(plan)

    def vehicle_odom_callback(self, msg):
        self.odom = msg.poses[self.veh_id]

    def vehicle_battery_callback(self, msg):
        self.battery = msg.vehicles[self.veh_id]
