import rospy
from geometry_msgs.msg import PoseStamped
from simple_drone_sim.msg import Plan, Waypoint, PoseStampedArray, BatteryArray

class SimInterface(object):
    def __init__(self):
        self._waypoint_pub = rospy.Publisher(
            "/global_path", Plan, queue_size=10
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

        self.plan = None


    def get_vehicle_odom(self):
        return self.odom


    def send_plan(self, x_points, y_points, z_points):
        if self.plan is None:
            x_points = [300., 350., 300., 350.]
            y_points = [500., 300., 450., 100.]
            z_points = [20., 25., 35., 25.]

            self.plan = Plan()
            self.plan.vehicle_id = self.veh_id

            for x, y, z in zip(x_points, y_points, z_points):
                wp = Waypoint()
                wp.position.position.x = x
                wp.position.position.y = y
                wp.position.position.z = z
                wp.waypoint_type = 1
                self.plan.plan.append(wp)

        first = self.plan.plan[0]
        self.plan.plan = self.plan.plan[1:]
        self.plan.plan.append(first)
        self._waypoint_pub.publish(self.plan)


    def vehicle_odom_callback(self, msg):
        self.odom = msg.poses[self.veh_id]

    def vehicle_battery_callback(self, msg):
        self.battery = msg.vehicles[self.veh_id]
