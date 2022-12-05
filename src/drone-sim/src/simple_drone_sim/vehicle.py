import math
import numpy as np
import rospy
from geometry_msgs.msg import Point

class Vehicle:
    '''
    Vehicle frame is ENU
    '''
    def __init__(self, id_num, init_x, init_y, init_z, vehicle_l, hvel, vvel, init_theta=0, init_psi=0, init_phi=0, battery=100, in_flight_cond = True):
        self.id_num = id_num
        self.x = init_x
        self.y = init_y
        self.z = init_z
        self.psi = init_psi  # yaw angle

        self.del_t = .02

        self.setpoint_pub = rospy.Publisher(
            "/current_setpoint", Point, queue_size=10
        )

        # pid state
        self.i_accumulator_phi = 0.
        self.last_error_phi = 0.

        self.i_accumulator_z = 0.
        self.last_error_z = 0.

        # these state variables are not being maintained/used
        self.theta = init_theta  # roll angle
        self.phi = init_phi  # pitch angle
        self.vel = hvel
        self.vvel = vvel

        # self.X = np.array([self.x, self.y, self.z])
        self.vehicle_l = vehicle_l
        self.pose_tip =  [self.x + self.vehicle_l/2 * math.cos(self.phi), self.y + self.vehicle_l/2 * math.sin(self.phi)]
        self.pose_bottom =  [self.x - self.vehicle_l/2 * math.cos(self.phi), self.y - self.vehicle_l/2 * math.sin(self.phi)]

        # battery status
        self.battery = battery
        self.battery_time = rospy.Time.now()

        # flight status
        self.in_flight_cond = in_flight_cond

    def go_to_goal(self, max_omega, max_zvel, next_waypt, K_p, K_p_z):
        '''
        Returns angular velocity and velocity in z-axis towards desired direction
        '''
        K_p = 2.0
        ki_phi = 0.
        kd_phi = 0.

        K_p_z = 10.0
        ki_z = 0.
        kd_z = 0.

        e = next_waypt - [self.x, self.y, self.z]  # dist to desired position
        psi_d = math.atan2(e[1], e[0])  # desired phi

        z_error = e[2]
        phi_error = math.atan2(math.sin(psi_d - self.psi), math.cos(psi_d - self.psi))  # omega is desired heading

        # update i terms
        self.i_accumulator_phi = self.i_accumulator_phi + ki_phi * phi_error * self.del_t
        self.i_accumulator_z = self.i_accumulator_z + ki_z * z_error * self.del_t

        # update d terms
        phi_d = kd_phi * (phi_error - self.last_error_phi) / self.del_t
        self.last_error_phi = phi_error

        z_d = kd_z * (z_error - self.last_error_z) / self.del_t
        self.last_error_z = z_error

        omega = K_p* phi_error + self.i_accumulator_phi - phi_d
        z_d = K_p_z*z_error + self.i_accumulator_z - z_d

        if omega > max_omega:
            omega = max_omega  # setting max angular vel
        if z_d > max_zvel:
            z_d = max_zvel

        setpoints = Point()
        setpoints.x = self.x
        setpoints.y = self.y
        setpoints.z = 0
        self.setpoint_pub.publish(setpoints)
    
        return omega, z_d

    def position_uncertainty(self):
        '''
        Returns position uncertainty
        '''

        # hard-coding values for now
        sigma_x = 0.1
        sigma_y = 0.1
        sigma_z = 0.1
        sigma_psi = 0.1
        sigma_phi = 0.1
        sigma_theta = 0.1
        return [sigma_x, sigma_y, sigma_z, sigma_psi, sigma_phi, sigma_theta]

    def update_battery(self, time):
        updated_battery = self.battery
        if self.in_flight_cond:
            battery_delta = -0.4
            elapsed_time = (time - self.battery_time).to_sec()

            updated_battery = self.battery + battery_delta * elapsed_time
            if updated_battery <= 0.0:
                updated_battery = 0.0
                print("battery is out of charge")
                self.in_flight_cond = False

            self.battery = updated_battery
            self.battery_time = time

        return updated_battery

    def reduce_battery(self, percent, time):
        if self.battery > percent:
            self.battery = percent
            self.battery_time = time

