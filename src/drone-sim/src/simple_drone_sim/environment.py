import math
import random
from turtle import heading
import numpy as np
from simple_drone_sim.vehicle import *
from simple_drone_sim.target import *
from simple_drone_sim.sensor import *
from simple_drone_sim.obstacle import *
from geographic_msgs.msg import GeoPose


class Environment:
    def __init__(self, list_of_target_dicts=[], max_omega=5, max_zvel=5,
                 init_x=None, init_y=None, init_z=None, init_psi=None,
                 K_p=0.01, K_p_z=0.01,
                 vehicle_num=3, vehicle_l=3, hvel=5, vvel=2, n_rand_targets=-1, n_rand_obst=-1, del_t=1,
                 waypt_threshold=5,
                 sensor_focal_length=5, sensor_width=10, sensor_height=10, sensor_a=1,
                 sensor_b=1, sensor_d=1, sensor_g=1, sensor_h=1, sensor_pitch=20, list_of_obstacle_dicts=[]):
        '''
        Setup simulation environment
        '''
        # if initial position not specified, randomly spawn vehicle between (50, 1000)
        init_x = random.randrange(50, 1000) if init_x is None else init_x
        init_y = random.randrange(50, 1000) if init_y is None else init_y
        init_z = random.randrange(20, 120,
                                  20) if init_z is None else init_z  # discretized by step-size 20
        init_psi = random.uniform(0, np.pi) if init_psi is None else init_psi

        # drone pose
        self.init_x = init_x
        self.init_y = init_y
        self.init_z = init_z
        self.init_psi = init_psi

        self.del_t = del_t

        self.vehicle_num = vehicle_num
        self.vehicle_l = vehicle_l  # vehicle length
        self.hvel = hvel  # horizontal velocity
        self.vvel = vvel  # vertical velocity
        self.max_omega = max_omega  # max angular velocity
        self.max_zvel = max_zvel  # max vertical velocity

        self.sensor_focal_length = sensor_focal_length
        self.sensor_width = sensor_width
        self.sensor_height = sensor_height
        self.sensor_a = sensor_a
        self.sensor_b = sensor_b
        self.sensor_d = sensor_d
        self.sensor_g = sensor_g
        self.sensor_h = sensor_h
        self.sensor_pitch = sensor_pitch

        # if targets not specified, randomly generate between 1-10 targets
        self.n_rand_targets = random.randrange(1, 10) if not list_of_target_dicts and n_rand_targets == -1 else n_rand_targets
        self.n_rand_obst = random.randrange(1, 10) if not list_of_obstacle_dicts and n_rand_obst == -1 else n_rand_obst

        self.obstacles = self.generate_obstacles(list_of_obstacle_dicts, self.n_rand_obst)
        self.targets = self.generate_targets(list_of_target_dicts, self.n_rand_targets)

        self.global_waypt_list = [[] for i in range(vehicle_num)]

        self.waypt_threshold = waypt_threshold

        self.K_p = K_p  # x-y proportionality constant for PID controller
        self.K_p_z = K_p_z  # z-axis proportionality constant for PID controller

        self.vehicle = [self.init_vehicle(i) for i in range(self.vehicle_num)]
        self.sensor = self.init_sensor()


        self.curr_waypt_num = 0

    def is_in_collision(self):
        for i in range(self.vehicle_num):
            for obst in self.obstacles:
                if obst.is_in_collision(self.vehicle[i].x, self.vehicle[i].y, self.vehicle[i].z):
                    return True
        return False

    def generate_obstacles(self, obsts, num_obst):
        if obsts is None or len(obsts) == 0:
            obstacles = [
                Obstacle(
                    id=idx,
                    init_x=np.random.uniform(-600, 600),
                    init_y=np.random.uniform(-600, 600),
                    width=20.0,
                    length=20.0,
                    height=100.0,
                    speed=0.0,
                    hold_heading_time=1.
                ) for idx in range(num_obst)
            ]
            return obstacles
        else:
            obstacles = [
                Obstacle(
                    id=obst["id"],
                    init_x=obst["x"], init_y=obst["y"],
                    width=obst["width"],
                    length=obst["length"],
                    height=obst["height"],
                    speed=obst["speed"],
                    hold_heading_time=obst["hold_heading_time"]
                ) for obst in obsts
            ]
            return obstacles

    def generate_targets(self, list_of_target_dicts, n_rand_targets=None):
        '''
        Generates landing zones with initial positions
        '''

        # when no targets specified
        if not list_of_target_dicts:
            if n_rand_targets is None:
                raise ValueError(
                    "Passed in no targets but didn't pass in n_rand_targets")
            targets = []
            idx = 0
            while idx < n_rand_targets:
                init_x=np.random.uniform(-600, 600)
                init_y=np.random.uniform(-600, 600)
                
                in_col = False
                for obst in self.obstacles:
                    if obst.is_in_collision(init_x, init_y, 0, 10.0):
                        in_col = True
                if not in_col:
                    target = Target(
                        id=idx,
                        init_x=init_x,
                        init_y=init_y,
                        heading=np.random.uniform(0, 2 * 3.1416),
                        linear_speed=0.0,
                        angular_speed=0.0,
                        linear_speed_std=0.0,
                        angular_speed_std=0.0
                    )
                    targets.append(target)
                    idx = idx + 1
            return targets
        # when targets are specified
        else:
            targets = [
                Target(
                    id=target["id"],
                    init_x=target["x"], init_y=target["y"], heading=target["heading"],
                    linear_speed=target["linear_speed"],
                    angular_speed=target["angular_speed"],
                    linear_speed_std=target["linear_speed_std"],
                    angular_speed_std=target["angular_speed_std"]
                ) for target in list_of_target_dicts
            ]
        return targets

    def init_vehicle(self, id_num):
        return Vehicle(id_num=id_num,
                       init_x=self.init_x,
                       init_y=self.init_y+(id_num*200),
                       init_z=self.init_z + (id_num*20),
                       init_psi=self.init_psi,
                       vehicle_l=self.vehicle_l,
                       hvel=self.hvel,
                       vvel=self.vvel)

    def init_sensor(self):
        return SensorModel(self.sensor_a,
                           self.sensor_b,
                           self.sensor_d,
                           self.sensor_g,
                           self.sensor_h,
                           self.sensor_width,
                           self.sensor_height,
                           self.sensor_focal_length)

    def get_ground_intersect(self, vehicle_pos, pitch, yaw):
        return self.sensor.reqd_plane_intercept(vehicle_pos,
                                                self.sensor.rotated_camera_fov(
                                                    phi=pitch, psi=yaw))

    def get_sensor_measurements(self):
        '''
        Get sensor measurements from camera sensor
        '''
        detected_targets = []
        camera_projection = []
        for i in range(self.vehicle_num):
            camera_projection_i = self.get_ground_intersect(
                [self.vehicle[i].x, self.vehicle[i].y, self.vehicle[i].z], self.sensor_pitch,
                self.vehicle[i].psi)
            detected_targets_i = []
            for target in self.targets:
                if self.sensor.is_point_inside_camera_projection([target.x, target.y],
                                                                camera_projection_i):
                    range_to_target = np.linalg.norm(
                        np.array([target.x, target.y, 0]) - np.array(
                            [self.vehicle[i].x, self.vehicle[i].y, self.vehicle[i].z]))
                    # is_detected = self.sensor.get_detection(range_to_target)
                    detection_prob = np.random.random()
                    sensor_tpr = self.sensor.tpr(range_to_target)
                    if detection_prob < sensor_tpr:
                        target.is_detected = True
                        detected_targets_i.append(target)

            detected_targets.append(detected_targets_i)
            camera_projection.append(camera_projection_i)

        return detected_targets, camera_projection

    def traverse(self):
        '''
        Waypoint manager and vehicle state update- moves vehicle towards waypoints as long as waypoints exist in global_waypt_list
        '''
        for veh_wps in self.global_waypt_list:
            if not veh_wps or len(veh_wps.plan) == 0:
                pass
            else:
                i = veh_wps.vehicle_id
                next_position = np.array(
                    [veh_wps.plan[0].position.position.x,
                     veh_wps.plan[0].position.position.y,
                     veh_wps.plan[0].position.position.z])
                dist_to_waypt = np.linalg.norm(
                    [self.vehicle[i].x, self.vehicle[i].y, self.vehicle[i].z] - next_position)

                # update waypoint list if reached waypoint
                if dist_to_waypt < self.waypt_threshold:
                    # print("Reached waypoint -> ", next_position)
                    self.curr_waypt_num += 1
                    self.global_waypt_list[i].plan.pop(0)

                # else keep trying to navigate to next waypoint
                else:
                    omega, z_d = self.vehicle[i].go_to_goal(self.max_omega, self.max_zvel,
                                                        next_position, self.K_p,
                                                        self.K_p_z)
                    self.vehicle[i].psi += self.del_t * omega
                    self.vehicle[i].x += self.del_t * self.hvel * math.cos(self.vehicle[i].psi)
                    self.vehicle[i].y += self.del_t * self.hvel * math.sin(self.vehicle[i].psi)
                    self.vehicle[i].z += self.del_t * z_d

    def update_waypts(self, new_wpts):
        '''
        Receive new waypoints and send them to waypoint manager
        '''
        # self.global_waypt_list.append(new_wpts)
        self.global_waypt_list[new_wpts.vehicle_id] = new_wpts
        self.curr_waypt_num = 0

    def update_states(self):
        '''
        Updates the environment states
        '''
        self.traverse()

        # update the states for all ships in the environment
        for target in self.targets:
            target.propagate(self.del_t)

        for obstacle in self.obstacles:
            obstacle.update()

    def get_vehicle_uncertainty(self, id_num):
        return self.vehicle[id_num].position_uncertainty()

    # function that gets target heading and return heading with gaussian noise
    def get_target_heading_noise(self, heading):
        # gaussian noise model for now
        return heading + np.random.normal(0, 0.05)
