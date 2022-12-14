#!/usr/bin/env python

import os
import rospy
import numpy as np
import time


from rospkg import RosPack
from simple_drone_sim.msg import Plan, Waypoint, ObstaclePose, ObstacleArray, PoseStampedArray, OdometryArray
from simple_drone_sim.environment import *
from geometry_msgs.msg import PoseStamped, Point, Pose, Quaternion, PoseArray
from std_msgs.msg import ColorRGBA, Bool
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import UInt8
from simple_drone_sim.msg import TargetPoses, TargetPose, Detections, BatteryArray, Battery
from tf.transformations import quaternion_from_euler

from visualization_msgs.msg import Marker, MarkerArray

package = RosPack()
package_path = package.get_path("simple_drone_sim")

# https://sashamaps.net/docs/resources/20-colors/
COLORS = [[230, 25, 75],   [60, 180, 75],   [255, 225, 25], [0, 130, 200],
               [245, 130, 48],  [145, 30, 180],  [70, 240, 240], [240, 50, 230],
               [210, 245, 60],  [250, 190, 212], [0, 128, 128],  [220, 190, 255],
               [170, 110, 40],  [255, 250, 200], [128, 0, 0],    [170, 255, 195],
               [128, 128, 0],   [255, 215, 180], [0, 0, 128],    [128, 128, 128],
               [255, 255, 255], [0, 0, 0]]
def get_color(index):
    ros_color = ColorRGBA()
    ros_color.r = 0.0
    ros_color.g = 1.0
    ros_color.b = 0.0
    ros_color.a = 1.0
    return  ros_color

def get_color_target(index):
    ros_color = ColorRGBA()
    ros_color.r = 1.0
    ros_color.g = 0.0
    ros_color.b = 0.0
    ros_color.a = 1.0
    return  ros_color

class SimManager:
    def __init__(self):

        self.planner_path_topic = rospy.get_param("~planner_path")
        self.sim_env = self.env_setup()
        self.vehicle_traj_list = [[] for v in range(self.sim_env.vehicle_num)]
        
        self.map_size = 500.0 # meters
        self.map_resolution = 1.0  # meters
        self.coverage_size = 50.0 # meters
        self.covered_value = 50.0
        self.map_voxels = int(self.map_size / self.map_resolution)
        self.coverage_map = np.ones(self.map_voxels**2, dtype=int)

    def env_setup(self):
        # landing zones
        targets_list = rospy.get_param("/env_setup/targets", [])

        # obstacles
        obst_list = rospy.get_param("/env_setup/obstacles", [])

        # drone state
        init_x = rospy.get_param("/env_setup/init_x")
        init_y = rospy.get_param("/env_setup/init_y")
        init_z = rospy.get_param("/env_setup/init_z")
        init_psi = rospy.get_param("/env_setup/init_psi")

        max_omega = rospy.get_param("/env_setup/max_omega")
        max_zvel = rospy.get_param("/env_setup/max_zvel")

        vehicle_num = rospy.get_param("/env_setup/vehicle_num")
        vehicle_l = rospy.get_param("/env_setup/vehicle_l")

        hvel = rospy.get_param("/env_setup/hvel")
        vvel = rospy.get_param("/env_setup/vvel")

        n_rand_targets = rospy.get_param("/env_setup/n_rand_targets")
        n_rand_obst =  rospy.get_param("/env_setup/n_rand_obst")

        del_t = rospy.get_param("/env_setup/del_t")

        K_p = rospy.get_param("/env_setup/K_p")
        K_p_z = rospy.get_param("/env_setup/K_p_z")

        waypt_threshold = rospy.get_param("/env_setup/waypt_threshold")

        sensor_focal_length = rospy.get_param("/env_setup/sensor_focal_length")
        sensor_width = rospy.get_param("/env_setup/sensor_width")
        sensor_height = rospy.get_param("/env_setup/sensor_height")
        sensor_a = rospy.get_param("/env_setup/sensor_a")
        sensor_b = rospy.get_param("/env_setup/sensor_b")
        sensor_d = rospy.get_param("/env_setup/sensor_d")
        sensor_g = rospy.get_param("/env_setup/sensor_g")
        sensor_h = rospy.get_param("/env_setup/sensor_h")
        sensor_pitch = rospy.get_param("/env_setup/sensor_pitch")

        return Environment(targets_list,
                            max_omega,
                            max_zvel,
                            init_x,
                            init_y,
                            init_z,
                            init_psi,
                            K_p,
                            K_p_z,
                            vehicle_num,
                            vehicle_l,
                            hvel,
                            vvel,
                            n_rand_targets,
                            n_rand_obst,
                            del_t,
                            waypt_threshold,
                            sensor_focal_length,
                            sensor_width,
                            sensor_height,
                            sensor_a,
                            sensor_b,
                            sensor_d,
                            sensor_g,
                            sensor_h,
                            sensor_pitch,
                            obst_list)

    def get_vehicle_position(self, time, frame):
        v_poses = PoseStampedArray()
        v_poses.header.frame_id = frame
        v_poses.header.stamp = time

        poses_array = []
        for id_num in range(self.sim_env.vehicle_num):
            vehicle_pose = PoseStamped()
            vehicle_pose.header.frame_id = frame
            vehicle_pose.header.stamp = time
            # print self.sim_env.vehicle.x
            vehicle_pose.pose.position.x = self.sim_env.vehicle[id_num].x
            vehicle_pose.pose.position.y = self.sim_env.vehicle[id_num].y
            vehicle_pose.pose.position.z = self.sim_env.vehicle[id_num].z

            quat = quaternion_from_euler(0, 0, self.sim_env.vehicle[id_num].psi)
            vehicle_pose.pose.orientation.x = quat[0]
            vehicle_pose.pose.orientation.y = quat[1]
            vehicle_pose.pose.orientation.z = quat[2]
            vehicle_pose.pose.orientation.w = quat[3]

            poses_array.append(vehicle_pose)

        v_poses.poses = poses_array
        return v_poses
    
    def get_coverage_grid(self, time, frame):
        for id_num in range(self.sim_env.vehicle_num):
            x = self.sim_env.vehicle[id_num].x
            y = self.sim_env.vehicle[id_num].y
            start_y = int((y - self.coverage_size / 2. + self.map_size / 2)  / self.map_resolution)
            end_y = int((y + self.coverage_size / 2. + self.map_size / 2)  / self.map_resolution)
            i = start_y
            while i <= end_y:
                start_x = int((x - self.coverage_size / 2. + self.map_size / 2) / self.map_resolution)
                end_x = int((x + self.coverage_size / 2. + self.map_size / 2) / self.map_resolution)

                start = i * self.map_voxels + start_x
                end = i * self.map_voxels + end_x
                self.coverage_map[start:end][self.coverage_map[start:end] <= self.covered_value] = self.covered_value
                i = i + 1

        grid = OccupancyGrid()
        grid.header.frame_id = frame
        grid.header.stamp = time
        grid.info.height = self.map_voxels
        grid.info.width = self.map_voxels
        grid.info.resolution = self.map_resolution
        grid.info.origin.position.x = -self.map_size / 2
        grid.info.origin.position.y = -self.map_size / 2
        grid.data = self.coverage_map.tolist()

        return grid


    def get_obstacle_positions(self, time, frame):
        obstacles = ObstacleArray()
        obstacles.header.frame_id = frame
        obstacles.header.stamp = time

        for obst in self.sim_env.obstacles:
            obst_pose = ObstaclePose()
            obst_pose.id = obst.id
            obst_pose.x = obst.x
            obst_pose.y = obst.y
            obst_pose.width = obst.width
            obst_pose.length = obst.length
            obst_pose.height = obst.height
            obstacles.obstacles.append(obst_pose)

        return obstacles

    def get_occupancy_grid(self, time, frame):
        for obst in self.sim_env.obstacles:
            start_y = int((obst.y - obst.width + self.map_size / 2)  / self.map_resolution)
            end_y = int((obst.y + obst.width + self.map_size / 2)  / self.map_resolution)
            i = start_y
            while i <= end_y:
                start_x = int((obst.x - obst.length + self.map_size / 2) / self.map_resolution)
                end_x = int((obst.x + obst.length + self.map_size / 2) / self.map_resolution)

                start = i * self.map_voxels + start_x
                end = i * self.map_voxels + end_x
                self.coverage_map[start:end] = obst.height
                i = i + 1

        grid = OccupancyGrid()
        grid.header.frame_id = frame
        grid.header.stamp = time
        grid.info.height = self.map_voxels
        grid.info.width = self.map_voxels
        grid.info.resolution = self.map_resolution
        grid.info.origin.position.x = -self.map_size / 2
        grid.info.origin.position.y = -self.map_size / 2
        grid.data = self.coverage_map.tolist()

        return grid

    def get_target_positions(self, time, frame):
        targets_pose = TargetPoses()
        targets_pose.header.frame_id = frame
        targets_pose.header.stamp = time

        for target in self.sim_env.targets:
            target_pose = TargetPose()

            target_pose.id = target.id
            target_pose.x = target.x
            target_pose.y = target.y
            target_pose.heading = target.heading
            target_pose.linear_speed = target.linear_speed
            target_pose.angular_speed = target.angular_speed
            target_pose.is_detected = target.is_detected

            # print target_pose
            targets_pose.targets.append(target_pose)

        return targets_pose

    def get_camera_pose(self, time, frame):
        odom_array = OdometryArray()

        cam_pose_list = []
        for id_num in range(self.sim_env.vehicle_num):
            camera_pose = Odometry()
            camera_pose.header.frame_id = frame
            camera_pose.header.stamp = time

            camera_pose.pose.pose.position.x = self.sim_env.vehicle[id_num].x
            camera_pose.pose.pose.position.y = self.sim_env.vehicle[id_num].y
            camera_pose.pose.pose.position.z = self.sim_env.vehicle[id_num].z

            quat = quaternion_from_euler(0, self.sim_env.sensor_pitch, self.sim_env.vehicle[id_num].psi)
            camera_pose.pose.pose.orientation.x = quat[0]
            camera_pose.pose.pose.orientation.y = quat[1]
            camera_pose.pose.pose.orientation.z = quat[2]
            camera_pose.pose.pose.orientation.w = quat[3]

            cov_matrix_i = np.zeros((6,6))
            cov_matrix_i[0,0] = self.sim_env.get_vehicle_uncertainty(id_num)[0]
            cov_matrix_i[1,1] = self.sim_env.get_vehicle_uncertainty(id_num)[1]
            cov_matrix_i[2,2] = self.sim_env.get_vehicle_uncertainty(id_num)[2]
            cov_matrix_i[3,3] = self.sim_env.get_vehicle_uncertainty(id_num)[3]
            cov_matrix_i[4,4] = self.sim_env.get_vehicle_uncertainty(id_num)[4]
            cov_matrix_i[5,5] = self.sim_env.get_vehicle_uncertainty(id_num)[5]

            camera_pose.pose.covariance = cov_matrix_i.flatten().tolist()
            cam_pose_list.append(camera_pose)

        odom_array.odom_array = cam_pose_list

        return odom_array

    def get_waypt_num(self):
        waypt_number = UInt8()
        waypt_number.data = self.sim_env.curr_waypt_num
        return waypt_number

    def get_target_detections(self, time, frame):
        detection_msg_list = []
        detection_msg = Detections()
        detection_msg.header.frame_id = frame
        detection_msg.header.stamp = time

        detected_targets, camera_projection = self.sim_env.get_sensor_measurements()

        for i in range(self.sim_env.vehicle_num):
            for target in detected_targets[i]:
                detection_msg.headings.append(self.sim_env.get_target_heading_noise(target.heading))

                target_camera_unit_vector = Point()

                range_to_target = np.linalg.norm(np.array([target.x, target.y, 0]) - np.array([self.sim_env.vehicle[i].x,
                                                                                self.sim_env.vehicle[i].y, self.sim_env.vehicle[i].z]))

                i_hat = (target.x - self.sim_env.vehicle[i].x) / range_to_target
                j_hat = (target.y - self.sim_env.vehicle[i].y) / range_to_target
                k_hat = - self.sim_env.vehicle[i].z / range_to_target

                # R = np.matmul(self.sim_env.sensor.Rz(self.sim_env.vehicle.phi), self.sim_env.sensor.Ry(self.sim_env.sensor_pitch))
                # print("current pitch is", self.sim_env.sensor_pitch)
                # print("current psi is", self.sim_env.vehicle.psi)
                R = np.matmul(self.sim_env.sensor.Rz(self.sim_env.vehicle[i].psi),self.sim_env.sensor.Ry(self.sim_env.sensor_pitch))
                R_inv = np.linalg.inv(R)
                camera_frame_pose = np.matmul(R_inv, [i_hat, j_hat, k_hat])

                target_camera_unit_vector.x = camera_frame_pose[0]
                target_camera_unit_vector.y = camera_frame_pose[1]
                target_camera_unit_vector.z = camera_frame_pose[2]
                detection_msg.target_camera_vectors.append(target_camera_unit_vector)

                detection_msg.target_ids.append(target.id)
                detection_msg_list.append(detection_msg)

        return detection_msg, camera_projection

    def planner_callback(self, msg):
        self.sim_env.update_waypts(msg)

    def command_land_callback(self, msg):
        self.sim_env.vehicle[msg.data].in_flight_cond = False
        rospy.loginfo_once("SimManager: Landing")

    def command_takeoff_callback(self, msg):
        self.sim_env.vehicle[msg.data].in_flight_cond = True
        plan = Plan()
        plan.header.frame_id =  "local_enu"
        plan.header.stamp = rospy.Time.now()
        plan.vehicle_id = msg.data
        waypoint_land = Waypoint()
        waypoint_land.position.position.x = self.sim_env.vehicle[plan.vehicle_id].x
        waypoint_land.position.position.y = self.sim_env.vehicle[plan.vehicle_id].y
        waypoint_land.position.position.z = self.sim_env.init_z
        plan.plan.append(waypoint_land)
        self.sim_env.update_waypts(plan)
        print("Taking off")

    def command_reduce_battery_callback(self, msg):
        print("Reducing battery")
        if msg.data >= 0 and msg.data <= 100:
            for i in range(self.sim_env.vehicle_num):
                self.sim_env.vehicle[i].reduce_battery(msg.data, rospy.Time.now())

    def get_obstacle_markers(self, time, frame):
        obstacles_marker_array = MarkerArray()

        for idx, obstacle in enumerate(self.sim_env.obstacles):
            obstacle_marker = Marker()
            obstacle_marker.header.frame_id = frame
            obstacle_marker.header.stamp = time
            obstacle_marker.ns = "obstacle_pose"
            obstacle_marker.id = idx
            obstacle_marker.type = Marker.CUBE
            obstacle_marker.action = Marker.ADD
            obstacle_marker.lifetime = rospy.Duration()
            quat = quaternion_from_euler(0, 0, 0)
            obstacle_marker.pose = Pose(Point(obstacle.x,
                                            obstacle.y,
                                            obstacle.height / 2.0),
                                            Quaternion(quat[0], quat[1], quat[2], quat[3]))

            obstacle_marker.color = get_color(obstacle.id)
            obstacle_marker.color.a = 0.8
            obstacle_marker.scale.x = obstacle.length
            obstacle_marker.scale.y = obstacle.width
            obstacle_marker.scale.z = obstacle.height
            obstacles_marker_array.markers.append(obstacle_marker)

        return obstacles_marker_array


    def get_vehicle_marker(self, time, frame, vehicle_pose):
        veh_markers = MarkerArray()

        for veh in self.sim_env.vehicle:
            vehicle_marker = Marker()
            vehicle_marker.header.frame_id = frame
            vehicle_marker.header.stamp = time
            vehicle_marker.ns = "vehicle_pose"
            vehicle_marker.id = veh.id_num
            vehicle_marker.type = Marker.MESH_RESOURCE
            vehicle_marker.action = Marker.ADD
            vehicle_marker.lifetime = rospy.Duration()
            vehicle_marker.color.r = 0
            vehicle_marker.color.g = 0
            vehicle_marker.color.b = 1
            vehicle_marker.color.a = 1
            vehicle_marker.scale.x = 5
            vehicle_marker.scale.y = 5
            vehicle_marker.scale.z = 5
            vehicle_marker.mesh_use_embedded_materials = True
            vehicle_marker.mesh_resource = "package://simple_drone_sim/meshes/drone.dae"
            vehicle_marker.pose.position.x = veh.x
            vehicle_marker.pose.position.y = veh.y
            vehicle_marker.pose.position.z = veh.z

            quat = quaternion_from_euler(1.57, 0, veh.psi)
            vehicle_marker.pose.orientation.x = quat[0]
            vehicle_marker.pose.orientation.y = quat[1]
            vehicle_marker.pose.orientation.z = quat[2]
            vehicle_marker.pose.orientation.w = quat[3]

            veh_markers.markers.append(vehicle_marker)

        return veh_markers

    def get_vehicle_trajectory_marker(self, time, frame, vehicle_pose):
        markers = MarkerArray()

        for veh in self.sim_env.vehicle:
            trajectory_marker = Marker()
            trajectory_marker.header.frame_id = frame
            trajectory_marker.header.stamp = time
            trajectory_marker.ns = "vehicle_trajectory"
            trajectory_marker.id = veh.id_num
            trajectory_marker.type = Marker.LINE_STRIP
            trajectory_marker.action = Marker.ADD
            trajectory_marker.lifetime = rospy.Duration()

            self.vehicle_traj_list[veh.id_num].append([veh.x, veh.y, veh.z])

            # setting traj length to 100
            if len(self.vehicle_traj_list[veh.id_num]) > 1000:
                self.vehicle_traj_list[veh.id_num].pop(0)

            trajectory_marker.pose.position.x = 0
            trajectory_marker.pose.position.y = 0
            trajectory_marker.pose.position.z = 0

            for p in self.vehicle_traj_list[veh.id_num]:
                trajectory_marker.points.append(Point(p[0], p[1], p[2]))

            trajectory_marker.color.r = 1
            trajectory_marker.color.g = 69/255
            trajectory_marker.color.b = 0
            trajectory_marker.color.a = 1
            trajectory_marker.scale.x = 1
            trajectory_marker.scale.y = 1
            trajectory_marker.scale.z = 1

            markers.markers.append(trajectory_marker)

        return markers

    """
    Four points of the sensor footprint polygon.
    """
    def get_projection_points_marker(self, time, frame, vehicle_pose, camera_projection):
        markers = MarkerArray()

        for i in range(self.sim_env.vehicle_num):
            projection_points_marker = Marker()
            projection_points_marker.header.frame_id = frame
            projection_points_marker.header.stamp = time
            projection_points_marker.ns = "projection_marker"
            projection_points_marker.id = i
            projection_points_marker.type = Marker.POINTS
            projection_points_marker.action = Marker.ADD
            projection_points_marker.color.r = 1
            projection_points_marker.color.g = 1
            projection_points_marker.color.b = 0
            projection_points_marker.color.a = 1
            projection_points_marker.scale.x = 1
            projection_points_marker.scale.y = 1
            projection_points_marker.scale.z = 1
            points = []
            for np_point in camera_projection[i]:
                ros_point = Point()
                ros_point.x = np_point[0]
                ros_point.y = np_point[1]
                ros_point.z = np_point[2]
                points.append(ros_point)
            projection_points_marker.points = points
            markers.markers.append(projection_points_marker)
        return markers

    def get_projection_marker(self, time, frame, vehicle_pose, camera_projection):
        marker_list = MarkerArray()
        proj_marker_list = []

        for vehicle_id in range(self.sim_env.vehicle_num):
            projection_marker = Marker()
            projection_marker.header.frame_id = frame
            projection_marker.header.stamp = time
            projection_marker.ns = "projection_marker"
            projection_marker.id = vehicle_id
            projection_marker.type = Marker.LINE_STRIP
            projection_marker.action = Marker.ADD
            # projection_marker.lifetime = rospy.Duration()
            projection_marker.color.r = 1
            projection_marker.color.g = 69/255
            projection_marker.color.b = 0
            projection_marker.color.a = 1
            projection_marker.scale.x = 1
            projection_marker.scale.y = 1
            projection_marker.scale.z = 1

            points = []

            vehicle_point = Point()
            vehicle_point.x = vehicle_pose.poses[vehicle_id].pose.position.x
            vehicle_point.y = vehicle_pose.poses[vehicle_id].pose.position.y
            vehicle_point.z = vehicle_pose.poses[vehicle_id].pose.position.z

            # connect the projected camera bounds
            for edge in range(len(camera_projection[vehicle_id])):
                point_a = Point()
                point_a.x = camera_projection[vehicle_id][edge][0]
                point_a.y = camera_projection[vehicle_id][edge][1]
                point_a.z = camera_projection[vehicle_id][edge][2]

                point_b = Point()
                point_b.x = camera_projection[vehicle_id][(edge + 1) % len(camera_projection[vehicle_id])][0]
                point_b.y = camera_projection[vehicle_id][(edge + 1) % len(camera_projection[vehicle_id])][1]
                point_b.z = camera_projection[vehicle_id][(edge + 1) % len(camera_projection[vehicle_id])][2]

                points.append(point_b)
                points.append(point_a)
                points.append(vehicle_point)
                points.append(point_b)
                points.append(point_a)

            projection_marker.points = points
            proj_marker_list.append(projection_marker)

        marker_list.markers = proj_marker_list
        return marker_list

    def get_targets_marker(self, time, frame, target_positions):
        targets_marker_array = MarkerArray()

        for idx, target in enumerate(target_positions.targets):
            target_marker = Marker()
            target_marker.header.frame_id = frame
            target_marker.header.stamp = time
            target_marker.ns = "target_pose"
            target_marker.id = idx
            target_marker.type = Marker.CYLINDER
            target_marker.action = Marker.ADD
            # target_marker.mesh_use_embedded_materials = True
            # target_marker.mesh_resource = "package://simple_drone_sim/meshes/boat.dae"
            target_marker.lifetime = rospy.Duration()
            quat = quaternion_from_euler(0, 0, target.heading)
            target_marker.pose = Pose(Point(target.x,
                                            target.y,
                                            0),  # z offset to make it appear above grid-map
                                            Quaternion(quat[0], quat[1], quat[2], quat[3]))

            target_marker.color = get_color_target(target.id)
            target_marker.scale.x = 10
            target_marker.scale.y = 10
            target_marker.scale.z = .1
            targets_marker_array.markers.append(target_marker)

        return targets_marker_array

    def get_vehicle_battery(self, time):
        battery_array = BatteryArray()

        battery_status_all = []
        for id_num in range(self.sim_env.vehicle_num):
            battery = Battery()
            battery.percent = self.sim_env.vehicle[id_num].update_battery(time)
            battery.vehicle_id = id_num 

            battery_array.vehicles.append(battery)

        return battery_array


    def main(self):
        waypt_num_pub = rospy.Publisher('/drone_sim/waypt_num', UInt8, queue_size=10)
        vehicle_pose_pub = rospy.Publisher('/drone_sim/vehicle_poses', PoseStampedArray, queue_size=10)
        target_pose_pub = rospy.Publisher('/drone_sim/target_poses', TargetPoses, queue_size=10)
        sensor_detections_pub = rospy.Publisher('/drone_sim/sensor_measurement', Detections, queue_size=10)
        camera_pose_pub = rospy.Publisher('/drone_sim/camera_pose', OdometryArray, queue_size=10)
        obstacle_pose_pub = rospy.Publisher('/drone_sim/obstacles', ObstacleArray, queue_size=10)
        occ_grid_pub = rospy.Publisher('/drone_sim/occupancy_grid', OccupancyGrid, queue_size=10, latch=True)
        coverage_grid_pub = rospy.Publisher('/drone_sim/coverage_grid', OccupancyGrid, queue_size=10)
        vehicle_in_collision_pub = rospy.Publisher('/drone_sim/collision_detected', Bool, queue_size=10)
        vehicle_battery_pub = rospy.Publisher('/drone_sim/vehicle_battery', BatteryArray, queue_size=10)

        # Marker Publishers
        vehicle_marker_pub = rospy.Publisher('/drone_sim/markers/vehicle_pose', MarkerArray, queue_size=10)
        projection_marker_pub = rospy.Publisher('/drone_sim/markers/camera_projection', MarkerArray, queue_size=10)
        projection_points_marker_pub = rospy.Publisher('/drone_sim/markers/camera_projection_points', MarkerArray, queue_size=10)
        targets_marker_pub = rospy.Publisher('/drone_sim/markers/targets', MarkerArray, queue_size=10)
        vehicle_trajectory_pub = rospy.Publisher('/drone_sim/markers/vehicle_trajectory', MarkerArray, queue_size=10)
        obstacles_marker_pub = rospy.Publisher('/drone_sim/markers/obstacles', MarkerArray, queue_size=10)

        waypt_sub = rospy.Subscriber(self.planner_path_topic, Plan, self.planner_callback)
        landing_sub = rospy.Subscriber('/drone_sim/command_land', UInt8, self.command_land_callback)
        takeoff_sub = rospy.Subscriber('/drone_sim/command_takeoff', UInt8, self.command_takeoff_callback)
        reduce_battery_sub = rospy.Subscriber('/drone_sim/command_reduce_battery', UInt8, self.command_reduce_battery_callback)
        rate = rospy.Rate(1/self.sim_env.del_t)
        counter = 0
        start_time = rospy.Time.now()
        time_since_last_write = start_time

        collision_detected = False

        published_grid = False

        while not rospy.is_shutdown():
            time = rospy.Time.now()
            frame = "local_enu"
            vehicle_position = self.get_vehicle_position(time, frame)
            target_positions = self.get_target_positions(time, frame)
            target_detections, camera_projection = self.get_target_detections(time, frame)
            camera_pose = self.get_camera_pose(time, frame)
            waypoint_number  = self.get_waypt_num()

            waypt_num_pub.publish(waypoint_number)
            vehicle_pose_pub.publish(vehicle_position)
            target_pose_pub.publish(target_positions)
            sensor_detections_pub.publish(target_detections)
            camera_pose_pub.publish(camera_pose)
            obstacle_pose_pub.publish(self.get_obstacle_positions(time, frame))
            coverage_grid_pub.publish(self.get_coverage_grid(time, frame))
            if not published_grid:
                occ_grid_pub.publish(self.get_occupancy_grid(time, frame))
                published_grid = True

            # publish battery_pub
            vehicle_battery_pub.publish(self.get_vehicle_battery(time))
            vehicle_marker_pub.publish(self.get_vehicle_marker(time, frame, vehicle_position))
            projection_marker_pub.publish(self.get_projection_marker(time, frame, vehicle_position, camera_projection))
            projection_points_marker_pub.publish(self.get_projection_points_marker(time, frame, vehicle_position, camera_projection))
            targets_marker_pub.publish(self.get_targets_marker(time, frame, target_positions))
            obstacles_marker_pub.publish(self.get_obstacle_markers(time, frame))
            if counter % 10 == 0:
                vehicle_trajectory_pub.publish(self.get_vehicle_trajectory_marker(time, frame, vehicle_position))

            counter += 1
            # if counter == 100:
            #     # Currently doing state update every 100 iters
            self.sim_env.update_states()

            if self.sim_env.is_in_collision():
                collision_detected = True

            msg_collision = Bool()
            msg_collision.data = collision_detected
            vehicle_in_collision_pub.publish(msg_collision)

                # counter = 0
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node("sim_manager_node", anonymous=True)
    obj = SimManager()
    obj.main()


