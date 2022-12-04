#ifndef _PLANNER_ROSN_H_
#define _PLANNER_ROS_H_

#include <ros/ros.h>


#include <sensor_msgs/PointCloud2.h>

#include <geometry_msgs/PoseStamped.h>


#include <nav_msgs/Odometry.h>
#include <std_msgs/Header.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>

#include <planner_pkg/landing_planner.h>

#include "simple_drone_sim/PoseStampedArray.h"

#include <string>
#include <iostream> 
#include <algorithm>
#include <vector>
#include <stdlib.h>

class LandingPlannerNode{
    public:
        LandingPlannerNode();
        void Run();

    private:

        // ROS Node Handlers
        ros::NodeHandle nh;
        ros::NodeHandle p_nh = ros::NodeHandle("~");
        
        // ROS Subscribers
        ros::Subscriber occupancy_grid_sub;
        ros::Subscriber target_locations_sub;

        ros::Subscriber battery_sub;
        ros::Subscriber odom_sub;
    
        ros::Subscriber enable_staircase_sub;

        // ROS Publishers
        ros::Publisher plan_publisher;

        void OdometryHandler(const simple_drone_sim::PoseStampedArray::ConstPtr& msg);
        void OccupancyGridHandler(const nav_msgs::OccupancyGrid::ConstPtr& msg);

        float ros_rate;

};
#endif
