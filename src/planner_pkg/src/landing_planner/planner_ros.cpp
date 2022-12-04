#include "planner_pkg/planner_ros.h"

LandingPlannerNode::LandingPlannerNode(){
    ros_rate = 10;
}

void LandingPlannerNode::Run(){

    // Create ROS Subs
    occupancy_grid_sub = nh.subscribe("/drone_sim/occupancy_grid", 1, &LandingPlannerNode::OccupancyGridHandler, this);
    odom_sub = nh.subscribe("/drone_sim/vehicle_poses", 1, &LandingPlannerNode::OdometryHandler, this);


    ros::spinOnce(); 
    ros::Rate rate(ros_rate);

    while(ros::ok()){

        
        ros::spinOnce();
        rate.sleep();
    }

    

}

void LandingPlannerNode::OccupancyGridHandler(const nav_msgs::OccupancyGrid::ConstPtr& msg){

    
}


void LandingPlannerNode::OdometryHandler(const simple_drone_sim::PoseStampedArray::ConstPtr& msg){
 
}

int main(int argc, char** argv) {

    ROS_INFO("Starting Landing Planner Node ");
    ros::init(argc, argv, "Landing Planner Node");
    LandingPlannerNode node;
    node.Run();
    return 0;
}