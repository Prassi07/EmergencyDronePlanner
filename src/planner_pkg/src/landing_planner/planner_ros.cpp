#include "../../include/landing_planner/planner_ros.h"

LandingPlannerNode::LandingPlannerNode(){
    ros_rate = 10;
}

void LandingPlannerNode::Run(){

    // Create ROS Subs
    occupancy_grid_sub = nh.subscribe("/drone_sim/occupancy_grid", 1, &LandingPlannerNode::OccupancyGridHandler, this);
    odom_sub = nh.subscribe("/drone_sim/vehicle_poses", 1, &LandingPlannerNode::OdometryHandler, this);
    battery_sub = nh.subscribe("/drone_sim/vehicle_battery",1, &LandingPlannerNode::BatteryHandler, this);
    target_locations_sub = nh.subscribe("/drone_sim/target_poses", 1, &LandingPlannerNode::TargetHandler, this);
    
    ros::spinOnce(); 
    ros::Rate rate(ros_rate);

    while(ros::ok()){

        
        ros::spinOnce();
        rate.sleep();
    }

    

}

void LandingPlannerNode::BatteryHandler(const simple_drone_sim::BatteryArray::ConstPtr& msg){

}
void LandingPlannerNode::OccupancyGridHandler(const nav_msgs::OccupancyGrid::ConstPtr& msg){

    
}

 void LandingPlannerNode::TargetHandler(const simple_drone_sim::TargetPoses::ConstPtr& msg){

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