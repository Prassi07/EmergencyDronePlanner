#include "../../include/landing_planner/planner_ros.h"

LandingPlannerNode::LandingPlannerNode(){

    initialized_map = false;
    init_battery = false;
    init_robot_pose = false;
    init_targets = false;
    ros_rate = 10;

    planner = LandingPlanner();
}


LandingPlannerNode::~LandingPlannerNode(){}


void LandingPlannerNode::Run(){

    // Create ROS Subs
    occupancy_grid_sub = nh.subscribe("/drone_sim/coverage_grid", 1, &LandingPlannerNode::OccupancyGridHandler, this);
    odom_sub = nh.subscribe("/drone_sim/vehicle_poses", 1, &LandingPlannerNode::OdometryHandler, this);
    battery_sub = nh.subscribe("/drone_sim/vehicle_battery",1, &LandingPlannerNode::BatteryHandler, this);
    target_locations_sub = nh.subscribe("/drone_sim/target_poses", 1, &LandingPlannerNode::TargetHandler, this);

    plan_publisher = nh.advertise<simple_drone_sim::Plan>("/planning/landing_zones", 1, true);
    ros::spinOnce(); 
    ros::Rate rate(ros_rate);

    ROS_WARN("Here!");
    while(ros::ok()){
        if(init_battery && initialized_map && init_targets && init_robot_pose){
            simple_drone_sim::Plan plan;
            plan.vehicle_id = 0;
            plan.header.frame_id = "local_enu";
            plan.header.stamp = ros::Time::now();
            int planLength = planner.planToGoals(plan);
            if(planLength > 0){
                ROS_INFO("Publishing plan.");
                plan_publisher.publish(plan);
            }
        }
        ros::spinOnce();
        rate.sleep();
    }

}

void LandingPlannerNode::BatteryHandler(const simple_drone_sim::BatteryArray::ConstPtr& msg){

    if(!init_battery){
        ROS_INFO("Updating battery for planner");
        float battery = msg->vehicles[0].percent;   
        planner.setBattery(battery); 
        init_battery = true;
    }
}

void LandingPlannerNode::OccupancyGridHandler(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    if(!initialized_map){
        ROS_INFO("Updating map for planner");
        planner.updateMap(msg);
        initialized_map = true;
    }
}

 void LandingPlannerNode::TargetHandler(const simple_drone_sim::TargetPoses::ConstPtr& msg){
    if(!init_targets){
        ROS_INFO("Updating targets for planner");
        planner.setTargets(*msg);
        init_targets = true;
    }
}

void LandingPlannerNode::OdometryHandler(const simple_drone_sim::PoseStampedArray::ConstPtr& msg){
    if(!init_robot_pose){
        ROS_INFO("Updating robot pose for planner");
        planner.setRobotLocation(msg->poses[0]);
        init_robot_pose = true;
    }
}

int main(int argc, char** argv) {

    ROS_INFO("Starting Landing Planner Node ");
    ros::init(argc, argv, "Landing Planner Node");
    LandingPlannerNode node;
    node.Run();
    return 0;
}