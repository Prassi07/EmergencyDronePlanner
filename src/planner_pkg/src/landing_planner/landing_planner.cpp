#include "../../include/landing_planner/landing_planner.h"

LandingPlanner::LandingPlanner(){
    obstacle_cost = 100;
}
LandingPlanner::~LandingPlanner(){}

void LandingPlanner::setBattery(float batt){
    start_battery = batt;
}

inline int LandingPlanner::computeKey(int x, int y){
    return (y*y_size + x);
}

void LandingPlanner::setTargets(const simple_drone_sim::TargetPoses& targets){
    for(const simple_drone_sim::TargetPose &target : targets.targets){
        Node t1(target.x, target.y);
        t1.key = computeKey(t1.x, t1.y);
        goal_locations.push_back(t1);
    }
}

void LandingPlanner::setRobotLocation(const geometry_msgs::PoseStamped& pose){
    start_node.x = pose.pose.position.x;
    start_node.y = pose.pose.position.y;

    robot_z = pose.pose.position.z;
}

void LandingPlanner::updateMap(const nav_msgs::OccupancyGrid::ConstPtr& grid){

    map_resolution = grid->info.resolution;
    y_size = grid->info.width;
    x_size = grid->info.height;
    x_offset = (grid->info.origin.position.x)/(map_resolution);
    y_offset = (grid->info.origin.position.y)/(map_resolution);

    map.clear();
    map = grid->data;

}

void LandingPlanner::printInfo(){
    ROS_INFO_STREAM(" Map size: " << map.size());
    ROS_INFO_STREAM("Robot Position: " <<  start_node.x << " " << start_node.y);
    ROS_INFO_STREAM("Number of Landing Zones: " << goal_locations.size());
    ROS_INFO_STREAM("Robot Battery Info: " << start_battery);

    ROS_INFO_STREAM("x_offset: "<< x_offset <<" y_offset: " <<y_offset);
}

void LandingPlanner::updateCells(){
    for(Node &n: goal_locations){
        n.x = (n.x)/(map_resolution) - x_offset;
        n.y = (n.y)/(map_resolution) - y_offset;
    }

    start_node.x = start_node.x/map_resolution - x_offset;
    start_node.y = start_node.y/map_resolution - y_offset;
}

int LandingPlanner::planToGoals(simple_drone_sim::Plan& plan){
    updateCells();

    //Setup start node, closed and open lists.
    CLOSED_LIST closed_list;
    OPEN_LIST open_list;

    Node* start = new Node(start_node.x, start_node.y);
    start->key = computeKey(start_node.x, start_node.y);
    start->g = 0;
    start->h = 0;
    start->f = start->g;
    start->parent = NULL;

    Node goalNode = goal_locations[0];
    int goal_key;
    open_list.push(start);
    ROS_INFO("Setup A star");
    bool pathFound = false;
    while(!open_list.empty() && closed_list.count(computeKey(goalNode.x, goalNode.y)) == 0 ){
        
        Node *curr_node = open_list.top();
        open_list.pop();
        int curr_key = computeKey(curr_node->x, curr_node->y);
        // ROS_INFO("Here1");
        if(closed_list.count(curr_key) == 0 ){
            closed_list.insert({curr_key, curr_node});

            if(*curr_node == goalNode){
                pathFound = true;
                goal_key = curr_key;
                ROS_INFO("Found Path");
                break; //Exit if we seem to reach the goal node
            }

            for(int dir = 0; dir < 8; dir++)
            {
                int newx = curr_node->x + dX[dir];
                int newy = curr_node->y + dY[dir];
                if(closed_list.count(computeKey(newx, newy)) == 0 ){
                    if (newx >= 0 && newx < x_size && newy >= 0 && newy < y_size)
                    {
                        if (((int)map[computeKey(newx, newy)] >= 0) && ((int)map[computeKey(newx,newy)] < obstacle_cost))  //if free
                        {   
                            int g_s_dash = curr_node->g + 1;
                            int h_s_dash = 1;
                            Node *successor = new Node(newx, newy);
                            successor->parent = curr_node;
                            successor->g = g_s_dash;
                            successor->h = h_s_dash;
                            successor->f = g_s_dash;
                            open_list.push(successor);
                        }
                    }
                }
            }
        }
            
    }

    int pathLength = 0;
    if(pathFound){
        ROS_INFO("Backtracking..");
        Node* backtrackNode = closed_list.at(goal_key);
        while(backtrackNode->parent != NULL){
            simple_drone_sim::Waypoint wp;
            wp.position.position.x = (backtrackNode->x + x_offset)*map_resolution;
            wp.position.position.y = (backtrackNode->y + y_offset)*map_resolution;
            wp.position.position.z = robot_z;
            plan.plan.push_back(wp);
            backtrackNode = backtrackNode->parent;
            pathLength++;
        }
    }

    return pathLength;

}