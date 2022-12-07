#include "../../include/landing_planner/landing_planner.h"

using namespace std;

LandingPlanner::LandingPlanner(){
    obstacle_cost = 100;
    goal_updated = false;
}
LandingPlanner::~LandingPlanner(){}

void LandingPlanner::setBattery(float batt){
    start_battery = batt;
    time_remaining = 2.4 * start_battery;
    max_steps = floor(time_remaining/0.12);
}

inline int LandingPlanner::computeKey(int x, int y){
    return (y*y_size + x);
}

inline int LandingPlanner::getMapIndex(int x, int y){
    return (y*y_size + x);
}


int LandingPlanner::estimateOctileDistance(int curr_x, int curr_y, int goal_x, int goal_y){
    int delta_x = abs(curr_x - goal_x);
    int delta_y = abs(curr_y - goal_y);
    return delta_x + delta_y - MIN(delta_x,delta_y);
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

void LandingPlanner::updateGoalCells(){
    if(!goal_updated){
        for(Node &n: goal_locations){
            n.x = (n.x)/(map_resolution) - x_offset;
            n.y = (n.y)/(map_resolution) - y_offset;
        }
    goal_updated = true;
    }
}

void LandingPlanner::updateStart(){
    start_node.x = start_node.x/map_resolution - x_offset;
    start_node.y = start_node.y/map_resolution - y_offset;
}

int LandingPlanner::planToGoals(simple_drone_sim::Plan& plan){
    updateGoalCells();
    updateStart();

    //Setup start node, closed and open lists.
    CLOSED_LIST closed_list;
    OPEN_LIST open_list;

    Node* start = new Node(start_node.x, start_node.y);
    start->key = computeKey(start_node.x, start_node.y);
    start->g = 0;
    start->h = 0;
    start->f = start->g;
    start->parent = NULL;
    start->time = max_steps;
    
    int current_iter = 0;

    Node goalNode = goal_locations[current_iter]; // original
    // int goal_key; // original
    vector<int> goal_keys;
    
    open_list.push(start);
    // ROS_INFO("Setup A star");
    bool pathFound = false;
    while(!open_list.empty() && closed_list.count(computeKey(goalNode.x, goalNode.y)) == 0 ){
        Node *curr_node = open_list.top();
        open_list.pop();
        int curr_key = computeKey(curr_node->x, curr_node->y);
        // ROS_INFO("Here1");
        if(closed_list.count(curr_key) == 0 ){

            closed_list.insert({curr_key, curr_node});
            
            if(*curr_node == goalNode){

                goal_keys.push_back(curr_key);
                current_iter++;
                pathFound = true;
                ROS_INFO("Found Path, Est Battery Time remaining: %d", curr_node->time);
                
                for (int i = current_iter; i < goal_locations.size(); i++) {
                    int key = computeKey(goal_locations[i].x,goal_locations[i].y);
                    if (closed_list.count(key) == 0) {
                        goalNode = goal_locations[i];
                        break;
                    }
                    else { 
                        goal_keys.push_back(key);
                    }
                }
            }

            for(int dir = 0; dir < 8; dir++)
            {
                int newx = curr_node->x + dX[dir];
                int newy = curr_node->y + dY[dir];
                int newt = curr_node->time - 1;

                bool expand = false;
                if(closed_list.count(computeKey(newx, newy)) == 0)
                    expand = true;
                else{
                    if(closed_list.at(computeKey(newx, newy))->time < newt)
                        expand = true;
                }

                if(expand){
                    if (newx >= 0 && newx < x_size && newy >= 0 && newy < y_size && newt > 50)
                    {
                        if (((int)map[getMapIndex(newx, newy)] >= 0) && ((int)map[getMapIndex(newx,newy)] < obstacle_cost))  //if free
                        {   
                            int g_s_dash = curr_node->g + (int)map[getMapIndex(newx, newy)];
                            int h_s_dash = estimateOctileDistance(newx, newy, goalNode.x, goalNode.y);
                            Node *successor = new Node(newx, newy);
                            successor->parent = curr_node;
                            successor->g = g_s_dash;
                            successor->h = h_s_dash;
                            successor->f = g_s_dash + h_s_dash;
                            successor->time = newt;
                            open_list.push(successor);
                        }
                    }
                }
            }
        }
            
    }

    int pathLength = 0;
    vector<simple_drone_sim::Waypoint> curr_path, best_path;
    if(pathFound){
        ROS_INFO("Backtracking..");
        for(int i = 0; i < goal_keys.size(); i++){
            if(i == 0){
                pathLength = 0;
                Node* backtrackNode = closed_list.at(goal_keys[i]);
                while(backtrackNode->parent != NULL){
                    simple_drone_sim::Waypoint wp;
                    wp.position.position.x = (backtrackNode->x + x_offset)*map_resolution;
                    wp.position.position.y = (backtrackNode->y + y_offset)*map_resolution;
                    wp.position.position.z = robot_z;
                    best_path.push_back(wp);
                    backtrackNode = backtrackNode->parent;
                    pathLength++;
                }
            }
            else{
                int newPathLength = 0;
                curr_path.clear();
                Node* backtrackNode = closed_list.at(goal_keys[i]);
                while(backtrackNode->parent != NULL){
                    simple_drone_sim::Waypoint wp;
                    wp.position.position.x = (backtrackNode->x + x_offset)*map_resolution;
                    wp.position.position.y = (backtrackNode->y + y_offset)*map_resolution;
                    wp.position.position.z = robot_z;
                    curr_path.push_back(wp);
                    backtrackNode = backtrackNode->parent;
                    newPathLength++;
                }

                if(newPathLength > pathLength){
                    best_path.clear();
                    best_path = curr_path;
                    pathLength = newPathLength;
                }
            }
        }
        plan.plan = best_path;
        plan.time_required = pathLength*0.12;
        plan.battery_required = plan.time_required/2.4;
        std::reverse(plan.plan.begin(), plan.plan.end());
    }
    return pathLength;

}