#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include "simple_drone_sim/TargetPoses.h"
#include "simple_drone_sim/TargetPose.h"
#include "simple_drone_sim/Waypoint.h"
#include "simple_drone_sim/Plan.h"

#include<queue>
#include<unordered_map>
#include<ros/ros.h>

struct Node{
    int key;
    int x, y;
    float g, h, f;
    Node* parent;

    Node(){}

    Node(int x, int y):x(x),y(y)
    {
    }

    //Equals Comparision operator for unordered map
    bool operator==(const Node &n) const {
            return n.x == x && n.y == y;
    }

};

//Less than operator for priority queue
struct CompareFValues {
    bool operator()(Node *n1, Node *n2)
    {
        return n1->f > n2->f;
    }
};

//Defining a custom closed list of type unordered_map
typedef std::unordered_map<int, Node*> CLOSED_LIST;

//Defining an open list of type priority queue
typedef std::priority_queue<Node*, std::vector<Node*>, CompareFValues> OPEN_LIST;

class LandingPlanner{
    public:
        LandingPlanner(int size_x, int size_y, int x_offset, int y_offset);
        LandingPlanner();
        ~LandingPlanner();

        int planToGoals(simple_drone_sim::Plan& plan);
        
        void updateMap(const nav_msgs::OccupancyGrid::ConstPtr&);

        void setTargets(const simple_drone_sim::TargetPoses&);
        void setRobotLocation(const geometry_msgs::PoseStamped&);
        void setBattery(const float);

        void printInfo();

    private:
        int x_size, y_size, x_offset, y_offset;
        float map_resolution;
        
        float obstacle_cost;
        float start_battery;
        float robot_z;
        
        int dX[8] = {-1, -1, -1,  0,  0,  1, 1, 1};
        int dY[8] = {-1,  0,  1, -1,  1, -1, 0, 1};

        Node start_node;
        std::vector<Node> goal_locations;

        std::vector<int8_t, std::allocator<int8_t>> map;

        int computeKey(int, int);
        void updateCells();

};