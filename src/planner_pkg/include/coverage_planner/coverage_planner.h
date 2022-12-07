#ifndef _COVERAGE_PLANNER_H_
#define _COVERAGE_PLANNER_H_

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include "simple_drone_sim/TargetPoses.h"
#include "simple_drone_sim/TargetPose.h"
#include "simple_drone_sim/Waypoint.h"
#include "simple_drone_sim/Plan.h"
#include "simple_drone_sim/ObstacleArray.h"
#include "simple_drone_sim/ObstaclePose.h"

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

class CoveragePlanner{
    public:
        CoveragePlanner(int size_x, int size_y, int x_offset, int y_offset) : 
            _xs(size_x),
            _ys(size_y), 
            _xo(x_offset), 
            _yo(y_offset)
        {};

        CoveragePlanner(){};
        ~CoveragePlanner(){};

        int planToGoals(simple_drone_sim::Plan& plan);
        int plan(simple_drone_sim::Plan& plan);
        int idiotPlan(simple_drone_sim::Plan& plan);
        
        void updateMap(const nav_msgs::OccupancyGrid::ConstPtr&);
        void updateObstacles(const simple_drone_sim::ObstacleArrayConstPtr& obstacles);

        void setRobotLocation(const geometry_msgs::PoseStamped&);

        void printInfo();

    private:
        int _xs, _ys, _xo, _yo;
        float _map_resolution;
        int _map_size;

        int _coverage_size = 50;
        
        float obstacle_cost;

        int _xinit, _yinit;
        int _path_height;
        
        int dX[8] = {-1, -1, -1,  0,  0,  1, 1, 1};
        int dY[8] = {-1,  0,  1, -1,  1, -1, 0, 1};

        Node start_node;
        std::vector<Node> goal_locations;

        std::vector<int8_t, std::allocator<int8_t>> _map = {};
        std::vector<bool> _covered = {};
        std::vector<simple_drone_sim::ObstaclePose> _obstacles = {};

        inline int xy2idx(int x, int y)
        {
            return (y*_ys + x);
        }

        std::vector<int> generateNeighbors(int x, int y)
        {

        }

        void updateCoverage(int x, int y);

        void updateCells();

};

#endif