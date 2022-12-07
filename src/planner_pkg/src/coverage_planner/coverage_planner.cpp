#include "../../include/coverage_planner/coverage_planner.h"
#include <chrono>

void CoveragePlanner::setRobotLocation(const geometry_msgs::PoseStamped& pose)
{
    start_node.x = pose.pose.position.x;
    start_node.y = pose.pose.position.y;
    _xinit = pose.pose.position.x;
    _yinit = pose.pose.position.y;
    ROS_INFO("Robot is starting at (%d, %d)", _xinit, _yinit);

    _path_height = pose.pose.position.z;
}

void CoveragePlanner::updateMap(const nav_msgs::OccupancyGrid::ConstPtr& grid)
{
    _map_resolution = grid->info.resolution;
    _ys = grid->info.width;
    _xs = grid->info.height;
    _xo = (grid->info.origin.position.x)/(_map_resolution);
    _yo = (grid->info.origin.position.y)/(_map_resolution);

    _map.clear();
    _map = grid->data;
    _map_size = 1500;

    _covered.clear();
    _covered.resize(_map.size());
    ROS_INFO("xs: %d, ys: %d, xo: %d, yo: %d, num cell: %ld", _xs, _ys, _xo, _yo, _map.size());
}

void CoveragePlanner::updateObstacles(const simple_drone_sim::ObstacleArrayConstPtr& obstacles)
{
    _obstacles = obstacles->obstacles;
    float temp_height = std::numeric_limits<float>::max();
    for (const auto& obs : _obstacles)
    {
        if (obs.height < temp_height) { temp_height = obs.height; }
    }
    _path_height = temp_height/2;
    ROS_INFO("Received obstacles. There are %ld of them. I will fly at a height of %d", _obstacles.size(), _path_height);
}

void CoveragePlanner::printInfo()
{
    ROS_INFO_STREAM("Map size: " << _map.size());
    ROS_INFO_STREAM("Robot Position: " <<  start_node.x << " " << start_node.y);
    ROS_INFO_STREAM("Number of Landing Zones: " << goal_locations.size());

    ROS_INFO_STREAM("x_offset: "<< _xo <<" y_offset: " <<_yo);
}

void CoveragePlanner::updateCoverage(int x, int y)
{
    int start_x = (int) (x - _coverage_size / 2. + _map_size / 2) / _map_resolution;
    int end_x = (int) (x + _coverage_size / 2. + _map_size / 2) / _map_resolution;
    int start_y = (int) (y - _coverage_size / 2. + _map_size / 2)  / _map_resolution;
    int end_y = (int) (y + _coverage_size / 2. + _map_size / 2)  / _map_resolution;
    for (int i = start_y; i < end_y; ++i)
    {
        for (int j = start_x; j < end_x; ++j)
        {
            _covered[xy2idx(j,i)] = true;
        }
    }
}

void CoveragePlanner::updateCells()
{
    for (Node &n: goal_locations)
    {
        n.x = (n.x)/(_map_resolution) - _xo;
        n.y = (n.y)/(_map_resolution) - _yo;
    }

    start_node.x = start_node.x/_map_resolution - _xo;
    start_node.y = start_node.y/_map_resolution - _yo;
}

int CoveragePlanner::plan(simple_drone_sim::Plan& plan)
{
    return 0;
}

int CoveragePlanner::idiotPlan(simple_drone_sim::Plan& plan)
{
    plan.plan.clear();
    // go to map corner
    simple_drone_sim::Waypoint wp;
    wp.position.position.x = _xo;
    wp.position.position.y = _yo;
    wp.position.position.z = _path_height;
    plan.plan.push_back(wp);

    wp.position.position.x = -_xo;
    wp.position.position.y = _yo;
    wp.position.position.z = _path_height;
    plan.plan.push_back(wp);

    for (int col = 0; col*_coverage_size < _map_size; col++)
    {
        simple_drone_sim::Waypoint wp;
        if (col % 2 == 0)
        {
            wp.position.position.x = -_xo;
        }
        else{
            wp.position.position.x = _xo;
        }
        wp.position.position.y = _yo+col*_coverage_size;
        wp.position.position.z = _path_height;
        plan.plan.push_back(wp);

        wp.position.position.y += _coverage_size;
        plan.plan.push_back(wp);
    }
    int i = 1;
    for (const auto& wp : plan.plan)
    {
        ROS_INFO("Waypoint %d: (%f,%f,%f)",i,wp.position.position.x,wp.position.position.y,wp.position.position.z);
        i++;
    }
    return 0;
}