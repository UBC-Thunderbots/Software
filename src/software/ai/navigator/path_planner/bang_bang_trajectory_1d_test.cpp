#include "software/ai/navigator/path_planner/bang_bang_trajectory_1d.h"

class TestEnlsvgPathPlanner : public testing::Test
{
public:
    TestEnlsvgPathPlanner()
            : robot_navigation_obstacle_config(TbotsProto::RobotNavigationObstacleConfig()),
              robot_navigation_obstacle_factory(robot_navigation_obstacle_config)
    {
    }

    TbotsProto::RobotNavigationObstacleConfig robot_navigation_obstacle_config;
    RobotNavigationObstacleFactory robot_navigation_obstacle_factory;
};
