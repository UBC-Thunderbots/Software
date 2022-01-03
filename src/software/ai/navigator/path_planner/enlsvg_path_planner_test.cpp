#include "software/ai/navigator/path_planner/enlsvg_path_planner.h"
#include <gtest/gtest.h>

#include "software/ai/navigator/obstacle/robot_navigation_obstacle_factory.h"

class TestEnlsvgPathPlanner : public testing::Test
{
    public:
        TestEnlsvgPathPlanner()
            : robot_navigation_obstacle_factory(
                std::make_shared<const RobotNavigationObstacleConfig>())
        {
        }
        
        RobotNavigationObstacleFactory robot_navigation_obstacle_factory;
};

TEST_F(TestEnlsvgPathPlanner, test_enlsvg_path_planner_empty_grid)
{
    Field field = Field::createSSLDivisionAField();
    Point start{2, 2}, dest{-3, -3};
    
    std::vector<ObstaclePtr> obstacles = {};
    Rectangle navigable_area = field.fieldBoundary();
    
    EnlsvgPathPlanner planner = EnlsvgPathPlanner(navigable_area, obstacles);
    auto path = planner.findPath(start, dest, navigable_area, obstacles);
    
    EXPECT_TRUE(path != std::nullopt);
}