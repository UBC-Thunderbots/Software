/**
 * This file contains tests for the AStarPathPlanner.
 */

#include "ai/navigator/path_planner/astar.h"

#include <gtest/gtest.h>

#include "ai/navigator/RobotObstacle.h"
#include "ai/world/world.h"

// define operator<< for std::vector so I can debug paths
template <typename T>
std::ostream& operator<<(std::ostream& os, const std::vector<T> items)
{
    for (const auto& item : items)
    {
        os << item << std::endl;
    }
    return os;
}

bool pathViolatesObstacles(const std::vector<Point>& path,
                           const std::vector<RobotObstacle>& obstacles)
{
    // a path violates obstacles if the total violation increases from
    // the first point to the last point
    double initial_violation = 0.0f;
    for (const auto& obstacle : obstacles)
    {
        initial_violation += obstacle.getViolationDistance(path[0]);
    }

    double total_violation = 0.0f;
    for (const auto& point : path)
    {
        for (const auto& obstacle : obstacles)
        {
            total_violation += obstacle.getViolationDistance(point);
        }
    }
    std::cout << "Initial violation: " << initial_violation << std::endl;
    std::cout << "Final violation: " << total_violation << std::endl;
    return initial_violation < total_violation;
}

class AStarPathPlannerTest : public ::testing::Test
{
   protected:
    AStarPathPlannerTest()
        : testField(3.0f, 3.0f, 0.1f, 0.1f, 0.1f, 0.0f, 0.1f),
          testViolationFunction([](const Point& p) { return 0; })
    {
        testWorld.updateFieldGeometry(testField);
    }
    void SetUp() override {}
    World testWorld;
    Field testField;
    const ViolationFunction testViolationFunction;
    static constexpr size_t GRID_VERTEX_DENSITY = 10;
};

TEST_F(AStarPathPlannerTest, test_construct_path_planner)
{
    std::unique_ptr<PathPlanner> planner =
        std::make_unique<AStar::AStarPathPlanner>(testField, GRID_VERTEX_DENSITY);
    SUCCEED();
}

TEST_F(AStarPathPlannerTest, test_find_path_trivial)
{
    std::unique_ptr<PathPlanner> planner =
        std::make_unique<AStar::AStarPathPlanner>(testField, GRID_VERTEX_DENSITY);
    Point start{0.0f, 0.0f}, end{0.0f, 0.0f};
    std::vector<Point> path = *planner->findPath(testViolationFunction, start, end);
    EXPECT_EQ(path[0], Point(0, 0));
    EXPECT_EQ(path[1], Point(0, 0));
}

TEST_F(AStarPathPlannerTest, test_find_path_diagonal)
{
    std::unique_ptr<PathPlanner> planner =
        std::make_unique<AStar::AStarPathPlanner>(testField, GRID_VERTEX_DENSITY);
    Point start{0.0f, 0.0f}, end{0.5f, 0.5f};
    std::vector<Point> path = *planner->findPath(testViolationFunction, start, end);
    std::cout << path << std::endl;
    EXPECT_EQ(path[0], start);
    EXPECT_EQ(path[path.size() - 1], end);
}

TEST_F(AStarPathPlannerTest, test_find_path_vertical_with_obstacles)
{
    std::unique_ptr<PathPlanner> planner =
        std::make_unique<AStar::AStarPathPlanner>(testField, GRID_VERTEX_DENSITY);
    Point start{0.0f, 0.0f}, end{0.0f, 1.0f};
    Robot obstacle_robot(0, Point(0.0f, 0.2f), Vector(), Angle(), AngularVelocity(),
                         Timestamp());
    RobotObstacle obstacle(obstacle_robot, 0.1f);
    ViolationFunction violation_function = [&obstacle](const Point& p) {
        return obstacle.getViolationDistance(p);
    };
    std::vector<Point> path = *planner->findPath(violation_function, start, end);
    std::cout << path << std::endl;
    EXPECT_FALSE(pathViolatesObstacles(path, {obstacle}));
}

int main(int argc, char** argv)
{
    std::cout << argv[0] << std::endl;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
