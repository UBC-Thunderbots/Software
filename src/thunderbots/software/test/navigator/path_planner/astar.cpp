//
// Created by jordan on 2/26/19.
//

#include "ai/navigator/path_planner/astar.h"

#include <gtest/gtest.h>

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

class AStarPathPlannerTest : public ::testing::Test
{
   protected:
    AStarPathPlannerTest() : testField(1.0f, 1.0f, 0.1f, 0.1f, 0.1f, 0.0f, 0.1f)
    {
        testWorld.updateFieldGeometry(testField);
    }
    void SetUp() override {}
    World testWorld;
    Field testField;
};

TEST_F(AStarPathPlannerTest, test_construct_path_planner)
{
    std::unique_ptr<PathPlanner> planner =
        std::make_unique<AStar::AStarPathPlanner>(testField);
    SUCCEED();
}

TEST_F(AStarPathPlannerTest, test_find_path_trivial)
{
    std::unique_ptr<PathPlanner> planner =
        std::make_unique<AStar::AStarPathPlanner>(testField);
    Point start{0.0f, 0.0f}, end{0.0f, 0.0f};
    std::vector<Point> path = *planner->findPath(testWorld, start, end);
    EXPECT_EQ(path[0], Point(0, 0));
    EXPECT_EQ(path[1], Point(0, 0));
}

TEST_F(AStarPathPlannerTest, test_find_path_diagonal)
{
    std::unique_ptr<PathPlanner> planner =
        std::make_unique<AStar::AStarPathPlanner>(testField);
    Point start{0.0f, 0.0f}, end{0.5f, 0.5f};
    std::vector<Point> path = *planner->findPath(testWorld, start, end);
    std::cout << path << std::endl;
    EXPECT_EQ(path[0], start);
    EXPECT_EQ(path[path.size() - 1], end);
}

int main(int argc, char** argv)
{
    std::cout << argv[0] << std::endl;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}