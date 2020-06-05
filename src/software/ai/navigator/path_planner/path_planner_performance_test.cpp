#include <gtest/gtest.h>

#include <iostream>
#include <typeinfo>

#include "software/ai/navigator/obstacle/obstacle.h"
#include "software/ai/navigator/obstacle/obstacle_factory.h"
#include "software/ai/navigator/path_planner/path_planner.h"
#include "software/ai/navigator/path_planner/straight_line_path_planner.h"
#include "software/ai/navigator/path_planner/theta_star_path_planner.h"
#include "software/new_geom/point.h"
#include "software/new_geom/rectangle.h"
#include "software/test_util/test_util.h"
#include "software/world/world.h"

using PathPlannerConstructor = std::function<std::unique_ptr<PathPlanner>()>;

struct PlannerTestCase
{
    std::string name = "Unnamed test case";
    Point start, end;
    Rectangle navigable_area;
    std::vector<ObstaclePtr> obstacles;
    unsigned int num_iterations;
};

ObstacleFactory obstacle_factory(
    Util::DynamicParameters->getAIConfig()->getObstacleFactoryConfig());

std::vector<ObstaclePtr> obstacles_10 = {
    obstacle_factory.createRobotObstacle({0, 0}),
    obstacle_factory.createRobotObstacle({0, 0.5}),
    obstacle_factory.createRobotObstacle({0, 1.0}),
    obstacle_factory.createRobotObstacle({0, 1.5}),
    obstacle_factory.createRobotObstacle({-0.5, 0}),
    obstacle_factory.createRobotObstacle({-0.5, 0.5}),
    obstacle_factory.createRobotObstacle({-0.5, 1.0}),
    obstacle_factory.createRobotObstacle({-0.5, 1.5}),
    obstacle_factory.createRobotObstacle({0.5, 1.0}),
    obstacle_factory.createRobotObstacle({0.5, 1.5}),
};

std::vector<ObstaclePtr> obstacles_20 = {
    obstacle_factory.createRobotObstacle({0, 0}),
    obstacle_factory.createRobotObstacle({0, 0.5}),
    obstacle_factory.createRobotObstacle({0, 1.0}),
    obstacle_factory.createRobotObstacle({0, 1.5}),
    obstacle_factory.createRobotObstacle({-0.5, 0}),
    obstacle_factory.createRobotObstacle({-0.5, 0.5}),
    obstacle_factory.createRobotObstacle({-0.5, 1.0}),
    obstacle_factory.createRobotObstacle({-0.5, 1.5}),
    obstacle_factory.createRobotObstacle({0.5, 1.0}),
    obstacle_factory.createRobotObstacle({0.5, 1.5}),
    obstacle_factory.createRobotObstacle({1, 0}),
    obstacle_factory.createRobotObstacle({1, 1.5}),
    obstacle_factory.createRobotObstacle({-1.5, 0}),
    obstacle_factory.createRobotObstacle({-1.5, 0.5}),
    obstacle_factory.createRobotObstacle({-1.5, 1.0}),
    obstacle_factory.createRobotObstacle({-1.5, 1.5}),
    obstacle_factory.createRobotObstacle({1.5, 0}),
    obstacle_factory.createRobotObstacle({1.5, 0.5}),
    obstacle_factory.createRobotObstacle({1.5, 1.0}),
    obstacle_factory.createRobotObstacle({1.5, 1.5}),
};

std::vector<ObstaclePtr> obstacles_30 = {
    obstacle_factory.createRobotObstacle({0, 0}),
    obstacle_factory.createRobotObstacle({0, 0.5}),
    obstacle_factory.createRobotObstacle({0, 1.0}),
    obstacle_factory.createRobotObstacle({0, 1.5}),
    obstacle_factory.createRobotObstacle({-0.5, 0}),
    obstacle_factory.createRobotObstacle({-0.5, 0.5}),
    obstacle_factory.createRobotObstacle({-0.5, 1.0}),
    obstacle_factory.createRobotObstacle({-0.5, 1.5}),
    obstacle_factory.createRobotObstacle({0.5, 0}),
    obstacle_factory.createRobotObstacle({0.5, 0.5}),
    obstacle_factory.createRobotObstacle({0.5, 1.0}),
    obstacle_factory.createRobotObstacle({0.5, 1.5}),
    obstacle_factory.createRobotObstacle({1, 0}),
    obstacle_factory.createRobotObstacle({1, 0.5}),
    obstacle_factory.createRobotObstacle({1, 1.0}),
    obstacle_factory.createRobotObstacle({1, 1.5}),
    obstacle_factory.createRobotObstacle({-1.5, 0}),
    obstacle_factory.createRobotObstacle({-1.5, 0.5}),
    obstacle_factory.createRobotObstacle({-1.5, 1.0}),
    obstacle_factory.createRobotObstacle({-1.5, 1.5}),
    obstacle_factory.createRobotObstacle({1.5, 0}),
    obstacle_factory.createRobotObstacle({1.5, 0.5}),
    obstacle_factory.createRobotObstacle({1.5, 1.0}),
    obstacle_factory.createRobotObstacle({1.5, 1.5}),
    obstacle_factory.createRobotObstacle({2, 0}),
    obstacle_factory.createRobotObstacle({2, 0.5}),
    obstacle_factory.createRobotObstacle({2, 1.0}),
    obstacle_factory.createRobotObstacle({2, 1.5}),
    obstacle_factory.createRobotObstacle({-2.5, 0}),
    obstacle_factory.createRobotObstacle({-2.5, 0.5})};

std::vector<PlannerTestCase> test_cases = {
    {.name           = "Empty small area short path",
     .start          = Point(-1, 0),
     .end            = Point(1, 0),
     .navigable_area = Rectangle({-2, -2}, {2, 2}),
     .obstacles      = {},
     .num_iterations = 10},

    {.name           = "Empty medium area short path",
     .start          = Point(-1, 0),
     .end            = Point(1, 0),
     .navigable_area = Rectangle({-3, -3}, {3, 3}),
     .obstacles      = {},
     .num_iterations = 10},

    {.name           = "Empty divB area short path",
     .start          = Point(-1, 0),
     .end            = Point(1, 0),
     .navigable_area = ::Test::TestUtil::createSSLDivBField().fieldBoundary(),
     .obstacles      = {},
     .num_iterations = 10},

    {.name           = "Empty divB area long path",
     .start          = Point(-4.5, 0),
     .end            = Point(4.5, 0),
     .navigable_area = ::Test::TestUtil::createSSLDivBField().fieldBoundary(),
     .obstacles      = {},
     .num_iterations = 10},

    {.name           = "10 obstacles on divB field short path",
     .start          = Point(-2, 0),
     .end            = Point(2, 0),
     .navigable_area = ::Test::TestUtil::createSSLDivBField().fieldBoundary(),
     .obstacles      = obstacles_10,
     .num_iterations = 10},

    {.name           = "20 obstacles on divB field short path",
     .start          = Point(-2, 0),
     .end            = Point(2, 0),
     .navigable_area = ::Test::TestUtil::createSSLDivBField().fieldBoundary(),
     .obstacles      = obstacles_20,
     .num_iterations = 10},

    {.name           = "30 obstacles on divB field short path",
     .start          = Point(-2, 0),
     .end            = Point(2, 0),
     .navigable_area = ::Test::TestUtil::createSSLDivBField().fieldBoundary(),
     .obstacles      = obstacles_30,
     .num_iterations = 10},

    {.name           = "10 obstacles on divB field",
     .start          = Point(-4.5, 0),
     .end            = Point(4.5, 0),
     .navigable_area = ::Test::TestUtil::createSSLDivBField().fieldBoundary(),
     .obstacles      = obstacles_10,
     .num_iterations = 10},

    {.name           = "20 obstacles on divB field",
     .start          = Point(-4.5, 0),
     .end            = Point(4.5, 0),
     .navigable_area = ::Test::TestUtil::createSSLDivBField().fieldBoundary(),
     .obstacles      = obstacles_20,
     .num_iterations = 10},

    {.name           = "30 obstacles on divB field",
     .start          = Point(-4.5, 0),
     .end            = Point(4.5, 0),
     .navigable_area = ::Test::TestUtil::createSSLDivBField().fieldBoundary(),
     .obstacles      = obstacles_30,
     .num_iterations = 10},
};

template <typename PlannerT>
std::pair<std::string, PathPlannerConstructor> nameAndConstructor()
{
    return std::pair<std::string, PathPlannerConstructor>(
        typeid(PlannerT).name(), []() { return std::make_unique<PlannerT>(); });
}

std::vector<std::pair<std::string, PathPlannerConstructor>>
    path_planner_names_and_constructors = {
        // add path planner constructors here
        nameAndConstructor<ThetaStarPathPlanner>(),
        nameAndConstructor<StraightLinePathPlanner>()};

class PlannerPerformanceTest
    : public testing::TestWithParam<
          std::tuple<std::pair<std::string, PathPlannerConstructor>, PlannerTestCase>>
{
};

// This test is disabled to speed up CI, it can be enabled by removing "DISABLED_" from
// the test name
TEST_P(PlannerPerformanceTest, DISABLED_path_planner_performance)
{
    std::string name                     = std::get<0>(GetParam()).first.substr(2);
    std::unique_ptr<PathPlanner> planner = std::get<0>(GetParam()).second();
    auto planner_test_case               = std::get<1>(GetParam());

    auto start_time = std::chrono::high_resolution_clock::now();
    for (unsigned int i = 0; i < planner_test_case.num_iterations; i++)
    {
        planner->findPath(planner_test_case.start, planner_test_case.end,
                          planner_test_case.navigable_area, planner_test_case.obstacles);
    }
    auto end_time = std::chrono::high_resolution_clock::now();

    double duration_us = static_cast<double>(
        std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time)
            .count());
    double avg_us = duration_us / (static_cast<double>(planner_test_case.num_iterations));

    std::cout << std::endl
              << name << " | # iterations = " << planner_test_case.num_iterations
              << " | # obstacles = " << planner_test_case.obstacles.size()
              << " | area = " << planner_test_case.navigable_area.area()
              << " | path length = "
              << (planner_test_case.start - planner_test_case.end).length() << std::endl;

    std::cout << "Total time = " << duration_us / 1000.0
              << "ms | Average time = " << avg_us / 1000.0 << "ms" << std::endl
              << std::endl;
}

INSTANTIATE_TEST_CASE_P(
    All, PlannerPerformanceTest,
    ::testing::Combine(testing::ValuesIn(path_planner_names_and_constructors),
                       testing::ValuesIn(test_cases)));
