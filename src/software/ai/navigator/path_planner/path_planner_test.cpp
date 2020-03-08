#include "software/ai/navigator/path_planner/path_planner.h"
#include "software/ai/navigator/path_planner/theta_star_path_planner.h"
#include "software/ai/navigator/path_planner/straight_line_path_planner.h"
#include "software/test_util/test_util.h"

#include <gtest/gtest.h>

#include <typeinfo>

#include "software/ai/navigator/obstacle/obstacle.h"
#include "software/new_geom/point.h"
#include "software/new_geom/rectangle.h"
#include "software/world/world.h"

// interval at which to evaluate the spline value and check
// if it intersects an obstacle
static constexpr double PATH_CHECK_INTERVAL = 0.05;

using namespace Test;

using PathPlannerConstructor = std::function<std::unique_ptr<PathPlanner>()>;

struct PlannerTestCase
{
    std::string name = "Unnamed test case";
    Point start, dest;
    Rectangle navigable_area;
    std::vector<Obstacle> obstacles;
    bool should_return_path;
};

std::vector<PlannerTestCase> test_cases = {
        {.name           = "Empty field straight line",
        .start          = Point(0, 0),
        .dest           = Point(1, 0),
        .navigable_area = Rectangle({-1, -1}, {1, 1}),
        .obstacles      = {},
        .should_return_path = true},

       {.name           = "Single stationary robot in path",
       .start          = Point(0, 0),
       .dest           = Point(2, 0),
       .navigable_area = Rectangle({-1, -1}, {1, 1}),
       .obstacles      = {
               Obstacle::createRobotObstacle(TestUtil::createRobotAtPos({1, 0}), false)},
       .should_return_path = true},

        {.name           = "Large rectangle in path",
        .start          = Point(0, 0),
        .dest           = Point(2, 0),
        .navigable_area = Rectangle({-1, -1}, {1, 1}),
        .obstacles      = {
        Obstacle::createRobotObstacle(TestUtil::createRobotAtPos({1, 0}), false)},
        .should_return_path = true},};

template <typename PlannerT>
std::pair<std::string, PathPlannerConstructor> name_and_constructor() {
    return std::pair<std::string, PathPlannerConstructor>(typeid(PlannerT).name(),
                                                   []() { return std::make_unique<PlannerT>(); });
}

std::vector<std::pair<std::string, PathPlannerConstructor>> path_planner_names_and_constructors = {
        // add path planner constructors here
        name_and_constructor<ThetaStarPathPlanner>(),
        name_and_constructor<StraightLinePathPlanner>()
};


void validatePath(const Path &path, const Rectangle &navigable_area,
                  const std::vector<Obstacle> &obstacles)
{
    for (long i = 0; i < std::lround(1.0 / PATH_CHECK_INTERVAL); i++)
    {
        // check if the path intersects an obstacle
        Point pt = path.valueAt(i * PATH_CHECK_INTERVAL);
        for (const Obstacle &obs : obstacles)
        {
            if (obs.containsPoint(pt))
            {
                std::stringstream fail_ss;
                fail_ss << "Point " << pt << "at s=" << i * PATH_CHECK_N<< " intersects obstacle {" << obs << "}";
                throw fail_ss.str();
            }
        }
        // check if the path stays within navigable area
        if (!navigable_area.contains(pt))
        {
            std::stringstream fail_ss;
            fail_ss << "Point " << pt << " is not in navigable area " << navigable_area;
            throw fail_ss.str();
        }
    }

    // check that the path reaches the destination if the destination is not in an obstacle
    // otherwise check that the path makes progress toward the destination
    bool dest_in_obstacle = std::any_of(obstacles.begin(), obstacles.end(), [&path](const auto& obs){
        return obs.containsPoint(path.valueAt(1.0));
    });
    if (dest_in_obstacle) {

    }
}

class PlannerTest
    : public testing::TestWithParam<
          std::tuple<std::pair<std::string, PathPlannerConstructor>,
                     PlannerTestCase>>
{
};


TEST_P(PlannerTest, test_path_planner)
{
    std::unique_ptr<PathPlanner> planner = std::get<0>(GetParam()).second();
    auto planner_test_case               = std::get<1>(GetParam());
    std::optional<Path> path =
        planner->findPath(planner_test_case.start, planner_test_case.dest,
                          planner_test_case.navigable_area, planner_test_case.obstacles);
    if (planner_test_case.should_return_path)
    {
        if (path)
        {
            try
            {
                validatePath(*path, planner_test_case.navigable_area,
                             planner_test_case.obstacles);
            }
            catch (const std::string &err)
            {
                FAIL() << std::get<0>(GetParam()).first << " failed because " << err;
            }
        }
        else
        {
            FAIL() << std::get<0>(GetParam()).first
                   << "should have returned path for case " << planner_test_case.name;
        }
    }
    else
    {
        EXPECT_FALSE(path.has_value())
            << std::get<0>(GetParam()).first << " should not have returned path for "
            << planner_test_case.name;
    }
}


INSTANTIATE_TEST_CASE_P(
    All, PlannerTest,
    ::testing::Combine(
        testing::ValuesIn(path_planner_names_and_constructors),
        testing::ValuesIn(test_cases)));
