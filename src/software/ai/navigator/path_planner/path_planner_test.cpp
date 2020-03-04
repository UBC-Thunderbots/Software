#include "software/ai/navigator/path_planner/path_planner.h"

#include <gtest/gtest.h>

#include <typeinfo>

#include "software/ai/navigator/obstacle/obstacle.h"
#include "software/ai/navigator/path_planner/path_planner_factory.h"
#include "software/new_geom/point.h"
#include "software/new_geom/rectangle.h"
#include "software/world/world.h"

// interval at which to evaluate the spline value and check
// if it intersects an obstacle
static constexpr double PATH_CHECK_INTERVAL = 0.05;

struct PlannerTestCase
{
    std::string name = "Unnamed test case";
    Point start, dest;
    Rectangle navigable_area;
    std::vector<Obstacle> obstacles;
    bool should_return_path;
};

std::vector<PlannerTestCase> test_cases = {{.name           = "Trivial test case",
                                            .start          = Point(0, 0),
                                            .dest           = Point(1, 0),
                                            .navigable_area = Rectangle({-1, -1}, {1, 1}),
                                            .obstacles      = {},
                                            .should_return_path = true}};


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
                fail_ss << "Point " << pt << " intersects obstacle {" << obs << "}";
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
}

class PlannerTest
    : public testing::TestWithParam<
          std::tuple<std::pair<std::string, PathPlannerFactory::PathPlannerConstructor>,
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
        testing::ValuesIn(PathPlannerFactory::getPathPlannerConstructors()),
        testing::ValuesIn(::test_cases)));
