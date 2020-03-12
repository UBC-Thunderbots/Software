#include "software/ai/navigator/path_planner/path_planner.h"

#include <gtest/gtest.h>

#include <typeinfo>

#include "software/ai/navigator/obstacle/obstacle.h"
#include "software/ai/navigator/path_planner/straight_line_path_planner.h"
#include "software/ai/navigator/path_planner/theta_star_path_planner.h"
#include "software/new_geom/point.h"
#include "software/new_geom/rectangle.h"
#include "software/test_util/test_util.h"
#include "software/world/world.h"

// length at which to evaluate the spline value and check
// if it intersects an obstacle
static constexpr double PATH_CHECK_INTERVAL_M = 0.05;
// distance to destination before the path is considered to
// have reached the destination
static constexpr double DEST_CHECK_EPSILON_M = 1e-3;

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

std::vector<PlannerTestCase>
    test_cases =
        {{.name               = "Empty field straight line",
          .start              = Point(0, 0),
          .dest               = Point(1, 0),
          .navigable_area     = Rectangle({-1, -1}, {1, 1}),
          .obstacles          = {},
          .should_return_path = true},

         {.name           = "Single stationary robot in path",
          .start          = Point(0, 0),
          .dest           = Point(2, 0),
          .navigable_area = Rectangle({-1, -1}, {1, 1}),
          .obstacles = {Obstacle::createRobotObstacle(TestUtil::createRobotAtPos({1, 0}),
                                                      false)},
          .should_return_path = true},

         {.name               = "Large rectangle in path",
          .start              = Point(-3, 0),
          .dest               = Point(4, 0),
          .navigable_area     = Rectangle({-5, -5}, {5, 5}),
          .obstacles          = {Obstacle(Rectangle({1, 4}, {2, -4}))},
          .should_return_path = true},

         {.name           = "Circle of robots surrounding ego at distance 1",
          .start          = Point(0, 0),
          .dest           = Point(4, 0),
          .navigable_area = Rectangle({-5, -5}, {5, 5}),
          .obstacles =
              {
                  Obstacle::createRobotObstacle(TestUtil::createRobotAtPos({1, 0}),
                                                false),
                  Obstacle::createRobotObstacle(
                      TestUtil::createRobotAtPos(
                          {std::cos(M_PI / 3), std::sin(M_PI / 3)}),
                      false),
                  Obstacle::createRobotObstacle(
                      TestUtil::createRobotAtPos(
                          {std::cos(2 * M_PI / 3), std::sin(2 * M_PI / 3)}),
                      false),
                  Obstacle::createRobotObstacle(
                      TestUtil::createRobotAtPos(
                          {std::cos(3 * M_PI / 3), std::sin(3 * M_PI / 3)}),
                      false),
                  Obstacle::createRobotObstacle(
                      TestUtil::createRobotAtPos(
                          {std::cos(4 * M_PI / 3), std::sin(4 * M_PI / 3)}),
                      false),
                  Obstacle::createRobotObstacle(
                      TestUtil::createRobotAtPos(
                          {std::cos(5 * M_PI / 3), std::sin(5 * M_PI / 3)}),
                      false),
              },
          .should_return_path = true},

         {.name           = "Circle of robots surrounding ego at distance 0.2",
          .start          = Point(0, 0),
          .dest           = Point(4, 0),
          .navigable_area = Rectangle({-5, -5}, {5, 5}),
          .obstacles =
              {
                  Obstacle::createRobotObstacle(TestUtil::createRobotAtPos({0.2, 0}),
                                                false),
                  Obstacle::createRobotObstacle(
                      TestUtil::createRobotAtPos(
                          {std::cos(M_PI / 3) * 0.2, std::sin(M_PI / 3) * 0.2}),
                      false),
                  Obstacle::createRobotObstacle(
                      TestUtil::createRobotAtPos(
                          {std::cos(2 * M_PI / 3) * 0.2, std::sin(2 * M_PI / 3) * 0.2}),
                      false),
                  Obstacle::createRobotObstacle(
                      TestUtil::createRobotAtPos(
                          {std::cos(3 * M_PI / 3) * 0.2, std::sin(3 * M_PI / 3) * 0.2}),
                      false),
                  Obstacle::createRobotObstacle(
                      TestUtil::createRobotAtPos(
                          {std::cos(4 * M_PI / 3) * 0.2, std::sin(4 * M_PI / 3) * 0.2}),
                      false),
                  Obstacle::createRobotObstacle(
                      TestUtil::createRobotAtPos(
                          {std::cos(5 * M_PI / 3) * 0.2, std::sin(5 * M_PI / 3) * 0.2}),
                      false),
              },
          .should_return_path = false}};

template <typename PlannerT>
std::pair<std::string, PathPlannerConstructor> name_and_constructor()
{
    return std::pair<std::string, PathPlannerConstructor>(
        typeid(PlannerT).name(), []() { return std::make_unique<PlannerT>(); });
}

std::vector<std::pair<std::string, PathPlannerConstructor>>
    path_planner_names_and_constructors = {
        // add path planner constructors here
        name_and_constructor<ThetaStarPathPlanner>(),
        // uncomment this if you want some tests to fail
        //        name_and_constructor<StraightLinePathPlanner>()
};


void validatePath(const Path &path, const Point &start, const Point &dest,
                  const Rectangle &navigable_area, const std::vector<Obstacle> &obstacles)
{
    // check for zero length path
    if (path.valueAt(0.f) == path.valueAt(1.f))
    {
        throw std::string("zero length path!");
    }

    // compute an s value interval that roughly corresponds to length
    // PATH_CHECK_INTERVAL_M assuming that the path is locally linear
    double s_per_meter_ratio   = 0.05 / (path.valueAt(0.05) - path.valueAt(0.0)).length();
    double path_check_interval = s_per_meter_ratio * PATH_CHECK_INTERVAL_M;

    std::cout << "Evaluating path at intervals of s=" << path_check_interval << std::endl;

    for (double s = 0.0; s <= 1.0; s += path_check_interval)
    {
        Point pt = path.valueAt(s);
        for (const Obstacle &obs : obstacles)
        {
            if (obs.containsPoint(pt))
            {
                // fail because path intersects obstacle
                std::stringstream fail_ss;
                fail_ss << "Point " << pt << "at s=" << s << " intersects obstacle {"
                        << obs << "}";
                throw fail_ss.str();
            }
        }
        if (!navigable_area.contains(pt))
        {
            // fail because path exited navigable area
            std::stringstream fail_ss;
            fail_ss << "Point " << pt << "at s=" << s << " is not in navigable area "
                    << navigable_area;
            throw fail_ss.str();
        }
    }

    // check that the path reaches the destination if the destination is not in an
    // obstacle otherwise check that the path makes progress toward the destination i.e.
    // the distance to dest is less at the end of the path than at the start of the path
    bool dest_in_obstacle = std::any_of(
        obstacles.begin(), obstacles.end(),
        [&path](const auto &obs) { return obs.containsPoint(path.valueAt(1.0)); });

    if (dest_in_obstacle &&
        (path.valueAt(1.0) - dest).length() >= (path.valueAt(0.0) - dest).length())
    {
        // fail because no progress to destination
        std::stringstream fail_ss;
        fail_ss
            << "Destination is in obstacle, but path does not make progress toward the destination!";
        throw fail_ss.str();
    }

    if ((path.valueAt(1.0) - dest).length() <= DEST_CHECK_EPSILON_M)
    {
        // fail because didn't reach destination
        std::stringstream fail_ss;
        fail_ss << "Path ends at " << path.valueAt(1.0) << " but dest is " << dest;
        throw fail_ss.str();
    }
    // we passed, yay!
}

class PlannerTest
    : public testing::TestWithParam<
          std::tuple<std::pair<std::string, PathPlannerConstructor>, PlannerTestCase>>
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
                validatePath(*path, planner_test_case.start, planner_test_case.dest,
                             planner_test_case.navigable_area,
                             planner_test_case.obstacles);
            }
            catch (const std::string &err)
            {
                FAIL() << std::get<0>(GetParam()).first << " failed on case \""
                       << planner_test_case.name << "\" because " << err;
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

// uncomment this to run tests
// cases still fail because theta* will intersect obstacles' bounding boxes at knots
// TODO: #1207 differentiate between Theta* pathfinding bounding boxes and the actual
// extent of obstacles
// INSTANTIATE_TEST_CASE_P(
//    All, PlannerTest,
//    ::testing::Combine(testing::ValuesIn(path_planner_names_and_constructors),
//                       testing::ValuesIn(test_cases)));
