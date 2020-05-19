#include "software/ai/navigator/path_planner/path_planner.h"

#include <gtest/gtest.h>

#include <typeinfo>

#include "software/ai/navigator/obstacle/obstacle.h"
#include "software/ai/navigator/obstacle/obstacle_factory.h"
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
    std::vector<ObstaclePtr> obstacles;
    bool should_return_path;
};

ObstacleFactory obstacle_factory(
    Util::DynamicParameters->getAIConfig()->getObstacleFactoryConfig());

std::vector<PlannerTestCase>
    test_cases =
        {{.name               = "Empty field straight line",
          .start              = Point(0, 0),
          .dest               = Point(1, 0),
          .navigable_area     = Rectangle({-1, -1}, {2, 2}),
          .obstacles          = {},
          .should_return_path = true},

         {.name               = "Single stationary robot in path",
          .start              = Point(0, 0),
          .dest               = Point(2, 0),
          .navigable_area     = Rectangle({-2, -2}, {2, 2}),
          .obstacles          = {obstacle_factory.createRobotObstacle(Point({1, 0}))},
          .should_return_path = true},

         {.name               = "Large rectangle in path",
          .start              = Point(-3, 0),
          .dest               = Point(4, 0),
          .navigable_area     = Rectangle({-5, -5}, {5, 5}),
          .obstacles          = {obstacle_factory.createObstacleFromRectangle(
              Rectangle({1, 4}, {2, -4}))},
          .should_return_path = true},

         {.name           = "Circle of robots surrounding friendly robot at distance 1",
          .start          = Point(0, 0),
          .dest           = Point(4, 0),
          .navigable_area = Rectangle({-5, -5}, {5, 5}),
          .obstacles =
              {
                  obstacle_factory.createRobotObstacle(Point({1, 0})),
                  obstacle_factory.createRobotObstacle(
                      Point({std::cos(M_PI / 3), std::sin(M_PI / 3)})),
                  obstacle_factory.createRobotObstacle(
                      Point({std::cos(2 * M_PI / 3), std::sin(2 * M_PI / 3)})),
                  obstacle_factory.createRobotObstacle(
                      Point({std::cos(3 * M_PI / 3), std::sin(3 * M_PI / 3)})),
                  obstacle_factory.createRobotObstacle(
                      Point({std::cos(4 * M_PI / 3), std::sin(4 * M_PI / 3)})),
                  obstacle_factory.createRobotObstacle(
                      Point({std::cos(5 * M_PI / 3), std::sin(5 * M_PI / 3)})),
              },
          .should_return_path = true},

         {.name           = "Circle of robots surrounding friendly robot at distance 0.2",
          .start          = Point(0, 0),
          .dest           = Point(4, 0),
          .navigable_area = Rectangle({-5, -5}, {5, 5}),
          .obstacles =
              {
                  obstacle_factory.createRobotObstacle(Point({0.2, 0})),
                  obstacle_factory.createRobotObstacle(
                      Point({std::cos(M_PI / 3) * 0.2, std::sin(M_PI / 3) * 0.2})),
                  obstacle_factory.createRobotObstacle(
                      Point(
                          {std::cos(2 * M_PI / 3) * 0.2, std::sin(2 * M_PI / 3) * 0.2})),
                  obstacle_factory
                      .createRobotObstacle(Point({std::cos(3 * M_PI / 3) * 0.2,
                                                  std::sin(3 * M_PI / 3) * 0.2})),
                  obstacle_factory
                      .createRobotObstacle(Point({std::cos(4 * M_PI / 3) * 0.2,
                                                  std::sin(4 * M_PI / 3) * 0.2})),
                  obstacle_factory
                      .createRobotObstacle(Point({std::cos(5 * M_PI / 3) * 0.2,
                                                  std::sin(5 * M_PI / 3) * 0.2})),
              },
          .should_return_path = false},
         {.name  = "Start inside a rectangular obstacle, dest is outside of obstacle",
          .start = Point(0, 0),
          .dest  = Point(4, 0),
          .navigable_area     = Rectangle({-5, -5}, {5, 5}),
          .obstacles          = {obstacle_factory.createObstacleFromRectangle(
              Rectangle({-1, -1}, {1, 1}))},
          .should_return_path = true},
         {.name = "Start and dest inside same obstacle",
          // NOTE: this test is designed specifically to pass the progress check
          .start              = Point(0, 0),
          .dest               = Point(1.5, 0),
          .navigable_area     = Rectangle({-5, -5}, {5, 5}),
          .obstacles          = {obstacle_factory.createObstacleFromRectangle(
              Rectangle({-1, -1}, {2, 1}))},
          .should_return_path = true}};


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
                  const Rectangle &navigable_area, std::vector<ObstaclePtr> &obstacles)
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

    // special case stuff for if the path starts in an obstacle
    std::optional<ObstaclePtr> start_obstacle_or_null = std::nullopt;
    // check if the path starts inside an obstacle
    auto start_obstacle_or_end_it = std::find_if(
        obstacles.begin(), obstacles.end(),
        [&path](const auto &obs) { return obs->contains(path.valueAt(0.f)); });
    // remove the obstacle from obstacles *temporarily* until we exit the obstacle
    if (start_obstacle_or_end_it != obstacles.end())
    {
        start_obstacle_or_null = *start_obstacle_or_end_it;
        obstacles.erase(start_obstacle_or_end_it);
    }


    for (double s = 0.0; s <= 1.0;
         s = (s != 1.0 && s + path_check_interval > 1.0) ? 1.0 : s + path_check_interval)
    {
        Point pt = path.valueAt(s);
        // check if we exited the first obstacle, and add it back to obstacles
        if (start_obstacle_or_null && !(*start_obstacle_or_null)->contains(pt))
        {
            obstacles.emplace_back(*start_obstacle_or_null);
            start_obstacle_or_null = std::nullopt;
        }

        for (const ObstaclePtr &obs : obstacles)
        {
            if (obs->contains(pt))
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

    bool dest_in_obstacle =
        std::any_of(obstacles.begin(), obstacles.end(),
                    [&dest](const auto &obs) { return obs->contains(dest); });

    // check if the specified destination is in an obstacle, and if so, check that the
    // robot made progress toward the destination we also check for start_obstacle_or_null
    // because it will only be true at this stage in the case where we start in an
    // obstacle and never exit it
    if (dest_in_obstacle || start_obstacle_or_null)
    {
        if ((path.valueAt(1.0) - dest).length() >= (path.valueAt(0.0) - dest).length())
        {
            // fail because no progress to destination
            std::stringstream fail_ss;
            fail_ss
                << "Destination is in obstacle, but path does not make progress toward the destination!";
            throw fail_ss.str();
        }
    }
    else if ((path.valueAt(1.0) - dest).length() >= DEST_CHECK_EPSILON_M)
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
