#include <gtest/gtest.h>
#include <include/gmock/gmock-matchers.h>

#include <chrono>

#include "software/ai/navigator/path_planner/hrvo/brute_force_nearest_neighbor_search.hpp"
#include "software/geom/algorithms/distance.h"
#include "software/test_util/test_util.h"
#include "software/world/robot.h"

class NearestNeighborSearchTest : public ::testing::TestWithParam<double>
{
   protected:
    NearestNeighborSearchTest()
    {
        robot_locations.push_back(Point(0, 0));
        robot_locations.push_back(Point(1, 1));
        robot_locations.push_back(Point(-1, -1));
        robot_locations.push_back(Point(-2, 2));
        robot_locations.push_back(Point(2, -2));
        robot_locations.push_back(Point(-4, -3));
        robot_locations.push_back(Point(4, 3));
    }

    void SetUp() override
    {
        for (Point point : robot_locations)
        {
            Robot agent = TestUtil::createRobotAtPos(point);
            agents.push_back(agent);
        }
    }

    std::vector<Point> robot_locations;
    std::vector<Robot> agents;
};

double compare(const Robot &r1, const Robot &r2)
{
    return distanceSquared(r1.position(), r2.position());
}

TEST_F(NearestNeighborSearchTest, no_robot_within_radius_test)
{
    double radius = 1.0;
    std::vector<Robot> expected{};
    std::vector<Robot> agent_subset =
        nearestNeighbours(agents[0], agents, radius, compare);
    EXPECT_THAT(expected, agent_subset);
}

TEST_F(NearestNeighborSearchTest, two_robots_within_radius_test)
{
    double radius = 2.0;
    std::vector<Robot> expected{agents[1], agents[2]};
    std::vector<Robot> agent_subset =
        nearestNeighbours(agents[0], agents, radius, compare);
    std::sort(expected.begin(), expected.end(), Robot::cmpRobotByID());
    std::sort(agent_subset.begin(), agent_subset.end(), Robot::cmpRobotByID());
    EXPECT_THAT(expected, ::testing::ContainerEq(agent_subset));
}

TEST_F(NearestNeighborSearchTest, four_robots_within_radius_test)
{
    double radius = 3.0;
    std::vector<Robot> expected{agents[1], agents[2], agents[3], agents[4]};
    std::vector<Robot> agent_subset =
        nearestNeighbours(agents[0], agents, radius, compare);
    std::sort(expected.begin(), expected.end(), Robot::cmpRobotByID());
    std::sort(agent_subset.begin(), agent_subset.end(), Robot::cmpRobotByID());
    EXPECT_THAT(expected, ::testing::ContainerEq(agent_subset));
}

TEST_F(NearestNeighborSearchTest, robot_on_edge_of_radius_test)
{
    double radius = 5.0;
    std::vector<Robot> expected{agents[1], agents[2], agents[3], agents[4]};
    std::vector<Robot> agent_subset =
        nearestNeighbours(agents[0], agents, radius, compare);
    std::sort(expected.begin(), expected.end(), Robot::cmpRobotByID());
    std::sort(agent_subset.begin(), agent_subset.end(), Robot::cmpRobotByID());
    EXPECT_THAT(expected, ::testing::ContainerEq(agent_subset));
}
