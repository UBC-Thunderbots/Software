#include <gtest/gtest.h>
#include <include/gmock/gmock-matchers.h>

#include "software/geom/algorithms/distance.h"
#include "software/geom/algorithms/nearest_neighbor_search.hpp"
#include "software/test_util/test_util.h"
#include "software/world/robot.h"

class NearestNeighborSearchTest : public ::testing::TestWithParam<double>
{
   protected:
    NearestNeighborSearchTest()
    {
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
    Robot candidate = TestUtil::createRobotAtPos(Point(0, 0));
};

double compare(const Robot &r1, const Robot &r2)
{
    return distanceSquared(r1.position(), r2.position());
}

TEST_F(NearestNeighborSearchTest, no_robot_within_radius_test)
{
    double radius_squared = 1.0 * 1.0;
    std::vector<Robot> expected{};
    std::vector<Robot> agent_subset =
        findNearestNeighbours(candidate, agents, radius_squared, compare);
    EXPECT_THAT(expected, agent_subset);
}

TEST_F(NearestNeighborSearchTest, two_robots_within_radius_test)
{
    double radius_squared = 2.0 * 2.0;
    std::vector<Robot> expected{agents[0], agents[1]};
    std::vector<Robot> agent_subset =
        findNearestNeighbours(candidate, agents, radius_squared, compare);
    std::sort(expected.begin(), expected.end(), Robot::cmpRobotByID());
    std::sort(agent_subset.begin(), agent_subset.end(), Robot::cmpRobotByID());
    EXPECT_THAT(expected, ::testing::ContainerEq(agent_subset));
}

TEST_F(NearestNeighborSearchTest, four_robots_within_radius_test)
{
    double radius_squared = 3.0 * 3.0;
    std::vector<Robot> expected{agents[0], agents[1], agents[2], agents[3]};
    std::vector<Robot> agent_subset =
        findNearestNeighbours(candidate, agents, radius_squared, compare);
    std::sort(expected.begin(), expected.end(), Robot::cmpRobotByID());
    std::sort(agent_subset.begin(), agent_subset.end(), Robot::cmpRobotByID());
    EXPECT_THAT(expected, ::testing::ContainerEq(agent_subset));
}

TEST_F(NearestNeighborSearchTest, robot_on_edge_of_radius_test)
{
    double radius_squared = 5.0 * 5.0;
    std::vector<Robot> expected{agents[0], agents[1], agents[2],
                                agents[3], agents[4], agents[5]};
    std::vector<Robot> agent_subset =
        findNearestNeighbours(candidate, agents, radius_squared, compare);
    std::sort(expected.begin(), expected.end(), Robot::cmpRobotByID());
    std::sort(agent_subset.begin(), agent_subset.end(), Robot::cmpRobotByID());
    EXPECT_THAT(expected, ::testing::ContainerEq(agent_subset));
}
