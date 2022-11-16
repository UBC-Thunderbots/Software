#include <gtest/gtest.h>

#include "software/ai/navigator/path_planner/hrvo/brute_force_nearest_neighbor_search.hpp"
#include "software/geom/algorithms/distance.h"
#include "software/test_util/test_util.h"
#include "software/world/robot.h"
#include "shared/constants.h"
#include <random>
#include <chrono>

class NearestNeighborSearchTest : public ::testing::TestWithParam<double> {
protected:
    NearestNeighborSearchTest() {
        for (Point point : robot_locations) {
            Robot agent = TestUtil::createRobotAtPos(point);
            agents.push_back(agent);
        }
    }

    void SetUp() override {
        robot_locations.push_back(Point(0,0));
        robot_locations.push_back(Point(-4, -3));
        robot_locations.push_back(Point(1, 1));
        robot_locations.push_back(Point(-1, -1));
        robot_locations.push_back(Point(-2, 2));
        robot_locations.push_back(Point(2, -2));
    }

    std::vector<Point> robot_locations;
    std::vector<Robot> agents;
};

double compare(const Robot &r1, const Robot &r2) {
    return distanceSquared(r1.position(), r2.position());
}

TEST_P(NearestNeighborSearchTest, num_robot_within_radius) {
    double radius = GetParam();
    std::vector<Robot> agent_subset = nearestNeighbours(agents[0], agents, radius, compare);
}

TEST(NearestNeighborSearchTest, generic_frnn_brute_force_test)
{
    unsigned int iterations = 1000;
    unsigned int num_of_agents = DIV_A_NUM_ROBOTS * 2;
    unsigned int friendly_agents = DIV_A_NUM_ROBOTS;
    double radius = 1.0;
    double lower_x_bound = -4.5;
    double upper_x_bound = 4.5;
    double lower_y_bound = -3;
    double upper_y_bound = 3;
    std::chrono::nanoseconds duration_total(0);
    std::chrono::nanoseconds max(0);
    std::chrono::nanoseconds min(0);

    std::uniform_real_distribution<double> x_unif(lower_x_bound,upper_x_bound);
    std::uniform_real_distribution<double> y_unif(lower_y_bound,upper_y_bound);
    std::default_random_engine re;

    for (unsigned int i = 0; i < iterations; i++) {
        std::vector<Robot> agents;
        for (unsigned int j = 0; j < num_of_agents; j++) {
            Robot agent = TestUtil::createRobotAtPos(Point(x_unif(re), y_unif(re)));
            agents.push_back(agent);
        }

        unsigned int robot_counter = 0;
        auto start = std::chrono::high_resolution_clock::now();
        for (const Robot &agent : agents) {
            if (robot_counter >= friendly_agents) {
                break;
            }
            std::vector<Robot> agent_subset = nearestNeighbours(agent, agents, radius, compare);
            robot_counter++;
        }
        auto stop = std::chrono::high_resolution_clock::now();

        auto duration = duration_cast<std::chrono::nanoseconds>(stop - start);
        duration_total += duration;

        if (duration > max) {
            max = duration;
        }

        if (duration < min) {
            min = duration;
        }
    }

    std::chrono::nanoseconds average = duration_total / iterations;
    std::cout << "average time: " << average.count() << " nanoseconds" << std::endl;
    std::cout << "max: " << max.count() << " nanoseconds" << std::endl;
    std::cout << "min: " << min.count() << " nanoseconds" << std::endl;
}

INSTANTIATE_TEST_CASE_P(
        parameters,
        NearestNeighborSearchTest,
        ::testing::Values(
               std::make_tuple(
                       1, std::vector<Robot> robot {TestUtil::createRobotAtPos(point)}
               )));
