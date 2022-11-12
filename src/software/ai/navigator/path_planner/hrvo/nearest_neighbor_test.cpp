#include <gtest/gtest.h>

#include "software/ai/navigator/path_planner/hrvo/brute_force_nearest_neighbor_search.h"
#include "software/geom/algorithms/distance.h"
#include "software/test_util/test_util.h"
#include <random>
#include <chrono>

class Test_Agent {
public:
    Point point;
    Point position() const;
    Test_Agent(double x, double y);
    bool operator==(const Test_Agent &other) const;
    bool operator!=(const Test_Agent &other) const;
};

bool Test_Agent::operator==(const Test_Agent &other) const
{
    return this->position() == other.position();
}

bool Test_Agent::operator!=(const Test_Agent &other) const
{
    return !(*this == other);
}

Point Test_Agent::position() const {
    return point;
}

Test_Agent::Test_Agent(double x, double y) {
    point = Point(x,y);
}

double compare(const Test_Agent &r1, const Test_Agent &r2) {
    return distanceSquared(r1.position(), r2.position());
}

template <class T, typename F>
std::vector<T> nearestNeighbours(const T& this_robot, const std::vector<T>& input,
                                 double radius, F comparator)
{
    std::vector<T> robot_subset;
    for (const T& candidate_robot : input)
    {
        if (comparator(this_robot, candidate_robot) < radius * radius &&
            this_robot != candidate_robot)
        {
            robot_subset.push_back(candidate_robot);
        }
    }
    return robot_subset;
}


TEST(NearestNeighborSearchTest, generic_frnn_brute_force_test)
{
    Test_Agent agent(0, 0);
    std::vector<Test_Agent> agents = {agent};

    std::vector<Test_Agent> agent_subset =
            nearestNeighbours(agent, agents, 2.0, compare);
}

//TEST(NearestNeighborSearchTest, generic_frnn_brute_force_test)
//{
//    unsigned int iterations = 1000;
//    unsigned int num_of_agents = 22;
//    unsigned int friendly_agents = 11;
//    double radius = 1.0;
//    double lower_x_bound = -4.5;
//    double upper_x_bound = 4.5;
//    double lower_y_bound = -3;
//    double upper_y_bound = 3;
//    std::chrono::nanoseconds duration_total(0);
//    std::chrono::nanoseconds max(0);
//    std::chrono::nanoseconds min(0);
//
//    std::uniform_real_distribution<double> x_unif(lower_x_bound,upper_x_bound);
//    std::uniform_real_distribution<double> y_unif(lower_y_bound,upper_y_bound);
//    std::default_random_engine re;
//
//    for (unsigned int i = 0; i < iterations; i++) {
//        std::vector<Test_Agent> agents;
//        for (unsigned int j = 0; j < num_of_agents; j++) {
//            double random_x = x_unif(re);
//            double random_y = y_unif(re);
//            std::cout << "random x value: " << random_x << std::endl;
//            std::cout << "random y value: " << random_y << std::endl;
//            Test_Agent agent(random_x, random_y);
//            agents.push_back(agent);
//        }
//
//        unsigned int robot_counter = 0;
//        auto start = std::chrono::high_resolution_clock::now();
//        for (const Test_Agent &agent : agents) {
//            if (robot_counter >= friendly_agents) {
//                break;
//            }
//            std::vector<Test_Agent> agent_subset = nearestNeighbours(agent, agents, radius, compare);
//            robot_counter++;
//        }
//        auto stop = std::chrono::high_resolution_clock::now();
//
//        auto duration = duration_cast<std::chrono::nanoseconds>(stop - start);
//        duration_total += duration;
//
//        if (duration > max) {
//            max = duration;
//        }
//
//        if (duration < min) {
//            min = duration;
//        }
//    }
//
//    std::chrono::nanoseconds average = duration_total / iterations;
//    std::cout << "average time: " << average.count() << " nanoseconds" << std::endl;
//    std::cout << "max: " << max.count() << " nanoseconds" << std::endl;
//    std::cout << "min: " << min.count() << " nanoseconds" << std::endl;
//}