#include "frnn_brute_force.h"

std::vector<std::pair<double, double>> queryClosestNeighbors(
    size_t index, float radius, std::vector<std::pair<double, double>> &agents)
{
    std::vector<std::pair<double, double>> neighbors;

    for (const std::pair<double, double> &other_agent : agents)
    {
        double x_difference   = other_agent.first - agents[index].first;
        double y_difference   = other_agent.second - agents[index].second;
        double length_squared = x_difference * x_difference + y_difference * y_difference;
        double distance       = std::sqrt(length_squared);

        if (distance < radius && other_agent != agents[index])
        {
            neighbors.push_back(other_agent);
        }
    }

    return neighbors;
}

template <class T, typename F>
std::vector<T> nearestNeighbours(const T &this_robot, const std::vector<T> &input,
                                 double radius, F comparator)
{
    std::vector<T> robot_subset;
    for (const T &candidate_robot : input)
    {
        if (comparator(this_robot, candidate_robot) < radius * radius &&
            this_robot != candidate_robot)
        {
            robot_subset.push_back(candidate_robot);
        }
    }
    return robot_subset;
}

/**
 * TODO: Add additional tests for the KDtree by timing it while running in the simulator,
 * have it run for a minute. Run AI vs AI test with (./tbots.py run thunderscope)
 * TODO: Move the implementation of FRNN into the algorithms folder
 */
