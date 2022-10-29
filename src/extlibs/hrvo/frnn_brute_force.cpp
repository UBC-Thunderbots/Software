#include "frnn_brute_force.h"

std::vector<std::pair<double, double>> FRNN::queryClosestNeighbors(size_t index, float radius, std::vector<std::pair<double, double>> &agents) {

    std::vector<std::pair<double, double>> neighbors;

    for (std::pair<double, double> other_agent : agents) {

        double x_difference = other_agent.first - agents[index].first;
        double y_difference = other_agent.second - agents[index].second;
        double length_squared = x_difference * x_difference + y_difference * y_difference;
        double distance = std::sqrt(length_squared);

        if (distance < radius && other_agent != agents[index]) {
            neighbors.push_back(other_agent);
        }
    }

    return neighbors;
}

template <class T>
std::vector<T> FRNN::nearestNeighbours(T this_robot, std::vector<T> input, double radius, std::function<std::vector<T>(T, std::vector<T>, double)> comparator) {
    return comparator(this_robot, input, radius);
}

template <class T>
std::vector<T> FRNN::compare_robots(T robot, std::vector<T> robots, double radius) {
    std::vector<T> robot_subset;

    for (T candidate_robot: robots) {
        if ((robot.postion() - candidate_robot.position()).lengthSquared() <  radius * radius && robot != candidate_robot) {
            robot_subset.push_back(candidate_robot);
        }
    }

    return robot_subset;
}

/**
 * Lambda Function Example?
 * nearestNeighbours(robots, 8.5, [](Robot r1, Robot r2) {
 *  return (r1.position() - r2.position()).lengthSquared();
 * }
 *
 * lambda functions: std:; function<T, T> -> double (distance)
 *
 * TODO: Add additional tests for the KDtree by timing it while running in the simulator, have it run for a minute. Run AI vs Ai simulate test with (./tbots.py run thunderscope)
 * TODO: Move the implementation of FRNN into the algorithms folder
 */


