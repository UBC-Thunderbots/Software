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


