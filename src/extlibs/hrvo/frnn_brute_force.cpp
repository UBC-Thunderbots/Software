//
// Created by joshua on 2022-10-21.
//

#include "frnn_brute_force.h"

std::vector<std::shared_ptr<Agent>> FRNN::queryClosestNeighbors(HRVOAgent *agent, float radius, std::vector<std::shared_ptr<Agent>> &agents) const {

    std::vector<std::shared_ptr<Agent>> neighbors;

    for (std::shared_ptr<Agent> other_agent : agents) {

        double x_difference = other_agent->getPosition().x() - agent->getPosition().x();
        double y_difference = other_agent->getPosition().y() - agent->getPosition().y();
        double length_squared = x_difference * x_difference + y_difference * y_difference;
        double distance = std::sqrt(length_squared);

        if (distance < radius && agent != other_agent.get()) {
            neighbors.push_back(other_agent);
        }
    }

    return neighbors;
}


