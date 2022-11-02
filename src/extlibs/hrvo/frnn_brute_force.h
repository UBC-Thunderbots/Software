#pragma once

#include "extlibs/hrvo/hrvo_agent.h"

class FRNN {
    public:
    static std::vector<std::pair<double, double>> queryClosestNeighbors(size_t index, float radius, std::vector<std::pair<double, double>> &agents);

    template<class T, class F>
    static std::vector<T> nearestNeighbours(const T& this_robot, const std::vector<T>& input, double radius, F comparator);
};
