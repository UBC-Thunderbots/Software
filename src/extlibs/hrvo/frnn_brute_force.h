#pragma once

#include "extlibs/hrvo/hrvo_agent.h"

class FRNN {
    public:
    static std::vector<std::pair<double, double>> queryClosestNeighbors(size_t index, float radius, std::vector<std::pair<double, double>> &agents);

    template<class T>
    static std::vector<T> nearestNeighbours(T this_robot, std::vector<T> input, double radius, std::function<std::vector<T>(T, std::vector<T>, double)> comparator);

    template <class T>
    static std::vector<T> compare_robots(T robot, std::vector<T> robots, double radius);
};
