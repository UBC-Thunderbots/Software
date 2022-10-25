#pragma once

#include "extlibs/hrvo/hrvo_agent.h"

class FRNN {
    public:
    static std::vector<std::pair<double, double>> queryClosestNeighbors(size_t index, float radius, std::vector<std::pair<double, double>> &agents);
};
