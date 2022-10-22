//
// Created by joshua on 2022-10-21.
//

#pragma once

#include "extlibs/hrvo/hrvo_agent.h"

class FRNN {
    public:
        std::vector<std::shared_ptr<Agent>> queryClosestNeighbors(HRVOAgent *agent, float radius, std::vector<std::shared_ptr<Agent>> &agents) const;
        //create a function that returns a dictionary of bin -> neighbors
};
