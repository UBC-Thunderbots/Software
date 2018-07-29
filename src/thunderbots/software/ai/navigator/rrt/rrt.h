#pragma once

#include "ai/navigator/navigator.h"

class RRTNav : public Navigator
{
   public:
    RRTNav();

    std::map<unsigned int, Primitive> getAssignedPrimitives(
        const std::vector<std::pair<unsigned int, Intent>> &assignedIntents,
        const World &world) override;
};
