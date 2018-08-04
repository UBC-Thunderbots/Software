#pragma once

#include "ai/navigator/navigator.h"

class RRTNav : public Navigator
{
   public:
    /**
     * Creates a new Navigator that uses RRT (Rapidly Expanding Random Trees)
     * to generate paths
     */
    RRTNav();

    std::vector<std::unique_ptr<Primitive>> getAssignedPrimitives(
        const World &world,
        const std::vector<std::unique_ptr<Intent>> &assignedIntents) const override;
};
