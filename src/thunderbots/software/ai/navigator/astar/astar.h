#pragma once

#include "ai/navigator/navigator.h"

class AStarNav : public Navigator
{
   public:
    /**
     * Creates a new Navigator that uses RRT (Rapidly Expanding Random Trees)
     * to generate paths
     */
    AStarNav();

    std::vector<std::unique_ptr<Primitive>> getAssignedPrimitives(
        const World &world,
        const std::vector<std::unique_ptr<Intent>> &assignedIntents) const override;
};
