#pragma once

#include "software/ai/passing/pass_with_rating.h"
#include "software/world/world.h"

/**
 * The base class for a pass generator
 */
class BasePassGenerator
{
   public:
    virtual ~BasePassGenerator() {}

    /**
     * Gets the best pass currently on the field using the given world
     *
     * @param world The current state of the world used to evaluate the best pass
     *
     * @return a single best pass with its rating
     */
    virtual PassWithRating getBestPass(const World& world) = 0;
};
