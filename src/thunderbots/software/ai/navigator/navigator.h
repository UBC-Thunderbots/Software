#pragma once

#include "ai/intent/intent.h"
#include "ai/primitive/primitive.h"
#include "ai/world/world.h"

/**
 * An abstraction for all navigation operations performed by our AI. The navigator is
 * responsible for all navigation, path-planning, and collision-avoidance for our robots.
 *
 * This is an Abstract, pure-virtual class. It is meant to define the interface that all
 * Navigator modules must follow. Other classes should inherit from this class and
 * implement the methods to create a useable Navigator class. This allows us to
 * potentially have multiple different navigators that we can swap out or combine at
 * runtime.
 */
class Navigator
{
   public:
    /**
     * Given the state of the world and a list of Intents the robots want to perform,
     * returns a list of Primitives that our Robots should perform in order to
     * work towards achieving their Intent.
     *
     * @param world The current state of the world
     * @param assignedIntents A list of Intents assigned to our friendly robots
     * @return A list of Primitives to be run by our Robots in order to work towards
     * achieving their Intents
     */
    virtual std::vector<std::unique_ptr<Primitive>> getAssignedPrimitives(
        const World &world,
        const std::vector<std::unique_ptr<Intent>> &assignedIntents) = 0;

    virtual ~Navigator() = default;
};
