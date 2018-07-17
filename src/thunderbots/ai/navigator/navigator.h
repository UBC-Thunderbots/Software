#ifndef AI_NAGIVATOR_NAVIGATOR_H_
#define AI_NAGIVATOR_NAVIGATOR_H_

#include "ai/intent.h"
#include "ai/primitive/primitive.h"
#include "ai/world/world.h"

/**
 * An abstraction for all navigation operations performed by our AI. The navigator is
 * responsible for all navigation,
 * path-planning, and collision-avoidance for our robots.
 *
 * This is an Abstract, pure-virtual class. It is meant to define the interace that all
 * Navigator modules must follow.
 * Other classes should inherit from this class and implement the methods to create a
 * useable Navigator class. This
 * allows us to potentially have multiple different navigators that we can swap out or
 * combine at runtime.
 */
class Navigator
{
   public:
    virtual std::map<unsigned int, Primitive> getAssignedPrimitives(
        const std::vector<std::pair<unsigned int, Intent>> &assignedIntents,
        const World &world) = 0;

   private:
    // TODO:
    // plan paths function
    // collision avoidance function
};


#endif  // AI_NAGIVATOR_NAVIGATOR_H_
