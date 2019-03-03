#pragma once

#include "ai/hl/hl.h"
#include "ai/navigator/navigator.h"
#include "ai/primitive/primitive.h"
#include "ai/world/world.h"
#include "util/time/timestamp.h"

/**
 * This class wraps all our AI logic and decision making to help separate our
 * logic from ROS communication as much as possible.
 */
class AI final
{
   public:
    /**
     * Creates a new AI
     */
    explicit AI();

    /**
     * Calculates the Primitives that should be run by our Robots given the current
     * state of the world.
     *
     * @param world The state of the World with which to make the decisions
     *
     * @return the Primitives that should be run by our Robots given the current state
     * of the world.
     */
    std::vector<std::unique_ptr<Primitive>> getPrimitives(const World& world) const;

   private:
    std::unique_ptr<HL> high_level;
    std::unique_ptr<Navigator> navigator;
};
