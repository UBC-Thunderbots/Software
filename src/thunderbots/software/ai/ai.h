#pragma once

#include "ai/hl/stp/stphl.h"
#include "ai/navigator/rrt/rrt.h"
#include "ai/primitive/primitive.h"
#include "ai/world/world.h"

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
     * @return the Primitives that should be run by our Robots given the current state
     * of the world.
     */
    std::vector<std::unique_ptr<Primitive>> getPrimitives() const;

    // The AI's view of the state of the world. We leave it public to make it easy to
    // update from the external ROS layer.
    World world;

   private:
    STPHL stp_high_level;
    RRTNav rrt_navigator;
};
