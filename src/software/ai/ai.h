#pragma once

#include "software/ai/hl/hl.h"
#include "software/ai/hl/stp/play_info.h"
#include "software/ai/navigator/navigator.h"
#include "software/ai/primitive/primitive.h"
#include "software/util/time/timestamp.h"
#include "software/world/world.h"

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

    /**
     * Returns information about the currently running plays and tactics, including the
     * name of the play, and which robots are running which tactics
     *
     * @return information about the currently running plays and tactics
     */
    PlayInfo getPlayInfo() const;

    std::shared_ptr<Navigator> getNavigator() const;

   private:
    std::shared_ptr<Navigator> navigator;
    std::unique_ptr<HL> high_level;
};
