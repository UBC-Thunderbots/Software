#pragma once

#include "software/world/robot.h"
#include "software/world/robot_state.h"

/**
 * Contains shared gameplay-related calculations.
 */
class Strategy
{
public:
    /**
     * Get a map of Robot to the best dribble locations.
     */
    std::map<Robot, Point> getBestDribbleLocations(const std::vector<Robot>& robots);

    /**
     * Get a map of Robot to the best pass locations.
     */
    std::map<Robot, Pass> getBestPasses(const std::vector<Robot>& robots);

    void reset();
}
