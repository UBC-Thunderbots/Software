#pragma once

#include "ai/world/robot.h"
#include "ai/world/team.h"

namespace Evaluation
{
    /**
     * Finds the robot on the friendly team that is closest to some reference point.
     *
     * @param friendly_team
     * @param ref_point The point where the distance to each robot will be measured.
     *
     * @return Robot that is closest to the reference point.
     */
    std::optional<Robot> nearest_friendly(const Team friendly_team,
                                          const Point ref_point);

};  // namespace Evaluation
