#pragma once

#include "ai/world/robot.h"
#include "ai/world/team.h"

namespace Evaluation
{
    /**
     * Given a team, finds the robot on that team that is closest to a reference point.
     *
     * @param team
     * @param ref_point The point where the distance to each robot will be measured.
     * @return Robot that is closest to the reference point.
     */
    std::optional<Robot> nearest_robot(const Team team, const Point ref_point);

};  // namespace Evaluation
