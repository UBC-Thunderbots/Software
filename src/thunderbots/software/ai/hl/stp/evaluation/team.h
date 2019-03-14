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
     * @return Robot on friendly team that is closest to the reference point.
     */
    std::optional<Robot> nearest_friendly(const Team friendly_team,
                                          const Point ref_point);

    /**
     * Finds the robot on the enemy team that is closest to some reference point.
     *
     * @param enemy_team
     * @param ref_point The point where the distance to each robot will be measured.
     *
     * @return Robot on enemy team that is closest to the reference point.
     */
    std::optional<Robot> nearest_enemy(const Team enemy_team, const Point ref_point);

    /**
     * Given a team, finds the robot on that team that is closest to a reference point.
     *
     * @param team
     * @param ref_point The point where the distance to each robot will be measured.
     * @return Robot that is closest to the reference point.
     */
    std::optional<Robot> nearest_robot(const Team team, const Point ref_point);

};  // namespace Evaluation
