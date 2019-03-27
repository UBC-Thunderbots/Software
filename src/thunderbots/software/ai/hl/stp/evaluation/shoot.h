#pragma once

#include "ai/world/field.h"
#include "ai/world/robot.h"
#include "ai/world/team.h"
#include "geom/angle.h"

namespace Evaluation
{
    struct ShotDirAndScore
    {
        double score;
        Vector shot_direction;
    };

    ShotDirAndScore getFriendlyRobotBestShotGoal(Robot shooter,
                                                 std::vector<Point> all_obstacles,
                                                 Field field);
    ShotDirAndScore getEnemyRobotBestShotGoal(Robot shooter,
                                              std::vector<Point> all_obstacles,
                                              Field field);

};  // namespace Evaluation
