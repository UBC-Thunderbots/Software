#ifndef EVALUATION_GET_SHOOT_SCORE_H
#define EVALUATION_GET_SHOOT_SCORE_H

#include "geom/angle.h"
#include "ai/world/field.h"
#include "ai/world/team.h"
#include "ai/world/robot.h"

namespace Evaluation {

    typedef struct ShotDirAndScore {
        double score;
        Vector shot_direction;
    };

  ShotDirAndScore getFriendlyRobotBestShotGoal(Robot shooter, Team friendly_team, Team enemy_team, Field field);
  ShotDirAndScore getEnemyRobotBestShotGoal(Robot shooter, Team friendly_team, Team enemy_team, Field field);

};
#endif //EVALUATION_GET_SHOOT_SCORE_H
