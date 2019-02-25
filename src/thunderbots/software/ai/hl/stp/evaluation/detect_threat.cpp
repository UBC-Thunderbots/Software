/**
 * This file contains independent Evaluation functions to evaluate threats to the friendly
 * team on the playing field
 */

#include "detect_threat.h"

#include <optional>

#include "ai/world/field.h"
#include "geom/util.h"
#include "shared/constants.h"

namespace Evaluation
{
    std::optional<Point> calc_ball_vel_intersect_friendly_net(Ball ball, Field field)
    {
        return velocity_line_intersection(
            ball.velocity(), ball.position(), field.friendlyGoalpostNeg(),
            field.friendlyGoalpostPos(),
            Rectangle(field.friendlyCornerPos(), field.enemyCornerNeg()));
    }

    std::optional<Point> calc_ball_vel_intersect_enemy_net(Ball ball, Field field)
    {
        return velocity_line_intersection(
            ball.velocity(), ball.position(), field.enemyGoalpostNeg(),
            field.enemyGoalpostPos(),
            Rectangle(field.friendlyCornerPos(), field.enemyCornerNeg()));
    }

};  // namespace Evaluation
