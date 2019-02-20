#include "detect_threat.h"
#include "shared/constants.h"
#include <optional>
#include "geom/util.h"
#include "ai/world/field.h"

std::optional<Point> BallThreat::calc_ball_vel_intersect_friendly_net(Ball ball, Field field) {

    return velocity_line_intersection(ball.velocity(), ball.position(), field.friendlyGoalpostNeg(), field.friendlyGoalpostPos(), Rectangle(field.friendlyCornerPos(), field.enemyCornerNeg()));

}

std::optional<Point> BallThreat::calc_ball_vel_intersect_enemy_net(Ball ball, Field field) {


    return velocity_line_intersection(ball.velocity(), ball.position(), field.enemyGoalpostNeg(), field.enemyGoalpostPos(), Rectangle(field.friendlyCornerPos(), field.enemyCornerNeg()));

}
