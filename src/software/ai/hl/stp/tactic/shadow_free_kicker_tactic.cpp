#include "software/ai/hl/stp/tactic/shadow_free_kicker_tactic.h"

#include <algorithm>

#include "shared/constants.h"
#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/ai/evaluation/possession.h"

ShadowFreekickerTactic::ShadowFreekickerTactic(FreekickShadower free_kick_shadower,
                                               Team enemy_team, Ball ball, Field field,
                                               bool loop_forever)
    : Tactic(loop_forever, {RobotCapability::Move}),
      free_kick_shadower(free_kick_shadower),
      enemy_team(enemy_team),
      ball(ball),
      field(field)
{
}

void ShadowFreekickerTactic::updateWorldParams(const World &world)
{
    this->enemy_team = world.enemyTeam();
    this->ball       = world.ball();
}

double ShadowFreekickerTactic::calculateRobotCost(const Robot &robot,
                                                  const World &world) const
{
    double cost = (robot.position() - world.ball().position()).length() /
                  world.field().totalXLength();
    return std::clamp<double>(cost, 0, 1);
}
void ShadowFreekickerTactic::calculateNextAction(ActionCoroutine::push_type &yield)
{
    auto move_action      = std::make_shared<MoveAction>(false);
    Point defend_position = robot_->position();
    // Experimentally determined to be a reasonable value
    double robot_separation_scaling_factor = 1.1;

    do
    {
        std::optional<Robot> enemy_with_ball =
            getRobotWithEffectiveBallPossession(enemy_team, ball, field);

        if (enemy_with_ball.has_value())
        {
            const Vector enemy_pointing_direction =
                (ball.position() - enemy_with_ball->position())
                    .normalize(FREE_KICK_MAX_PROXIMITY + ROBOT_MAX_RADIUS_METERS);

            Vector perpendicular_to_enemy_direction =
                enemy_pointing_direction.perpendicular().normalize(
                    ROBOT_MAX_RADIUS_METERS * robot_separation_scaling_factor);

            defend_position = free_kick_shadower == FreekickShadower::RIGHT
                                  ? ball.position() + enemy_pointing_direction +
                                        perpendicular_to_enemy_direction
                                  : ball.position() + enemy_pointing_direction -
                                        perpendicular_to_enemy_direction;
        }
        else
        {
            const Vector ball_to_net_direction =
                (field.friendlyGoalCenter() - ball.position())
                    .normalize(FREE_KICK_MAX_PROXIMITY + ROBOT_MAX_RADIUS_METERS);

            Vector perpendicular_to_ball_direction =
                ball_to_net_direction.perpendicular().normalize(
                    ROBOT_MAX_RADIUS_METERS * robot_separation_scaling_factor);

            defend_position = free_kick_shadower == FreekickShadower::RIGHT
                                  ? ball.position() + ball_to_net_direction +
                                        perpendicular_to_ball_direction
                                  : ball.position() + ball_to_net_direction -
                                        perpendicular_to_ball_direction;
        }

        move_action->updateControlParams(
            *robot_, defend_position,
            (ball.position() - robot_->position()).orientation(), 0, DribblerMode::OFF,
            BallCollisionType::AVOID);
        yield(move_action);
    } while (true);
}

void ShadowFreekickerTactic::accept(TacticVisitor &visitor) const
{
    visitor.visit(*this);
}

Ball ShadowFreekickerTactic::getBall() const
{
    return this->ball;
}

Field ShadowFreekickerTactic::getField() const
{
    return this->field;
}

Team ShadowFreekickerTactic::getEnemyTeam() const
{
    return this->enemy_team;
}
