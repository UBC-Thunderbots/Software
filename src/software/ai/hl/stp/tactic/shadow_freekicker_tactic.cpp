#include "software/ai/hl/stp/tactic/shadow_freekicker_tactic.h"

#include <algorithm>

#include "shared/constants.h"
#include "software/ai/evaluation/possession.h"
#include "software/ai/hl/stp/tactic/tactic_visitor.h"
#include "software/util/parameter/dynamic_parameters.h"

ShadowFreekickerTactic::ShadowFreekickerTactic(FreekickShadower free_kick_shadower,
                                               Team enemy_team, Ball ball, Field field,
                                               bool loop_forever)
    : Tactic(loop_forever),
      free_kick_shadower(free_kick_shadower),
      enemy_team(enemy_team),
      ball(ball),
      field(field)
{
}

std::string ShadowFreekickerTactic::getName() const
{
    return "Shadow Freekick Tactic";
}

void ShadowFreekickerTactic::updateWorldParams(Team enemy_team, Ball ball)
{
    this->enemy_team = enemy_team;
    this->ball       = ball;
}

double ShadowFreekickerTactic::calculateRobotCost(const Robot &robot, const World &world)
{
    double cost =
        (robot.position() - world.ball().position()).len() / world.field().totalXLength();
    return std::clamp<double>(cost, 0, 1);
}
void ShadowFreekickerTactic::calculateNextIntent(IntentCoroutine::push_type &yield)
{
    MoveAction move_action = MoveAction(MoveAction::ROBOT_CLOSE_TO_DEST_THRESHOLD);
    Point defend_position  = robot->position();

    do
    {
        std::optional<Robot> enemy_with_ball =
            Evaluation::getRobotWithEffectiveBallPossession(enemy_team, ball, field);
        double robot_separation_scaling_factor =
            Util::DynamicParameters->getShadowFreekickerTacticConfig()
                ->RobotSeparationScalingFactor()
                ->value();

        if (enemy_with_ball.has_value())
        {
            const Vector enemy_pointing_direction =
                (ball.position() - enemy_with_ball->position())
                    .norm(FREE_KICK_MAX_PROXIMITY + ROBOT_MAX_RADIUS_METERS);

            Vector perpendicular_to_enemy_direction =
                enemy_pointing_direction.perp().norm(ROBOT_MAX_RADIUS_METERS *
                                                     robot_separation_scaling_factor);

            defend_position = free_kick_shadower == FreekickShadower::RIGHT
                                  ? ball.position() + enemy_pointing_direction +
                                        perpendicular_to_enemy_direction
                                  : ball.position() + enemy_pointing_direction -
                                        perpendicular_to_enemy_direction;
        }
        else
        {
            const Vector ball_to_net_direction =
                (field.friendlyGoal() - ball.position())
                    .norm(FREE_KICK_MAX_PROXIMITY + ROBOT_MAX_RADIUS_METERS);

            Vector perpendicular_to_ball_direction = ball_to_net_direction.perp().norm(
                ROBOT_MAX_RADIUS_METERS * robot_separation_scaling_factor);

            defend_position = free_kick_shadower == FreekickShadower::RIGHT
                                  ? ball.position() + ball_to_net_direction +
                                        perpendicular_to_ball_direction
                                  : ball.position() + ball_to_net_direction -
                                        perpendicular_to_ball_direction;
        }

        yield(move_action.updateStateAndGetNextIntent(
            *robot, defend_position, (ball.position() - robot->position()).orientation(),
            0, DribblerEnable::OFF, MoveType::NORMAL, AutokickType::NONE));
    } while (true);
}

void ShadowFreekickerTactic::accept(TacticVisitor &visitor) const
{
    visitor.visit(*this);
}
