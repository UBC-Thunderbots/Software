#include "ai/hl/stp/tactic/shadow_freekicker.h"

#include <algorithm>

#include "ai/hl/stp/evaluation/possession.h"
#include "shared/constants.h"


ShadowFreekickerTactic::ShadowFreekickerTactic(FreekickShadower free_kick_shadower, Team enemy_team, Ball ball, Field field,  bool loop_forever) : free_kick_shadower(free_kick_shadower), enemy_team(enemy_team), ball(ball), field(field), Tactic(loop_forever)
{
}

std::string ShadowFreekickerTactic::getName() const
{
    return "Shadow Freekick Tactic";
}

void ShadowFreekickerTactic::updateParams(Team enemy_team, Ball ball)
{
    this->enemy_team = enemy_team;
    this->ball = ball;
}

double ShadowFreekickerTactic::calculateRobotCost(const Robot& robot ,const World &world)
{
    double cost = (robot.position() - world.ball().position()).len() /
                  world.field().totalLength();
    return std::clamp<double>(cost, 0, 1);
}
void ShadowFreekickerTactic::calculateNextIntent(IntentCoroutine::push_type &yield)
{

    MoveAction move_action = MoveAction(MoveAction::ROBOT_CLOSE_TO_DEST_THRESHOLD);
    Point defend_position = robot->position();

    do {
        std::optional<Robot> enemy_with_ball = Evaluation::getRobotWithEffectiveBallPossession(enemy_team, ball, field);

        if(enemy_with_ball.has_value()) {

            const Vector enemy_pointing_direction = (ball.position() - enemy_with_ball->position()).norm(
                    FREE_KICK_MAX_PROXIMITY + ROBOT_MAX_RADIUS_METERS);

            Vector perpendicular_to_enemy_direction = enemy_pointing_direction.perp().norm(ROBOT_MAX_RADIUS_METERS);

            defend_position = free_kick_shadower == FreekickShadower::First ? enemy_pointing_direction +
                                                                                          perpendicular_to_enemy_direction
                                                                                        : enemy_pointing_direction -
                                                                                          perpendicular_to_enemy_direction;
        }
        else {

            const Vector ball_to_net_direction = (field.friendlyGoal() - ball.position()).norm(
                    FREE_KICK_MAX_PROXIMITY + ROBOT_MAX_RADIUS_METERS);

            Vector perpendicular_to_ball_direction = ball_to_net_direction.perp().norm(ROBOT_MAX_RADIUS_METERS);

            defend_position = free_kick_shadower == FreekickShadower::First ? ball_to_net_direction +
                                                                                          perpendicular_to_ball_direction
                                                                                        : ball_to_net_direction -
                                                                                          perpendicular_to_ball_direction;
        }

        yield(move_action.updateStateAndGetNextIntent(*robot, defend_position, (ball.position() - robot->position()).orientation(), 0, false ));
    } while(!move_action.done());



        // Calculate enemy direction
        //      Move 20cm away from the ball in this direction
        //  Calc 1 robot radii perpendicular to the ball
        // Depending on first/second robot decide on a direction
    }
//    StopAction stop_action =
//            StopAction(StopAction::ROBOT_STOPPED_SPEED_THRESHOLD_DEFAULT, false);
//    do
//    {
//        yield(stop_action.updateStateAndGetNextIntent(*robot, this->coast));
//    } while (!stop_action.done());
}
