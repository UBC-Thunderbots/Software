/**
 * Implementation of the PenaltyKickTactic
 */
#include "penalty_kick_tactic.h"

#include "ai/hl/stp/action/dribble_action.h"
#include "ai/hl/stp/action/kick_action.h"
#include "ai/hl/stp/action/move_action.h"
#include "ai/hl/stp/evaluation/calc_best_shot.h"
#include "geom/util.h"
#include "shared/constants.h"
#include "util/logger/init.h"


PenaltyKickTactic::PenaltyKickTactic(const Ball& ball, const Field& field,
                                     const std::optional<Robot>& enemy_goalie,
                                     bool loop_forever)
    : ball(ball), field(field), enemy_goalie(enemy_goalie), Tactic(loop_forever)
{
}

std::string PenaltyKickTactic::getName() const
{
    return "Penalty Kick Tactic";
}

void PenaltyKickTactic::updateParams(const Ball& updated_ball,
                                     const std::optional<Robot>& updated_enemy_goalie,
                                     const Field& updated_field)
{
    this->enemy_goalie = updated_enemy_goalie;
    this->ball         = updated_ball;
    this->field        = updated_field;
}

double PenaltyKickTactic::calculateRobotCost(const Robot& robot, const World& world)
{
    // Prefer robots closer to the pass start position
    // We normalize with the total field length so that robots that are within the field
    // have a cost less than 1
    double cost =
        (robot.position() - world.ball().position()).len() / world.field().totalLength();
    return std::clamp<double>(cost, 0, 1);
}

void PenaltyKickTactic::calculateNextIntent(IntentCoroutine::push_type& yield)
{
    // Keep track if a shot has been taken
    bool shot_taken             = false;

    MoveAction get_behind_ball_act = MoveAction();
    KickAction kick_ball_act = KickAction();

    do {

        Vector behind_ball_direction = (ball.position() - field.enemyGoalpostPos() + Point(0,-0.20)).norm();
        Point behind_ball = ball.position() + behind_ball_direction.norm(DIST_TO_FRONT_OF_ROBOT_METERS + BALL_MAX_RADIUS_METERS + 0.02);
        Angle shot_orientation = (-1*behind_ball_direction).orientation();

        if( ( robot->position() - behind_ball ).len() >= MoveAction::ROBOT_CLOSE_TO_DEST_THRESHOLD || (robot->orientation().minDiff(shot_orientation) > Angle::ofDegrees(2))) {
            yield(get_behind_ball_act.updateStateAndGetNextIntent(*robot, behind_ball, shot_orientation, 0, true, false, AutokickType::NONE));
        }
        else {
            Point shot_location = field.enemyGoalpostPos() + Point(0,-0.20);
            yield(kick_ball_act.updateStateAndGetNextIntent(*robot, ball, ball.position(), field.enemyGoal(), PENALTY_KICK_SHOT_SPEED ));
        }
    } while(true);
}
