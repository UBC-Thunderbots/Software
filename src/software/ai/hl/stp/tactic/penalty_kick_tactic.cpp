/**
 * Implementation of the PenaltyKickTactic
 */
#include "software/ai/hl/stp/tactic/penalty_kick_tactic.h"

#include <g3log/g3log.hpp>

#include "shared/constants.h"
#include "software/ai/evaluation/calc_best_shot.h"
#include "software/ai/hl/stp/action/dribble_action.h"
#include "software/ai/hl/stp/action/kick_action.h"
#include "software/ai/hl/stp/action/move_action.h"
#include "software/ai/hl/stp/tactic/tactic_visitor.h"
#include "software/geom/util.h"


PenaltyKickTactic::PenaltyKickTactic(const Ball& ball, const Field& field,
                                     const std::optional<Robot>& enemy_goalie,
                                     bool loop_forever)
    : Tactic(loop_forever), ball(ball), field(field), enemy_goalie(enemy_goalie)
{
    addWhitelistedAvoidArea(AvoidArea::BALL);
    addWhitelistedAvoidArea(AvoidArea::HALF_METER_AROUND_BALL);
    addWhitelistedAvoidArea(AvoidArea::ENEMY_DEFENSE_AREA);
    addWhitelistedAvoidArea(AvoidArea::ENEMY_HALF);
}

std::string PenaltyKickTactic::getName() const
{
    return "Penalty Kick Tactic";
}

void PenaltyKickTactic::updateWorldParams(
    const Ball& updated_ball, const std::optional<Robot>& updated_enemy_goalie,
    const Field& updated_field)
{
    this->enemy_goalie = updated_enemy_goalie;
    this->ball         = updated_ball;
    this->field        = updated_field;
}

double PenaltyKickTactic::calculateRobotCost(const Robot& robot, const World& world)
{
    // We normalize with the total field length so that robots that are within the field
    // have a cost less than 1
    double cost =
        (robot.position() - world.ball().position()).len() / world.field().totalXLength();
    return std::clamp<double>(cost, 0, 1);
}

bool PenaltyKickTactic::evaluate_penalty_shot()
{
    // If there is no goalie, the net is wide open
    if (!enemy_goalie.has_value())
    {
        return true;
    }

    // The value of a penalty shot is proportional to how far away the enemy goalie is
    // from the current shot of the robot

    // We will make a penalty shot if the enemy goalie cannot accelerate in time to block
    // it
    Segment goal_line = Segment(field.enemyGoalpostPos(), field.enemyGoalpostNeg());

    Ray shot_ray = Ray(ball.position(), Point(robot.value().orientation().cos(),
                                              robot.value().orientation().sin()));

    std::optional<Point> intersect_1 = raySegmentIntersection(shot_ray, goal_line).first;

    if (intersect_1.has_value())
    {
        // If we have an intersection, calculate if we have a viable shot

        const double shooter_to_goal_distance =
            (robot.value().position() - intersect_1.value()).len();
        const double time_to_score =
            fabs(shooter_to_goal_distance / PENALTY_KICK_SHOT_SPEED) -
            SSL_VISION_DELAY;  // Include the vision delay in our penalty shot
                               // calculations
        const Point goalie_to_goal_distance =
            (intersect_1.value() = enemy_goalie.value().position());

        // Based on constant acceleration -> // dX = init_vel*t + 0.5*a*t^2
        //          dX - init_vel - (0.5*a*t)t
        const double max_enemy_movement_x = robot.value().velocity().x() * time_to_score +
                                            0.5 * sign(goalie_to_goal_distance.x()) *
                                                PENALTY_KICK_GOALIE_MAX_ACC *
                                                pow(time_to_score, 2);
        const double max_enemy_movement_y = robot.value().velocity().y() * time_to_score +
                                            0.5 * sign(goalie_to_goal_distance.y()) *
                                                PENALTY_KICK_GOALIE_MAX_ACC *
                                                pow(time_to_score, 2);

        // If the position to block the ball is further than the enemy goalie can reach in
        // the time required to score
        if (fabs((goalie_to_goal_distance.x() - max_enemy_movement_x) >
                 ROBOT_MAX_RADIUS_METERS) ||
            fabs((goalie_to_goal_distance.y() - max_enemy_movement_y)) >
                ROBOT_MAX_RADIUS_METERS)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    else
    {
        // If a shot in our current direction will not end in a goal, don't shoot
        return false;
    }
}

Point PenaltyKickTactic::evaluate_next_position()
{
    // Evaluate if the goalie is closer to the negative or positive goalpost

    if (enemy_goalie.has_value())
    {
        double goalie_dist_to_neg_goalpost =
            (field.enemyGoalpostNeg() - enemy_goalie.value().position()).lensq();
        double goalie_dist_to_pos_goalpost =
            (field.enemyGoalpostPos() - enemy_goalie.value().position()).lensq();

        return goalie_dist_to_neg_goalpost > goalie_dist_to_pos_goalpost
                   ? field.enemyGoalpostNeg()
                   : field.enemyGoalpostPos();
    }
    else
    {
        // Return the center of the enemy goal
        return Point(field.enemyGoalpostPos().x(), 0);
    }
}

void PenaltyKickTactic::calculateNextIntent(IntentCoroutine::push_type& yield)
{
    // We will need to keep track of time so we don't break the rules by taking too long
    Timestamp penalty_kick_start = robot->getMostRecentTimestamp();


    MoveAction approach_ball_move_act =
        MoveAction(MoveAction::ROBOT_CLOSE_TO_DEST_THRESHOLD, Angle(), false);
    MoveAction rotate_with_ball_move_act =
        MoveAction(MoveAction::ROBOT_CLOSE_TO_DEST_THRESHOLD, Angle(), false);
    KickAction kick_action = KickAction();

    do
    {
        Vector behind_ball_vector = (ball.position() - field.enemyGoal());
        // A point behind the ball that leaves 5cm between the ball and kicker of the
        // robot
        Point behind_ball = ball.position() +
                            behind_ball_vector.norm(BALL_MAX_RADIUS_METERS +
                                                    DIST_TO_FRONT_OF_ROBOT_METERS + 0.04);

        // If we haven't approached the ball yet, get close

        if ((robot.value().position() - behind_ball).len() <=
                MoveAction::ROBOT_CLOSE_TO_DEST_THRESHOLD &&
            (robot.value()
                 .orientation()
                 .minDiff((-behind_ball_vector).orientation())
                 .toDegrees() < 5.0))
        {
            if (evaluate_penalty_shot())
            {
                yield(kick_action.updateStateAndGetNextIntent(
                    *robot, ball, ball.position(), robot.value().orientation(),
                    PENALTY_KICK_SHOT_SPEED));
            }
        }
        else if (!approach_ball_move_act.done())
        {
            yield(approach_ball_move_act.updateStateAndGetNextIntent(
                *robot, behind_ball, (-behind_ball_vector).orientation(), 0,
                DribblerEnable::ON, MoveType::NORMAL, AutokickType::NONE));
        }
        else
        {
            const Point next_shot_position = evaluate_next_position();
            const Angle next_angle = (next_shot_position - ball.position()).orientation();
            yield(rotate_with_ball_move_act.updateStateAndGetNextIntent(
                *robot, robot.value().position(), next_angle, 0, DribblerEnable::ON,
                MoveType::NORMAL, AutokickType::NONE));
        }

    } while (
        !(kick_action.done() ||
          (penalty_kick_start - robot->getMostRecentTimestamp()) < penalty_shot_timeout));
}

void PenaltyKickTactic::accept(TacticVisitor& visitor) const
{
    visitor.visit(*this);
}
