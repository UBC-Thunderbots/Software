#include "ai/hl/stp/tactic/goalie_tactic.h"

#include "ai/hl/stp/action/chip_action.h"
#include "ai/hl/stp/action/move_action.h"
#include "ai/hl/stp/evaluation/calc_best_shot.h"
#include "geom/point.h"
#include "geom/ray.h"
#include "geom/segment.h"
#include "geom/util.h"
#include "shared/constants.h"

GoalieTactic::GoalieTactic(const Ball &ball, const Field &field,
                           const Team &friendly_team, const Team &enemy_team)
    : ball(ball),
      field(field),
      friendly_team(friendly_team),
      enemy_team(enemy_team),
      Tactic(true)
{
}

std::string GoalieTactic::getName() const
{
    return "Goalie Tactic";
}

void GoalieTactic::updateParams(
    const Ball &ball, const Field &field, const Team &friendly_team,
    const Team &enemy_team, const std::optional<Evaluation::EnemyThreat> &enemy_threat)
{
    // Update the parameters stored by this Tactic
    this->ball          = ball;
    this->field         = field;
    this->friendly_team = friendly_team;
    this->enemy_team    = enemy_team;
    this->enemy_threat  = enemy_threat;
}

double GoalieTactic::calculateRobotCost(const Robot &robot, const World &world)
{
    // Strongly prefer the robot assigned to be the goalie.
    // TODO: This is a hack to "ensure" the right robot will be assigned. We should
    // normally return values in the range [0, 1]
    if (world.friendlyTeam().getGoalieID() &&
        robot.id() == world.friendlyTeam().getGoalieID().value())
    {
        return 0.0;
    }
    else
    {
        return 1000000;
    }
}

void GoalieTactic::calculateNextIntent(IntentCoroutine::push_type &yield)
{
    MoveAction move_action = MoveAction();
    ChipAction chip_action = ChipAction();
    do
    {
        Ray ball_ray = Ray(ball.position(), ball.velocity());

        // Create a segment along the goal line, slightly shortened to account for the
        // robot radius so as we move along the segment we don't try run into the goal
        // posts
        Point p1                        = Point(field.friendlyGoalpostNeg().x(),
                         field.friendlyGoalpostNeg().y() + ROBOT_MAX_RADIUS_METERS);
        Point p2                        = Point(field.friendlyGoalpostPos().x(),
                         field.friendlyGoalpostPos().y() - ROBOT_MAX_RADIUS_METERS);
        Segment goalie_movement_segment = Segment(p1, p2);
        Segment full_goal_segment =
            Segment(field.friendlyGoalpostNeg(), field.friendlyGoalpostPos());

        // Our default behavior is to track the ball along the goal line, and always face
        // the ball
        Point goalie_pos = closestPointOnSeg(ball.position(), goalie_movement_segment);
        Angle goalie_orientation = (ball.position() - goalie_pos).orientation();
        std::unique_ptr<Intent> next_intent;

        auto [intersection1, intersection2] =
            raySegmentIntersection(ball_ray, full_goal_segment);
        if (intersection1.has_value() &&
            ball.velocity().len() > BALL_SLOW_SPEED_THRESHOLD)
        {
            // The ball is heading towards the net. Move to block the shot
            goalie_pos  = closestPointOnSeg(*intersection1, goalie_movement_segment);
            next_intent = move_action.updateStateAndGetNextIntent(
                *robot, goalie_pos, goalie_orientation, 0.0, false, AUTOCHIP);
        }
        else if (ball.velocity().len() < BALL_SLOW_SPEED_THRESHOLD &&
                 field.pointInFriendlyDefenseArea(ball.position()))
        {
            // If the ball is slow or stationary inside our defense area, chip it out
            next_intent = chip_action.updateStateAndGetNextIntent(
                *robot, ball, ball.position(),
                (ball.position() - field.friendlyGoal()).orientation(), 2);
        }
        else if (enemy_threat.has_value())
        {
            // If we have been provided with enemy threat information, use it to
            // preemptively block their best shot. Calculate the best shot the enemy could
            // take, ignoring the goalie as an obstacle
            auto best_shot = Evaluation::calcBestShotOnFriendlyGoal(
                field, friendly_team, enemy_team, ball.position(),
                ROBOT_MAX_RADIUS_METERS, {*robot});
            if (best_shot.has_value())
            {
                Vector shot_vector = (best_shot->first - enemy_threat->robot.position());
                Ray shot_ray       = Ray(enemy_threat->robot.position(), shot_vector);
                auto [best_shot_intersection_1, best_shot_intersection_2] =
                    raySegmentIntersection(shot_ray, full_goal_segment);

                if (best_shot_intersection_1.has_value())
                {
                    // Move to block the estimated best shot the enemy could take
                    goalie_pos = closestPointOnSeg(*best_shot_intersection_1,
                                                   goalie_movement_segment);
                }
            }

            next_intent = move_action.updateStateAndGetNextIntent(
                *robot, goalie_pos, goalie_orientation, 0.0, false, AUTOCHIP);
        }

        yield(std::move(next_intent));

    } while (!move_action.done());
}
