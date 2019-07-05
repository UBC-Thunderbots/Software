#include "ai/hl/stp/tactic/goalie_tactic.h"

#include "ai/hl/stp/action/chip_action.h"
#include "ai/hl/stp/action/move_action.h"
#include "ai/hl/stp/action/stop_action.h"
#include "ai/hl/stp/evaluation/calc_best_shot.h"
#include "geom/point.h"
#include "geom/ray.h"
#include "geom/segment.h"
#include "geom/util.h"
#include "shared/constants.h"
#include "util/parameter/dynamic_parameters.h"


GoalieTactic::GoalieTactic(const Ball &ball, const Field &field,
        const Team &friendly_team, const Team &enemy_team)
    : ball(ball),
    field(field),
    friendly_team(friendly_team),
    enemy_team(enemy_team),
    Tactic(true)
{
    addWhitelistedAvoidArea(AvoidArea::FRIENDLY_DEFENSE_AREA);
    addWhitelistedAvoidArea(AvoidArea::BALL);
    addWhitelistedAvoidArea(AvoidArea::HALF_METER_AROUND_BALL);
}

std::string GoalieTactic::getName() const
{
    return "Goalie Tactic";
}

void GoalieTactic::updateParams(const Ball &ball, const Field &field,
        const Team &friendly_team, const Team &enemy_team)
{
    // Update the parameters stored by this Tactic
    this->ball          = ball;
    this->field         = field;
    this->friendly_team = friendly_team;
    this->enemy_team    = enemy_team;
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
    StopAction stop_action = StopAction();

    do
    {
        // Goalie Tactic
        //
        // The goalie tactic is responsible for blocking as many shots as it can, to the
        // best of its ability. The tactic consists of 3 cases
        //
        // Case 1: The ball is moving towards the goal and has a speed that is concerning
        //      Goalie moves onto the closest point on the oncoming line to stop the ball
        //
        // Case 2: The ball is moving at a slow speed and is inside the defense area
        //      Goalie moves to the ball and chips it out of the defense area
        //
        // Case 3: Any other case
        //      Goalie blocks the cone to the net. (Cone being from the ball to either
        //      goal post) The goalie also snaps to a semicircle inside the defense area,
        //      to avoid leaving the defense area
        //
        std::unique_ptr<Intent> next_intent;

        // rectangle we do not chip the ball if its in as it would be unsafe to do so
        // as we risk bumping the ball into our own net
        auto dont_chip_rectangle = Rectangle(
                field.friendlyGoalpostNeg(),
                field.friendlyGoalpostPos() + Point(2 * ROBOT_MAX_RADIUS_METERS, 0));

        // compute intersection points from ball position and velocity
        Ray ball_ray = Ray(ball.position(), ball.velocity());
        Segment full_goal_segment =
            Segment(field.friendlyGoalpostNeg(), field.friendlyGoalpostPos());

        auto [intersection1, intersection2] =
            raySegmentIntersection(ball_ray, full_goal_segment);

        // when should the goalie start panicking to move into place to stop the ball
        auto ball_speed_panic =
            Util::DynamicParameters::GoalieTactic::ball_speed_panic.value();

        // case 1: goalie should panic and stop the ball, its moving too fast towards the net
        if (intersection1.has_value() && ball.velocity().len() > ball_speed_panic)
        {
            // the ball is heading towards the net, move to intercept the shot
            // the final speed is a dynamic parameter so that if the goalie needs
            // to dive for the shot instead of stop when reaching the intersection
            // point it can do so.
            Point goalie_pos = closestPointOnSeg(
                    (*robot).position(), Segment(ball.position(), *intersection1));
            Angle goalie_orientation = (ball.position() - goalie_pos).orientation();

            next_intent = move_action.updateStateAndGetNextIntent(
                    *robot, goalie_pos, goalie_orientation,
                    Util::DynamicParameters::GoalieTactic::goalie_final_speed.value(), false,
                    AUTOCHIP);
        }
        // case 2: goalie does not need to panic and just needs to chip the ball out
        // of the net
        else if (ball.velocity().len() <= ball_speed_panic &&
                field.pointInFriendlyDefenseArea(ball.position()))
        {
            // if the ball is slow but its not safe to chip it out, dont.
            //
            // TODO finesse the ball out of the goal using the dribbler.
            // for now we just stop
            if (dont_chip_rectangle.containsPoint(ball.position()) == true)
            {
                next_intent = stop_action.updateStateAndGetNextIntent(*robot, false);
            }
            // if the ball is slow or stationary inside our defense area, and is safe
            // to do so, chip it out
            else
            {
                next_intent = chip_action.updateStateAndGetNextIntent(
                        *robot, ball, ball.position(),
                        (ball.position() - field.friendlyGoal()).orientation(), 2);
            }
        }
        // case 3: ball does not have a clear velocity vector towards the goal, so
        // position goalie in best position to block shot
        else
        {
            // compute angle between two vectors, negative goal post to ball and positive
            // goal post to ball
            Angle block_cone_angle =
                (ball.position() - field.friendlyGoalpostNeg()).orientation()
                .minDiff((ball.position() - field.friendlyGoalpostPos()).orientation());

            // compute block cone position, allowing 1 ROBOT_MAX_RADIUS_METERS extra on either side
            Point goalie_pos =
                calcBlockCone(field.friendlyGoalpostNeg(), field.friendlyGoalpostPos(),
                        ball.position(), 4*ROBOT_MAX_RADIUS_METERS*block_cone_angle.toRadians());

            //// we want to restrict the block cone to the friendly crease
            Rectangle deflated_defense_area = field.friendlyDefenseArea();
            deflated_defense_area.expand(-5 * ROBOT_MAX_RADIUS_METERS);

            Angle goalie_orientation = (ball.position() - goalie_pos).orientation();
                next_intent             = move_action.updateStateAndGetNextIntent(
                        *robot, clipPoint(goalie_pos, deflated_defense_area), goalie_orientation, Util::DynamicParameters::GoalieTactic::goalie_final_speed.value(), false, AUTOCHIP);
        }


        yield(std::move(next_intent));
    } while (!move_action.done());
}
