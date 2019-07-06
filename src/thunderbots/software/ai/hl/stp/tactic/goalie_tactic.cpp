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
#include "util/logger/init.h"
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
        return std::numeric_limits<int>::max() - 10;
    }
}

std::optional<Point> GoalieTactic::restrainGoalieInRectangle(Point goalie_desired_position, Rectangle goalie_restricted_area){

    Point clamped_goalie_pos;

    // get the intersections to all 3 crease lines in question
    auto width_x_goal = lineIntersection(goalie_desired_position, field.friendlyGoal(),
            goalie_restricted_area.neCorner(),
            goalie_restricted_area.seCorner());
    auto pos_side_x_goal = lineIntersection(goalie_desired_position, field.friendlyGoal(),
            goalie_restricted_area.neCorner(),
            goalie_restricted_area.nwCorner());
    auto neg_side_x_goal = lineIntersection(goalie_desired_position, field.friendlyGoal(),
            goalie_restricted_area.seCorner(),
            goalie_restricted_area.swCorner());

    // if the goalie restricted area already contains the point, then we are
    // safe to move there.
    if (goalie_restricted_area.containsPoint(goalie_desired_position)){
        return std::make_optional<Point>(goalie_desired_position);
    }
    // if the goalie position to the block cone position intersects the 
    // longest crease line which runs across the y axis, then intersect
    else if (width_x_goal && width_x_goal->y() <= goalie_restricted_area.neCorner().y() &&
            width_x_goal->y() >= goalie_restricted_area.seCorner().y())
    {
        return std::make_optional<Point>(*width_x_goal);
    }

    // if either two sides of the goal are intercepted, then use those positions
    else if (pos_side_x_goal &&
            pos_side_x_goal->x() <= goalie_restricted_area.neCorner().x() &&
            pos_side_x_goal->x() >= goalie_restricted_area.nwCorner().x())
    {
        return std::make_optional<Point>(*pos_side_x_goal);
    }
    else if (neg_side_x_goal &&
            neg_side_x_goal->x() <= goalie_restricted_area.seCorner().x() &&
            neg_side_x_goal->x() >= goalie_restricted_area.swCorner().x())
    {
        return std::make_optional<Point>(*neg_side_x_goal);
    }

    // if there are no intersections, then we are out of luck
    else {
        return std::nullopt;
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
        // 		NOTE: If the ball is in the dont_chip_rectangle, then we prefer timeout, over
        //			scoring on ourselfs
        //
        // Case 3: Any other case
        //      Goalie blocks the cone to the net. (Cone being from the ball to either
        //      goal post) The goalie also snaps to a rectangle inside the defense area,
        //      to avoid leaving the defense area
        //
        std::unique_ptr<Intent> next_intent;

        // Create a segment along the goal line, slightly shortened to account for the
        // robot radius so as we move along the segment we don't try run into the goal
        // posts. This will be used in case3 as a fallback when we don't have an intersection
        // with the crease lines
        Point p1                        = Point(field.friendlyGoalpostNeg().x(),
                field.friendlyGoalpostNeg().y() + ROBOT_MAX_RADIUS_METERS);
        Point p2                        = Point(field.friendlyGoalpostPos().x(),
                field.friendlyGoalpostPos().y() - ROBOT_MAX_RADIUS_METERS);
        Segment goalie_movement_segment = Segment(p1, p2);

        // compute intersection points from ball position and velocity
        Ray ball_ray = Ray(ball.position(), ball.velocity());
        Segment full_goal_segment =
            Segment(field.friendlyGoalpostNeg(), field.friendlyGoalpostPos());

        auto [intersection1, intersection2] =
            raySegmentIntersection(ball_ray, full_goal_segment);

        // Load DynamicParameter
        // when should the goalie start panicking to move into place to stop the ball
        auto ball_speed_panic =
            Util::DynamicParameters::GoalieTactic::ball_speed_panic.value();
        // what should the final goalie speed be, so that the goalie accelarates faster
        auto goalie_final_speed =
            Util::DynamicParameters::GoalieTactic::goalie_final_speed.value();
        // how far in should the goalie wedge itself into the block cone, to block balls
        auto block_cone_radius =
            Util::DynamicParameters::GoalieTactic::block_cone_radius.value();
        // by how much should the defense are be decreased so the goalie stays close
        // towards the net
        auto defense_area_deflation =
            Util::DynamicParameters::GoalieTactic::defense_area_deflation.value();

        // rectangle we do not chip the ball if its in as it would be unsafe to do so
        // as we risk bumping the ball into our own net
        auto dont_chip_rectangle = Rectangle(
                field.friendlyGoalpostNeg(),
                field.friendlyGoalpostPos() + Point(2 * ROBOT_MAX_RADIUS_METERS, 0));

        // case 1: goalie should panic and stop the ball, its moving too fast towards the
        // net
        if (intersection1.has_value() && ball.velocity().len() > ball_speed_panic)
        {
            // the ball is heading towards the net, move to intercept the shot
            // the final speed is a dynamic parameter so that if the goalie needs
            // to dive for the shot instead of stop when reaching the intersection
            // point it can do so.
            Point goalie_pos = closestPointOnSeg(
                    (*robot).position(), Segment(ball.position(), *intersection1));
            Angle goalie_orientation = (ball.position() - goalie_pos).orientation();
            next_intent              = move_action.updateStateAndGetNextIntent(
                    *robot, goalie_pos, goalie_orientation, goalie_final_speed, AUTOCHIP);
        }
        // case 2: goalie does not need to panic and just needs to chip the ball out
        // of the net
        else if (ball.velocity().len() <= ball_speed_panic &&
                field.pointInFriendlyDefenseArea(ball.position()))
        {
            // if the ball is slow but its not safe to chip it out, dont.
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
                (ball.position() - field.friendlyGoalpostNeg())
                .orientation()
                .minDiff(
                        (ball.position() - field.friendlyGoalpostPos()).orientation());

            // compute block cone position, allowing 1 ROBOT_MAX_RADIUS_METERS extra on
            // either side
            Point goalie_pos = calcBlockCone(
                    field.friendlyGoalpostNeg(), field.friendlyGoalpostPos(), ball.position(),
                    block_cone_radius * block_cone_angle.toRadians());

            // we want to restrict the block cone to the friendly crease, also potentially
            // scaled by a a defense_area_deflation_parameter
            Rectangle deflated_defense_area = field.friendlyDefenseArea();
            deflated_defense_area.expand(-defense_area_deflation);

            // restrain the goalie in the deflated defense area, if the goalie cannot be restrained
            // or if there is no proper intersection, then we safely default to center of the goal
            auto clamped_goalie_pos = restrainGoalieInRectangle(goalie_pos, deflated_defense_area);

            if (!clamped_goalie_pos){
                if (ball.position().y() > 0){
                    goalie_pos = field.friendlyGoalpostPos() + Point(-ROBOT_MAX_RADIUS_METERS, 0);
                } else {
                    goalie_pos = field.friendlyGoalpostNeg() + Point(ROBOT_MAX_RADIUS_METERS, 0);
                }
            } else {
                goalie_pos = *clamped_goalie_pos;
            }
            Angle goalie_orientation = (ball.position() - goalie_pos).orientation();

            next_intent = move_action.updateStateAndGetNextIntent(
                    *robot, goalie_pos, goalie_orientation, goalie_final_speed,
                    AUTOCHIP);
        }
        yield(std::move(next_intent));
    } while (!move_action.done());
}

