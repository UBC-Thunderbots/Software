#include "software/ai/hl/stp/tactic/goalie_tactic.h"

#include "shared/constants.h"
#include "software/ai/evaluation/calc_best_shot.h"
#include "software/ai/hl/stp/action/chip_action.h"
#include "software/ai/hl/stp/action/move_action.h"
#include "software/ai/hl/stp/action/stop_action.h"
#include "software/ai/hl/stp/tactic/tactic_visitor.h"
#include "software/geom/point.h"
#include "software/geom/ray.h"
#include "software/geom/segment.h"
#include "software/geom/util.h"
#include "software/util/parameter/dynamic_parameters.h"


GoalieTactic::GoalieTactic(const Ball &ball, const Field &field,
                           const Team &friendly_team, const Team &enemy_team)
    : Tactic(true),
      ball(ball),
      field(field),
      friendly_team(friendly_team),
      enemy_team(enemy_team)
{
    addWhitelistedAvoidArea(AvoidArea::FRIENDLY_DEFENSE_AREA);
    addWhitelistedAvoidArea(AvoidArea::HALF_METER_AROUND_BALL);
}

std::string GoalieTactic::getName() const
{
    return "Goalie Tactic";
}

void GoalieTactic::updateWorldParams(const Ball &ball, const Field &field,
                                     const Team &friendly_team, const Team &enemy_team)
{
    // Update the world parameters stored by this Tactic
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
        // TODO perform proper goalie assignment using plays
        // https://github.com/UBC-Thunderbots/Software/issues/745
        return std::numeric_limits<int>::max() - 10;
    }
}

std::optional<Point> GoalieTactic::restrainGoalieInRectangle(
    Point goalie_desired_position, Rectangle goalie_restricted_area)
{
    //           NW    pos_side   NE
    //            +---------------+
    //            |               |
    //            |               |
    //            |               |
    //       +----+               |
    //       |    |               |
    //       |    |               |
    // goal  |    |               | width
    //       |    |               |
    //       |    |               |
    //       |    |               |
    //       +----+               |
    //            |               |
    //            |               |
    //            |               |
    //           ++---------------+
    //           SW    neg_side   SE
    //
    // Given the goalies desired position and the restricted area,
    // first find the 3 intersections with each side of the restricted area
    // (width, pos_side, neg_side) and the line from the desired position to the
    // center of the friendly goal
    auto width_x_goal    = lineIntersection(goalie_desired_position, field.friendlyGoal(),
                                         goalie_restricted_area.posXPosYCorner(),
                                         goalie_restricted_area.posXNegYCorner());
    auto pos_side_x_goal = lineIntersection(goalie_desired_position, field.friendlyGoal(),
                                            goalie_restricted_area.posXPosYCorner(),
                                            goalie_restricted_area.negXPosYCorner());
    auto neg_side_x_goal = lineIntersection(goalie_desired_position, field.friendlyGoal(),
                                            goalie_restricted_area.posXNegYCorner(),
                                            goalie_restricted_area.negXNegYCorner());

    // if the goalie restricted area already contains the point, then we are
    // safe to move there.
    if (goalie_restricted_area.containsPoint(goalie_desired_position))
    {
        return std::make_optional<Point>(goalie_desired_position);
    }
    // Due to the nature of the line intersection, its important to make sure the
    // corners are included, if the goalies desired position intersects with width (see
    // above), use those positions
    else if (width_x_goal &&
             width_x_goal->y() <= goalie_restricted_area.posXPosYCorner().y() &&
             width_x_goal->y() >= goalie_restricted_area.posXNegYCorner().y())
    {
        return std::make_optional<Point>(*width_x_goal);
    }

    // if either two sides of the goal are intercepted, then use those positions
    else if (pos_side_x_goal &&
             pos_side_x_goal->x() <= goalie_restricted_area.posXPosYCorner().x() &&
             pos_side_x_goal->x() >= goalie_restricted_area.negXPosYCorner().x())
    {
        return std::make_optional<Point>(*pos_side_x_goal);
    }
    else if (neg_side_x_goal &&
             neg_side_x_goal->x() <= goalie_restricted_area.posXNegYCorner().x() &&
             neg_side_x_goal->x() >= goalie_restricted_area.negXNegYCorner().x())
    {
        return std::make_optional<Point>(*neg_side_x_goal);
    }

    // if there are no intersections (ex. ball behind net), then we are out of luck
    else
    {
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
        // NOTE: If the ball is in the dont_chip_rectangle, then we prefer timeout,
        // over scoring on ourselves
        //
        // Case 3: Any other case
        //      Goalie blocks the cone to the net. (Cone being from the ball to either
        //      goal post) The goalie also snaps to a rectangle inside the defense area,
        //      to avoid leaving the defense area
        //
        std::unique_ptr<Intent> next_intent;

        // Create a segment along the goal line, slightly shortened to account for the
        // robot radius so as we move along the segment we don't try run into the goal
        // posts. This will be used in case3 as a fallback when we don't have an
        // intersection with the crease lines

        // compute intersection points from ball position and velocity
        Ray ball_ray = Ray(ball.position(), ball.velocity());

        const Point neg_goal_line_inflated =
            field.friendlyGoalpostNeg() + Point(0, -ROBOT_MAX_RADIUS_METERS);
        const Point pos_goal_line_inflated =
            field.friendlyGoalpostPos() + Point(0, ROBOT_MAX_RADIUS_METERS);
        Segment full_goal_segment =
            Segment(neg_goal_line_inflated, pos_goal_line_inflated);

        std::optional<Point> intersection1 =
            raySegmentIntersection(ball_ray, full_goal_segment).first;

        // Load DynamicParameter
        // when should the goalie start panicking to move into place to stop the ball
        auto ball_speed_panic =
            Util::DynamicParameters->getGoalieTacticConfig()->BallSpeedPanic()->value();
        // what should the final goalie speed be, so that the goalie accelerates faster
        auto goalie_final_speed =
            Util::DynamicParameters->getGoalieTacticConfig()->GoalieFinalSpeed()->value();
        // how far in should the goalie wedge itself into the block cone, to block balls
        auto block_cone_radius =
            Util::DynamicParameters->getGoalieTacticConfig()->BlockConeRadius()->value();
        // by how much should the defense are be decreased so the goalie stays close
        // towards the net
        auto defense_area_deflation = Util::DynamicParameters->getGoalieTacticConfig()
                                          ->DefenseAreaDeflation()
                                          ->value();

        // if the ball is in the don't chip rectangle we do not chip the ball
        // as we risk bumping the ball into our own net trying to move behind
        // the ball
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

            next_intent = move_action.updateStateAndGetNextIntent(
                *robot, goalie_pos, goalie_orientation, 0.0, DribblerEnable::OFF,
                MoveType::NORMAL, AutokickType::AUTOCHIP);
        }
        // case 2: goalie does not need to panic and just needs to chip the ball out
        // of the net
        else if (ball.velocity().len() <= ball_speed_panic &&
                 field.pointInFriendlyDefenseArea(ball.position()))
        {
            // if the ball is slow but its not safe to chip it out, don't.
            // TODO finesse the ball out of the goal using the dribbler.
            // for now we just stop https://github.com/UBC-Thunderbots/Software/issues/744
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
            // block the cone by default
            float radius = Util::DynamicParameters->getGoalieTacticConfig()
                               ->BlockConeBuffer()
                               ->value() +
                           ROBOT_MAX_RADIUS_METERS;

            Point goalie_pos =
                calcBlockCone(field.friendlyGoalpostNeg(), field.friendlyGoalpostPos(),
                              ball.position(), radius);

            // restrict the goalie to a semicircle inscribed inside the defense area
            Point goalie_restricted_pos =
                field.friendlyGoal() - (field.friendlyDefenseArea().yLength() *
                                        (field.friendlyGoal() - goalie_pos).norm());

            // restrict the point to be within the defense area
            auto goalie_orientation = (ball.position() - goalie_pos).orientation();
            next_intent             = move_action.updateStateAndGetNextIntent(
                *robot, goalie_restricted_pos, goalie_orientation, 0.0,
                DribblerEnable::OFF, MoveType::NORMAL, AUTOCHIP);
        }

        // compute angle between two vectors, negative goal post to ball and positive
        // goal post to ball
        Angle block_cone_angle =
            (ball.position() - field.friendlyGoalpostNeg())
                .orientation()
                .minDiff((ball.position() - field.friendlyGoalpostPos()).orientation());

        // compute block cone position, allowing 1 ROBOT_MAX_RADIUS_METERS extra on
        // either side
        Point goalie_pos = calcBlockCone(
            field.friendlyGoalpostNeg(), field.friendlyGoalpostPos(), ball.position(),
            block_cone_radius * block_cone_angle.toRadians());

        // we want to restrict the block cone to the friendly crease, also potentially
        // scaled by a defense_area_deflation_parameter
        Rectangle deflated_defense_area = field.friendlyDefenseArea();
        deflated_defense_area.expand(-defense_area_deflation);

        // restrain the goalie in the deflated defense area, if the goalie cannot be
        // restrained or if there is no proper intersection, then we safely default to
        // center of the goal
        auto clamped_goalie_pos =
            restrainGoalieInRectangle(goalie_pos, deflated_defense_area);

        // if the goalie could not be restrained in the deflated defense area,
        // then the ball must be either on a really sharp angle to the net where
        // its impossible to get a shot, or the ball is behind the net, in which
        // case we snap to either post
        if (!clamped_goalie_pos)
        {
            if (ball.position().y() > 0)
            {
                goalie_pos =
                    field.friendlyGoalpostPos() + Point(-ROBOT_MAX_RADIUS_METERS, 0);
            }
            else
            {
                goalie_pos =
                    field.friendlyGoalpostNeg() + Point(ROBOT_MAX_RADIUS_METERS, 0);
            }
        }
        else
        {
            goalie_pos = *clamped_goalie_pos;
        }
        Angle goalie_orientation = (ball.position() - goalie_pos).orientation();

        next_intent = move_action.updateStateAndGetNextIntent(
            *robot, goalie_pos, goalie_orientation, goalie_final_speed,
            DribblerEnable::OFF, MoveType::NORMAL, AUTOCHIP);

        yield(std::move(next_intent));
    } while (!move_action.done());
}

void GoalieTactic::accept(TacticVisitor &visitor) const
{
    visitor.visit(*this);
}
