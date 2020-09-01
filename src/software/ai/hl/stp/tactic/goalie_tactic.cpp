#include "software/ai/hl/stp/tactic/goalie_tactic.h"

#include "shared/constants.h"
#include "software/ai/evaluation/calc_best_shot.h"
#include "software/ai/hl/stp/action/chip_action.h"
#include "software/ai/hl/stp/action/move_action.h"
#include "software/ai/hl/stp/action/stop_action.h"
#include "software/geom/algorithms/calculate_block_cone.h"
#include "software/geom/algorithms/closest_point.h"
#include "software/geom/algorithms/contains.h"
#include "software/geom/algorithms/intersection.h"
#include "software/geom/line.h"
#include "software/geom/point.h"
#include "software/geom/ray.h"
#include "software/geom/segment.h"
#include "software/parameter/dynamic_parameters.h"

GoalieTactic::GoalieTactic(const Ball &ball, const Field &field,
                           const Team &friendly_team, const Team &enemy_team)
    : Tactic(true, {RobotCapability::Move}),
      ball(ball),
      field(field),
      friendly_team(friendly_team),
      enemy_team(enemy_team)
{
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
    auto width_x_goal =
        intersection(Line(goalie_desired_position, field.friendlyGoalCenter()),
                     Line(goalie_restricted_area.posXPosYCorner(),
                          goalie_restricted_area.posXNegYCorner()));
    auto pos_side_x_goal =
        intersection(Line(goalie_desired_position, field.friendlyGoalCenter()),
                     Line(goalie_restricted_area.posXPosYCorner(),
                          goalie_restricted_area.negXPosYCorner()));
    auto neg_side_x_goal =
        intersection(Line(goalie_desired_position, field.friendlyGoalCenter()),
                     Line(goalie_restricted_area.posXNegYCorner(),
                          goalie_restricted_area.negXNegYCorner()));

    // if the goalie restricted area already contains the point, then we are
    // safe to move there.
    if (contains(goalie_restricted_area, goalie_desired_position))
    {
        return std::make_optional<Point>(goalie_desired_position);
    }
    // Due to the nature of the line intersection, its important to make sure the
    // corners are included, if the goalies desired position intersects with width (see
    // above), use those positions
    // The last comparison is for the edge case when the ball is behind the net
    else if (width_x_goal &&
             width_x_goal->y() <= goalie_restricted_area.posXPosYCorner().y() &&
             width_x_goal->y() >= goalie_restricted_area.posXNegYCorner().y() &&
             field.friendlyGoalCenter().x() <= goalie_desired_position.x())
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

void GoalieTactic::updateWorldParams(const Ball &ball, const Field &field,
                                     const Team &friendly_team, const Team &enemy_team)
{
    // Update the world parameters stored by this Tactic
    this->ball          = ball;
    this->field         = field;
    this->friendly_team = friendly_team;
    this->enemy_team    = enemy_team;
}

bool GoalieTactic::isGoalieTactic() const
{
    return true;
}

double GoalieTactic::calculateRobotCost(const Robot &robot, const World &world)
{
    // We don't prefer any particular robot to be the goalie, as there should only
    // ever be one robot that can act as the goalie
    return 0.5;
}

void GoalieTactic::calculateNextAction(ActionCoroutine::push_type &yield)
{
    auto move_action = std::make_shared<MoveAction>(true);
    auto chip_action = std::make_shared<ChipAction>();
    auto stop_action = std::make_shared<StopAction>(false);

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
        std::shared_ptr<Action> next_action;

        // compute intersection points from ball position and velocity
        Ray ball_ray = Ray(ball.position(), ball.velocity());

        // Create a segment along the goal line, slightly shortened to account for the
        // robot radius so as we move along the segment we don't try to run into the goal
        // posts. This will be used in case 3 as a fallback when we don't have an
        // intersection with the crease lines
        const Point neg_goal_line_inflated =
            field.friendlyGoalpostNeg() + Vector(0, -ROBOT_MAX_RADIUS_METERS);
        const Point pos_goal_line_inflated =
            field.friendlyGoalpostPos() + Vector(0, ROBOT_MAX_RADIUS_METERS);
        Segment full_goal_segment =
            Segment(neg_goal_line_inflated, pos_goal_line_inflated);

        std::vector<Point> intersections = intersection(ball_ray, full_goal_segment);

        // Load DynamicParameter
        // when should the goalie start panicking to move into place to stop the ball
        auto ball_speed_panic = DynamicParameters->getAIConfig()
                                    ->getGoalieTacticConfig()
                                    ->BallSpeedPanic()
                                    ->value();
        // what should the final goalie speed be, so that the goalie accelerates faster
        auto goalie_final_speed = DynamicParameters->getAIConfig()
                                      ->getGoalieTacticConfig()
                                      ->GoalieFinalSpeed()
                                      ->value();
        // how far in should the goalie wedge itself into the block cone, to block balls
        auto block_cone_radius = DynamicParameters->getAIConfig()
                                     ->getGoalieTacticConfig()
                                     ->BlockConeRadius()
                                     ->value();
        // by how much should the defense area be decreased so the goalie stays close
        // towards the net
        auto defense_area_deflation = DynamicParameters->getAIConfig()
                                          ->getGoalieTacticConfig()
                                          ->DefenseAreaDeflation()
                                          ->value();

        // if the ball is in the don't chip rectangle we do not chip the ball
        // as we risk bumping the ball into our own net trying to move behind
        // the ball
        auto dont_chip_rectangle = Rectangle(
            field.friendlyGoalpostNeg(),
            field.friendlyGoalpostPos() + Vector(2 * ROBOT_MAX_RADIUS_METERS, 0));

        // case 1: goalie should panic and stop the ball, its moving too fast towards the
        // net
        if (!intersections.empty() && ball.velocity().length() > ball_speed_panic)
        {
            // the ball is heading towards the net, move to intercept the shot
            // the final speed is a dynamic parameter so that if the goalie needs
            // to dive for the shot instead of stop when reaching the intersection
            // point it can do so.
            Point goalie_pos         = closestPoint((*robot).position(),
                                            Segment(ball.position(), intersections[0]));
            Angle goalie_orientation = (ball.position() - goalie_pos).orientation();

            move_action->updateControlParams(
                *robot, goalie_pos, goalie_orientation, 0.0, DribblerEnable::OFF,
                MoveType::NORMAL, AutochickType::AUTOCHIP, BallCollisionType::ALLOW);
            next_action = move_action;
        }
        // case 2: goalie does not need to panic and just needs to chip the ball out
        // of the net
        else if (ball.velocity().length() <= ball_speed_panic &&
                 field.pointInFriendlyDefenseArea(ball.position()))
        {
            // if the ball is slow but its not safe to chip it out, don't.
            // TODO finesse the ball out of the goal using the dribbler.
            // for now we just stop https://github.com/UBC-Thunderbots/Software/issues/744
            if (contains(dont_chip_rectangle, ball.position()) == true)
            {
                stop_action->updateControlParams(*robot, false);
                next_action = stop_action;
            }
            // if the ball is slow or stationary inside our defense area, and is safe
            // to do so, chip it out
            else
            {
                chip_action->updateControlParams(
                    *robot, ball.position(),
                    (ball.position() - field.friendlyGoalCenter()).orientation(), 2);
                next_action = chip_action;
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
            Point goalie_pos = calculateBlockCone(
                field.friendlyGoalpostNeg(), field.friendlyGoalpostPos(), ball.position(),
                block_cone_radius * block_cone_angle.toRadians());

            // we want to restrict the block cone to the friendly crease, also potentially
            // scaled by a defense_area_deflation_parameter
            Rectangle deflated_defense_area = field.friendlyDefenseArea();
            deflated_defense_area.inflate(-defense_area_deflation);

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
                        field.friendlyGoalpostPos() + Vector(0, -ROBOT_MAX_RADIUS_METERS);
                }
                else
                {
                    goalie_pos =
                        field.friendlyGoalpostNeg() + Vector(0, ROBOT_MAX_RADIUS_METERS);
                }
            }
            else
            {
                goalie_pos = *clamped_goalie_pos;
            }
            Angle goalie_orientation = (ball.position() - goalie_pos).orientation();

            move_action->updateControlParams(*robot, goalie_pos, goalie_orientation,
                                             goalie_final_speed, DribblerEnable::OFF,
                                             MoveType::NORMAL, AutochickType::AUTOCHIP,
                                             BallCollisionType::ALLOW);
            next_action = move_action;
        }

        yield(next_action);
    } while (!move_action->done());
}

void GoalieTactic::accept(MutableTacticVisitor &visitor)
{
    visitor.visit(*this);
}

Ball GoalieTactic::getBall() const
{
    return this->ball;
}

Field GoalieTactic::getField() const
{
    return this->field;
}

Team GoalieTactic::getFriendlyTeam() const
{
    return this->friendly_team;
}

Team GoalieTactic::getEnemyTeam() const
{
    return this->enemy_team;
}
