#include "ai/hl/stp/action/kick_action.h"

#include "ai/intent/kick_intent.h"
#include "ai/intent/move_intent.h"
#include "geom/polygon.h"
#include "geom/util.h"

KickAction::KickAction(double length_of_region_behind_ball)
    : Action(), length_of_region_behind_ball(length_of_region_behind_ball)
{
}

std::unique_ptr<Intent> KickAction::updateStateAndGetNextIntent(
    const Robot& robot, Point kick_origin, Point kick_target,
    double kick_speed_meters_per_second)
{
    return updateStateAndGetNextIntent(robot, kick_origin,
                                       (kick_target - kick_origin).orientation(),
                                       kick_speed_meters_per_second);
}

std::unique_ptr<Intent> KickAction::updateStateAndGetNextIntent(
    const Robot& robot, Point kick_origin, Angle kick_direction,
    double kick_speed_meters_per_second)
{
    // Update the parameters stored by this Action
    this->robot                        = robot;
    this->kick_origin                  = kick_origin;
    this->kick_direction               = kick_direction;
    this->kick_speed_meters_per_second = kick_speed_meters_per_second;

    return getNextIntent();
}

std::unique_ptr<Intent> KickAction::calculateNextIntent(
    intent_coroutine::push_type& yield)
{
    // TODO: replace with real constant
    double DIST_TO_FRONT_OF_ROBOT = 0.05;
    // ASCII art showing the region behind the ball
    //
    //              X
    //       v-------------v
    //
    //    >  \-------------/
    //    |   \           /
    //    |    \         /
    //    |     \       /       <- Region considered "behind ball"
    // 2X |      \     /
    //    |       \   /
    //    |        \ /
    //    >         v
    //
    //
    //
    //              O
    //             O O          <- Ball
    //              O
    //
    //              |
    //              V
    //       direction of kick

    do
    {
        // A vector in the direction opposite the kick (behind the ball)
        Vector behind_ball =
            Vector::createFromAngle(this->kick_direction + Angle::half());
        // We make the closest point of the region close enough to the ball so that the
        // robot will still be inside it when it's taking the kick.
        // This point is the "top" of the Isosceles triangle
        Point behind_ball_close =
            kick_origin + behind_ball.norm(DIST_TO_FRONT_OF_ROBOT * 0.5);
        // These points form the "base" of the Isosceles triangle
        Point behind_ball_far_1 =
            kick_origin +
            behind_ball.norm(DIST_TO_FRONT_OF_ROBOT * 0.5 +
                             length_of_region_behind_ball) +
            behind_ball.perp().norm(length_of_region_behind_ball / 2);
        Point behind_ball_far_2 =
            kick_origin +
            behind_ball.norm(DIST_TO_FRONT_OF_ROBOT * 0.5 +
                             length_of_region_behind_ball) -
            behind_ball.perp().norm(length_of_region_behind_ball / 2);
        // Forms an Isosceles triangle that represents the region behind the ball. This
        // region is close enough to the ball that the robot will still be inside it
        // even if the ball is in the robot's dribbler, since we don't want the robot
        // to think it's no longer behind the ball right when it's going to take the kick
        // The height of this triangle is equal to the specified length of the region
        // behind the ball. The base of the triangle is also equal to this length.
        Polygon behind_ball_region =
            Polygon({behind_ball_close, behind_ball_far_1, behind_ball_far_2});

        bool robot_behind_ball = behind_ball_region.containsPoint(robot->position());
        // The point in the middle of the region behind the ball
        Point point_behind_ball =
            kick_origin + behind_ball.norm(DIST_TO_FRONT_OF_ROBOT * 0.5 +
                                           length_of_region_behind_ball / 2);

        if (!robot_behind_ball)
        {
            yield(std::make_unique<MoveIntent>(robot->id(), point_behind_ball,
                                               kick_direction, 0.0, 0));
        }
        else
        {
            yield(std::make_unique<KickIntent>(robot->id(), kick_origin, kick_direction,
                                               kick_speed_meters_per_second, 0));
        }
    } while (true);
}
