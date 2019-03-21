#include "ai/hl/stp/action/chip_action.h"

#include "ai/intent/chip_intent.h"
#include "ai/intent/move_intent.h"
#include "geom/polygon.h"
#include "geom/util.h"

ChipAction::ChipAction(double length_of_region_behind_ball)
    : Action(), length_of_region_behind_ball(length_of_region_behind_ball)
{
}

std::unique_ptr<Intent> ChipAction::updateStateAndGetNextIntent(
    const Robot& robot, Point chip_origin, Point chip_target, double chip_distance_meters)
{
    return updateStateAndGetNextIntent(robot, chip_origin,
                                       (chip_target - chip_origin).orientation(),
                                       chip_distance_meters);
}

std::unique_ptr<Intent> ChipAction::updateStateAndGetNextIntent(
    const Robot& robot, Point chip_origin, Angle chip_direction,
    double chip_distance_meters)
{
    // Update the parameters stored by this Action
    this->robot                = robot;
    this->chip_origin          = chip_origin;
    this->chip_direction       = chip_direction;
    this->chip_distance_meters = chip_distance_meters;

    return getNextIntent();
}

std::unique_ptr<Intent> ChipAction::calculateNextIntent(
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
    //       direction of chip

    do
    {
        // A vector in the direction opposite the chip (behind the ball)
        Vector behind_ball =
            Vector::createFromAngle(this->chip_direction + Angle::half());
        // We make the closest point of the region close enough to the ball so that the
        // robot will still be inside it when it's taking the chip.
        // This point is the "top" of the Isosceles triangle
        Point behind_ball_close =
            chip_origin + behind_ball.norm(DIST_TO_FRONT_OF_ROBOT * 0.5);
        // These points form the "base" of the Isosceles triangle
        Point behind_ball_far_1 =
            chip_origin +
            behind_ball.norm(DIST_TO_FRONT_OF_ROBOT * 0.5 +
                             length_of_region_behind_ball) +
            behind_ball.perp().norm(length_of_region_behind_ball / 2);
        Point behind_ball_far_2 =
            chip_origin +
            behind_ball.norm(DIST_TO_FRONT_OF_ROBOT * 0.5 +
                             length_of_region_behind_ball) -
            behind_ball.perp().norm(length_of_region_behind_ball / 2);
        // Forms an Isosceles triangle that represents the region behind the ball. This
        // region is close enough to the ball that the robot will still be inside it
        // even if the ball is in the robot's dribbler, since we don't want the robot
        // to think it's no longer behind the ball right when it's going to take the chip
        // The height of this triangle is equal to the specified length of the region
        // behind the ball. The base of the triangle is also equal to this length.
        Polygon behind_ball_region =
            Polygon({behind_ball_close, behind_ball_far_1, behind_ball_far_2});

        bool robot_behind_ball = behind_ball_region.containsPoint(robot->position());
        // The point in the middle of the region behind the ball
        Point point_behind_ball =
            chip_origin + behind_ball.norm(DIST_TO_FRONT_OF_ROBOT * 0.5 +
                                           length_of_region_behind_ball / 2);

        if (!robot_behind_ball)
        {
            yield(std::make_unique<MoveIntent>(robot->id(), point_behind_ball,
                                               chip_direction, 0.0, 0));
        }
        else
        {
            yield(std::make_unique<ChipIntent>(robot->id(), chip_origin, chip_direction,
                                               chip_distance_meters, 0));
        }
    } while (true);
}
