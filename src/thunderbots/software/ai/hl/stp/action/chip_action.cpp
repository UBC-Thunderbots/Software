#include "ai/hl/stp/action/chip_action.h"

#include "ai/intent/chip_intent.h"
#include "ai/intent/move_intent.h"
#include "geom/polygon.h"
#include "geom/util.h"
#include "shared/constants.h"

ChipAction::ChipAction() : Action() {}

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
    // How large the triangle is that defines the region where the robot is
    // behind the ball and ready to chip.
    // We want to keep the region small enough that we won't use the ChipIntent from too
    // far away (since the ChipIntent doesn't avoid obstacles and we risk colliding
    // with something), but large enough we can reasonably get in the region and chip the
    // ball successfully.
    // This value is 'X' in the ASCII art below
    double size_of_region_behind_ball = 6 * ROBOT_MAX_RADIUS_METERS;

    // ASCII art showing the region behind the ball
    // Diagram not to scale
    //
    //                             X
    //                      v-------------v
    //
    //                   >  B-------------C
    //                   |   \           /
    //                   |    \         /
    //                   |     \       /       <- Region considered "behind ball"
    //                 X |      \     /
    //                   |       \   /
    //                   |        \ /
    //                   >         A       <
    //                                     |
    //                                     | DIST_TO_FRONT_OF_ROBOT / 2
    //                                     |
    //                             O       |
    //             ball ->        O O      <
    //                             O
    //
    //                             |
    //                             V
    //                     direction of chip

    do
    {
        // A vector in the direction opposite the chip (behind the ball)
        Vector behind_ball =
            Vector::createFromAngle(this->chip_direction + Angle::half());


        // The points below make up the triangle that defines the region we treat as
        // "behind the ball". They correspond to the vertices labeled 'A', 'B', and 'C'
        // in the ASCII diagram

        // We make the region close enough to the ball so that the robot will still be
        // inside it when taking the chip.
        Point behind_ball_vertex_A =
            chip_origin + behind_ball.norm(DIST_TO_FRONT_OF_ROBOT_METERS * 0.5);
        Point behind_ball_vertex_B =
            behind_ball_vertex_A + behind_ball.norm(size_of_region_behind_ball) +
            behind_ball.perp().norm(size_of_region_behind_ball / 2);
        Point behind_ball_vertex_C =
            behind_ball_vertex_A + behind_ball.norm(size_of_region_behind_ball) -
            behind_ball.perp().norm(size_of_region_behind_ball / 2);

        Polygon behind_ball_region =
            Polygon({behind_ball_vertex_A, behind_ball_vertex_B, behind_ball_vertex_C});

        bool robot_behind_ball = behind_ball_region.containsPoint(robot->position());
        // The point in the middle of the region behind the ball
        Point point_behind_ball =
            chip_origin + behind_ball.norm(DIST_TO_FRONT_OF_ROBOT_METERS * 0.5 +
                                           size_of_region_behind_ball / 2);

        // If we're not in position to chip, move into position
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
