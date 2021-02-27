#include "software/ai/hl/stp/action/chip_action.h"

#include "shared/constants.h"
#include "software/ai/intent/chip_intent.h"
#include "software/ai/intent/move_intent.h"
#include "software/geom/algorithms/contains.h"
#include "software/geom/polygon.h"
#include "software/geom/triangle.h"
#include "software/world/ball.h"

ChipAction::ChipAction() : Action(false), ball({0, 0}, {0, 0}, Timestamp::fromSeconds(0))
{
}

void ChipAction::updateWorldParams(const World& world)
{
    this->ball = world.ball();
}

void ChipAction::updateControlParams(const Robot& robot, Point chip_origin,
                                     Angle chip_direction, double chip_distance_meters)
{
    this->robot                = robot;
    this->chip_origin          = chip_origin;
    this->chip_direction       = chip_direction;
    this->chip_distance_meters = chip_distance_meters;
}

void ChipAction::updateControlParams(const Robot& robot, Point chip_origin,
                                     Point chip_target)
{
    updateControlParams(robot, chip_origin, (chip_target - chip_origin).orientation(),
                        (chip_target - chip_origin).length());
}

Ball ChipAction::getBall()
{
    return ball;
}

Point ChipAction::getChipOrigin()
{
    return chip_origin;
}

Angle ChipAction::getChipDirection()
{
    return chip_direction;
}

double ChipAction::getChipDistanceMeters()
{
    return chip_distance_meters;
}

void ChipAction::calculateNextIntent(IntentCoroutine::push_type& yield)
{
    // How large the triangle is that defines the region where the robot is
    // behind the ball and ready to chip.
    // We want to keep the region small enough that we won't use the ChipIntent from too
    // far away (since the ChipIntent doesn't avoid obstacles and we risk colliding
    // with something), but large enough we can reasonably get in the region and chip the
    // ball successfully.
    // This value is 'X' in the ASCII art below
    double size_of_region_behind_ball = 4 * ROBOT_MAX_RADIUS_METERS;

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
    //    The ball is    >         A
    //    at A
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
        Point behind_ball_vertex_A = chip_origin;
        Point behind_ball_vertex_B =
            behind_ball_vertex_A + behind_ball.normalize(size_of_region_behind_ball) +
            behind_ball.perpendicular().normalize(size_of_region_behind_ball / 2);
        Point behind_ball_vertex_C =
            behind_ball_vertex_A + behind_ball.normalize(size_of_region_behind_ball) -
            behind_ball.perpendicular().normalize(size_of_region_behind_ball / 2);

        Triangle behind_ball_region =
            Triangle(behind_ball_vertex_A, behind_ball_vertex_B, behind_ball_vertex_C);

        bool robot_behind_ball = contains(behind_ball_region, robot->position());
        // The point in the middle of the region behind the ball
        Point point_behind_ball =
            chip_origin + behind_ball.normalize(size_of_region_behind_ball * 3 / 4);

        // If we're not in position to chip, move into position
        if (!robot_behind_ball)
        {
            yield(std::make_unique<MoveIntent>(robot->id(), point_behind_ball,
                                               chip_direction, 0.0, DribblerMode::OFF,
                                               BallCollisionType::ALLOW, std::nullopt,
                                               ROBOT_MAX_SPEED_METERS_PER_SECOND));
        }
        else
        {
            yield(std::make_unique<ChipIntent>(robot->id(), chip_origin, chip_direction,
                                               chip_distance_meters));
        }
    } while (!ball.hasBallBeenKicked(chip_direction));
}
