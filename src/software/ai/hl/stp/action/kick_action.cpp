#include "software/ai/hl/stp/action/kick_action.h"

#include "shared/constants.h"
#include "software/ai/intent/kick_intent.h"
#include "software/ai/intent/move_intent.h"
#include "software/geom/algorithms/contains.h"
#include "software/geom/polygon.h"
#include "software/world/ball.h"

KickAction::KickAction() : Action(false), ball({0, 0}, {0, 0}, Timestamp::fromSeconds(0))
{
}

void KickAction::updateWorldParams(const World &world)
{
    this->ball = world.ball();
}

void KickAction::updateControlParams(const Robot &robot, Point kick_origin,
                                     Point kick_target,
                                     double kick_speed_meters_per_second)
{
    updateControlParams(robot, kick_origin, (kick_target - kick_origin).orientation(),
                        kick_speed_meters_per_second);
}

void KickAction::updateControlParams(const Robot &robot, Point kick_origin,
                                     Angle kick_direction,
                                     double kick_speed_meters_per_second)
{
    this->robot                        = robot;
    this->kick_origin                  = kick_origin;
    this->kick_direction               = kick_direction;
    this->kick_speed_meters_per_second = kick_speed_meters_per_second;
}

Angle KickAction::getKickDirection()
{
    return kick_direction;
}

Point KickAction::getKickOrigin()
{
    return kick_origin;
}

double KickAction::getKickSpeed()
{
    return kick_speed_meters_per_second;
}

void KickAction::calculateNextIntent(IntentCoroutine::push_type &yield)
{
    // How large the triangle is that defines the region where the robot is
    // behind the ball and ready to kick.
    // We want to keep the region small enough that we won't use the KickIntent from too
    // far away (since the KickIntent doesn't avoid obstacles and we risk colliding
    // with something), but large enough we can reasonably get in the region and kick the
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
    //                     direction of kick

    do
    {
        // A vector in the direction opposite the kick (behind the ball)
        Vector behind_ball =
            Vector::createFromAngle(this->kick_direction + Angle::half());


        // The points below make up the triangle that defines the region we treat as
        // "behind the ball". They correspond to the vertices labeled 'A', 'B', and 'C'
        // in the ASCII diagram

        // We make the region close enough to the ball so that the robot will still be
        // inside it when taking the kick.
        Point behind_ball_vertex_A = kick_origin;
        Point behind_ball_vertex_B =
            behind_ball_vertex_A + behind_ball.normalize(size_of_region_behind_ball) +
            behind_ball.perpendicular().normalize(size_of_region_behind_ball / 2);
        Point behind_ball_vertex_C =
            behind_ball_vertex_A + behind_ball.normalize(size_of_region_behind_ball) -
            (behind_ball.perpendicular().normalize(size_of_region_behind_ball / 2));

        Polygon behind_ball_region =
            Polygon({behind_ball_vertex_A, behind_ball_vertex_B, behind_ball_vertex_C});

        bool robot_behind_ball = contains(behind_ball_region, robot->position());
        // The point in the middle of the region behind the ball
        Point point_behind_ball =
            kick_origin + behind_ball.normalize(size_of_region_behind_ball * 3 / 4);

        // If we're not in position to kick, move into position
        if (!robot_behind_ball)
        {
            yield(std::make_unique<MoveIntent>(
                robot->id(), point_behind_ball, kick_direction, 0.0, DribblerMode::OFF,
                BallCollisionType::AVOID, AutoChipOrKick{AutoChipOrKickMode::OFF, 0},
                MaxAllowedSpeedMode::PHYSICAL_LIMIT, 0.0, robot->robotConstants()));
        }
        else
        {
            yield(std::make_unique<KickIntent>(robot->id(), kick_origin, kick_direction,
                                               kick_speed_meters_per_second,
                                               robot->robotConstants()));
        }
    } while (!ball.hasBallBeenKicked(kick_direction));
}
