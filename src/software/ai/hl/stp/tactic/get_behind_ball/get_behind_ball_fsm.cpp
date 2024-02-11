#include "software/ai/hl/stp/tactic/get_behind_ball/get_behind_ball_fsm.h"

#include "software/ai/hl/stp/tactic/move_primitive.h"


GetBehindBallFSM::GetBehindBallFSM()
    : size_of_region_behind_ball(3 * ROBOT_MAX_RADIUS_METERS)
{
}

void GetBehindBallFSM::updateMove(const Update& event)
{
    Vector behind_ball =
        Vector::createFromAngle(event.control_params.chick_direction + Angle::half());
    Point point_behind_ball = event.control_params.ball_location +
                              behind_ball.normalize(ROBOT_MAX_RADIUS_METERS + 0.04);

    event.common.set_primitive(std::make_unique<MovePrimitive>(
        event.common.robot, point_behind_ball, event.control_params.chick_direction,
        TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT, TbotsProto::DribblerMode::OFF,
        TbotsProto::BallCollisionType::AVOID,
        AutoChipOrKick{AutoChipOrKickMode::OFF, 0}));
}

bool GetBehindBallFSM::behindBall(const Update& event)
{
    // A vector in the direction opposite the chip (behind the ball)
    Vector behind_ball =
        Vector::createFromAngle(event.control_params.chick_direction + Angle::half());


    // The points below make up the triangle that defines the region we treat as
    // "behind the ball". They correspond to the vertices labeled 'A', 'B', and
    // 'C' in the ASCII diagram

    // We make the region close enough to the ball so that the robot will still be
    // inside it when taking the chip.
    Point behind_ball_vertex_A = event.control_params.ball_location +
        behind_ball.normalize(0.02 + ROBOT_MAX_RADIUS_METERS);
    Point behind_ball_vertex_A1 =
        behind_ball_vertex_A +
        behind_ball.perpendicular().normalize(0.03 / 2);
    Point behind_ball_vertex_A2 =
        behind_ball_vertex_A -
        behind_ball.perpendicular().normalize(0.03 / 2);
    Point behind_ball_vertex_B =
        behind_ball_vertex_A + behind_ball.normalize(0.06) +
        behind_ball.perpendicular().normalize(0.04 / 2);
    Point behind_ball_vertex_C =
        behind_ball_vertex_A + behind_ball.normalize(0.06) -
        behind_ball.perpendicular().normalize(0.04 / 2);

    Polygon behind_ball_region = Polygon({behind_ball_vertex_A2, behind_ball_vertex_A1,
                                          behind_ball_vertex_B, behind_ball_vertex_C});

    std::cout << behind_ball_region << std::endl;
    std::cout << event.control_params.ball_location << std::endl;
    std::cout << event.common.robot.position() << std::endl;
    std::cout << contains(behind_ball_region, event.common.robot.position()) << std::endl;
    std::cout << compareAngles(event.common.robot.orientation(),
                         event.control_params.chick_direction, Angle::fromDegrees(4)) << std::endl;
    return contains(behind_ball_region, event.common.robot.position()) &&
           compareAngles(event.common.robot.orientation(),
                         event.control_params.chick_direction, Angle::fromDegrees(4));
}
