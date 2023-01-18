#include "software/ai/hl/stp/tactic/pass_defender/pass_defender_fsm.h"

#include "software/geom/algorithms/closest_point.h"

bool PassDefenderFSM::passStarted(const Update& event)
{
    auto ball_position = event.common.world.ball().position();
    Vector ball_receiver_point_vector(
        event.control_params.position_to_block_from.x() - ball_position.x(),
        event.control_params.position_to_block_from.y() - ball_position.y());

    return event.common.world.ball().hasBallBeenKicked(
        ball_receiver_point_vector.orientation());
}

bool PassDefenderFSM::strayPass(const Update& event)
{
    auto ball_position = event.common.world.ball().position();
    Vector ball_receiver_point_vector(
        event.control_params.position_to_block_from.x() - ball_position.x(),
        event.control_params.position_to_block_from.y() - ball_position.y());

    auto orientation_difference = event.common.world.ball().velocity().orientation() -
                                  ball_receiver_point_vector.orientation();

    // If pass has strayed far from its intended destination (ex it was deflected)
    // we consider the pass finished
    bool stray_pass =
        event.common.world.ball().velocity().length() > MIN_STRAY_PASS_SPEED &&
        orientation_difference > MIN_STRAY_PASS_ANGLE;

    return stray_pass;
}

void PassDefenderFSM::blockPass(const Update &event)
{
    auto position_to_block_from = event.control_params.position_to_block_from;
    auto ball_position = event.common.world.ball().position();
    auto face_ball_orientation =
        (ball_position - event.common.robot.position()).orientation();
    
    event.common.set_primitive(createMovePrimitive(
        CREATE_MOTION_CONTROL(position_to_block_from), face_ball_orientation, 0,
        TbotsProto::DribblerMode::OFF, TbotsProto::BallCollisionType::AVOID,
        AutoChipOrKick{AutoChipOrKickMode::OFF, 0},
        TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT, 0.0,
        event.common.robot.robotConstants()));
}

void PassDefenderFSM::interceptBall(const Update &event)
{
    auto ball = event.common.world.ball();
    auto robot_position = event.common.robot.position();

    if ((ball.position() - robot_position).length() >
        BALL_TO_FRONT_OF_ROBOT_DISTANCE_WHEN_DRIBBLING)
    {
        auto intercept_position = closestPoint(
                robot_position, Line(ball.position(), ball.position() + ball.velocity()));

        auto face_ball_orientation = (ball.position() - robot_position).orientation();

        event.common.set_primitive(createMovePrimitive(
            CREATE_MOTION_CONTROL(intercept_position), face_ball_orientation, 0,
            TbotsProto::DribblerMode::MAX_FORCE, TbotsProto::BallCollisionType::ALLOW,
            AutoChipOrKick{AutoChipOrKickMode::AUTOCHIP, YEET_CHIP_DISTANCE_METERS},
            TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT, 0.0,
            event.common.robot.robotConstants()));
    }
}
