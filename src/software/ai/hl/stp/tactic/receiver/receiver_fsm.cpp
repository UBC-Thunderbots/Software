#include "software/ai/hl/stp/tactic/receiver/receiver_fsm.h"

#include "software/ai/hl/stp/primitive/move_primitive.h"

void ReceiverFSM::updateReceive(const Update& event)
{
    if (event.control_params.receiver_point)
    {
        const Ball& ball           = event.common.world_ptr->ball();
        const Point receiver_point = event.control_params.receiver_point.value();
        const Angle receiver_orientation =
            (ball.position() - receiver_point).orientation();

        event.common.set_primitive(std::make_unique<MovePrimitive>(
            event.common.robot, receiver_point, receiver_orientation,
            TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT,
            TbotsProto::ObstacleAvoidanceMode::AGGRESSIVE,
            TbotsProto::DribblerMode::MAX_FORCE, TbotsProto::BallCollisionType::ALLOW,
            AutoChipOrKick{AutoChipOrKickMode::OFF, 0}));
    }
}

void ReceiverFSM::adjustReceive(const Update& event)
{
    auto ball      = event.common.world_ptr->ball();
    auto robot_pos = event.common.robot.position();

    if ((ball.position() - robot_pos).length() >
        BALL_TO_FRONT_OF_ROBOT_DISTANCE_WHEN_DRIBBLING)
    {
        Point ball_receive_pos = ball.position();

        if (ball.velocity().length() > MIN_PASS_START_SPEED)
        {
            ball_receive_pos = closestPoint(
                robot_pos, Line(ball.position(), ball.position() + ball.velocity()));
        }

        Angle ball_receive_orientation = (ball.position() - robot_pos).orientation();

        event.common.set_primitive(std::make_unique<MovePrimitive>(
            event.common.robot, ball_receive_pos, ball_receive_orientation,
            TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT,
            TbotsProto::ObstacleAvoidanceMode::AGGRESSIVE,
            TbotsProto::DribblerMode::MAX_FORCE, TbotsProto::BallCollisionType::ALLOW,
            AutoChipOrKick{AutoChipOrKickMode::OFF, 0}));
    }
}

bool ReceiverFSM::passStarted(const Update& event)
{
    const Ball& ball           = event.common.world_ptr->ball();
    const Point receiver_point = event.control_params.receiver_point.value();

    return ball.hasBallBeenKicked((receiver_point - ball.position()).orientation());
}

bool ReceiverFSM::passReceived(const Update& event)
{
    return event.common.robot.isNearDribbler(event.common.world_ptr->ball().position());
}

bool ReceiverFSM::passReceivedByTeammate(const Update& event)
{
    auto friendly_robots =
        event.common.world_ptr->friendlyTeam().getAllRobotsExcept({event.common.robot});

    return std::any_of(
        friendly_robots.begin(), friendly_robots.end(), [&](const Robot& robot) {
            return robot.isNearDribbler(event.common.world_ptr->ball().position());
        });
}
