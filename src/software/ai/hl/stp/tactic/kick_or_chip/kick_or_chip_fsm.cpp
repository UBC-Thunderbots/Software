#include "software/ai/hl/stp/tactic/kick_or_chip/kick_or_chip_fsm.h"

#include "software/ai/hl/stp/tactic/move_primitive.h"

KickOrChipFSM::KickOrChipFSM(std::shared_ptr<const TbotsProto::AiConfig> ai_config_ptr)
    : TacticFSM<KickOrChipFSM>(ai_config_ptr)
{
}

void KickOrChipFSM::kickOrChipBall(const Update& event)
{
    event.common.set_primitive(std::make_unique<MovePrimitive>(
        event.common.robot, event.control_params.kick_or_chip_origin,
        event.control_params.kick_or_chip_direction,
        TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT,
        TbotsProto::ObstacleAvoidanceMode::AGGRESSIVE, TbotsProto::DribblerMode::OFF,
        TbotsProto::BallCollisionType::ALLOW, event.control_params.auto_chip_or_kick));
}

void KickOrChipFSM::updateGetBehindBall(
    const Update &event, boost::sml::back::process<GetBehindBallFSM::Update> processEvent)
{
    GetBehindBallFSM::ControlParams control_params{
        .ball_location   = event.control_params.kick_or_chip_origin,
        .chick_direction = event.control_params.kick_or_chip_direction};

    // Update the get behind ball fsm
    processEvent(GetBehindBallFSM::Update(control_params, event.common));
}

bool KickOrChipFSM::ballChicked(const Update &event)
{
    return event.common.world_ptr->ball().hasBallBeenKicked(
        event.control_params.kick_or_chip_direction);
}

bool KickOrChipFSM::shouldRealignWithBall(const Update &event)
{
    const Robot &robot = event.common.robot;

    // First check to see if it's too late to realign with the ball
    if (robot.isNearDribbler(event.control_params.kick_or_chip_origin, 0.05))
    {
        return false;
    }

    // Check if the robot is already aligned to kick the ball
    return !isRobotReadyToChick(robot, event.control_params.kick_or_chip_origin,
                                event.control_params.kick_or_chip_direction);
}

