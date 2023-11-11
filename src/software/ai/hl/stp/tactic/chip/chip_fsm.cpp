#include "software/ai/hl/stp/tactic/chip/chip_fsm.h"

void ChipFSM::updateGetBehindBall(
    const Update &event, boost::sml::back::process<GetBehindBallFSM::Update> processEvent)
{
    GetBehindBallFSM::ControlParams control_params{
        .ball_location   = event.control_params.chip_origin,
        .chick_direction = event.control_params.chip_direction};

    // Update the get behind ball fsm
    processEvent(GetBehindBallFSM::Update(control_params, event.common));
}

void ChipFSM::updateChip(const Update &event)
{
    event.common.set_primitive(createMovePrimitive(
            event.common.robot,
            event.control_params.chip_origin,
            TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT,
            event.control_params.chip_direction, TbotsProto::DribblerMode::OFF,
            TbotsProto::BallCollisionType::ALLOW,
            AutoChipOrKick{AutoChipOrKickMode::AUTOCHIP,
                           event.control_params.chip_distance_meters},
            event.common.robot.robotConstants()));
}

bool ChipFSM::ballChicked(const Update &event)
{
    return event.common.world.ball().hasBallBeenKicked(
        event.control_params.chip_direction);
}
