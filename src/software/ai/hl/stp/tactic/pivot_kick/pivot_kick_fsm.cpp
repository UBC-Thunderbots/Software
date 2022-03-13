#include "software/ai/hl/stp/tactic/pivot_kick/pivot_kick_fsm.h"


void PivotKickFSM::getPossessionAndPivot(
    const Update& event, boost::sml::back::process<DribbleFSM::Update> processEvent)
{
    DribbleFSM::ControlParams control_params{
        .dribble_destination       = event.control_params.kick_origin,
        .final_dribble_orientation = event.control_params.kick_direction,
        .allow_excessive_dribbling = false};

    processEvent(DribbleFSM::Update(control_params, event.common));
}

void PivotKickFSM::kickBall(const Update& event)
{
    event.common.set_intent(std::make_unique<MoveIntent>(
        event.common.robot.id(), event.control_params.kick_origin,
        event.control_params.kick_direction, 0, TbotsProto::DribblerMode::OFF,
        TbotsProto::BallCollisionType::ALLOW, event.control_params.auto_chip_or_kick,
        TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT, 0.0, event.common.robot.robotConstants()));

    event.common.set_primitive(createMovePrimitive(
        event.control_params.kick_origin, event.control_params.kick_direction, 0,
        TbotsProto::DribblerMode::OFF, BallCollisionType::ALLOW,
        event.control_params.auto_chip_or_kick, TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT, 0.0,
        event.common.robot.robotConstants()));
}

bool PivotKickFSM::ballKicked(const Update& event)
{
    return event.common.world.ball().hasBallBeenKicked(
        event.control_params.kick_direction);
}
