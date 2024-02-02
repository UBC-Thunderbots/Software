#include "software/ai/hl/stp/tactic/pivot_kick/pivot_kick_fsm.h"

#include "software/ai/hl/stp/tactic/move_primitive.h"


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
    event.common.set_primitive(std::make_unique<MovePrimitive>(
        event.common.robot, event.control_params.kick_origin,
        event.control_params.kick_direction,
        TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT, TbotsProto::DribblerMode::OFF,
        TbotsProto::BallCollisionType::ALLOW, event.control_params.auto_chip_or_kick));
}

bool PivotKickFSM::ballKicked(const Update& event)
{
    if (event.control_params.auto_chip_or_kick.auto_chip_kick_mode ==
        AutoChipOrKickMode::AUTOKICK)
    {
        return event.common.world_ptr->ball().hasBallBeenKicked(
            event.control_params.kick_direction);
    }
    else
    {
        // check for separation for chipping since kick angle is not reliable
        return !event.common.robot.isNearDribbler(
            event.common.world_ptr->ball().position(), ROBOT_MAX_RADIUS_METERS);
    }
}
