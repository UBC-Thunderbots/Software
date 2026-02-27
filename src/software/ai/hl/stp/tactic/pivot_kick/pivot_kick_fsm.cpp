#include "software/ai/hl/stp/tactic/pivot_kick/pivot_kick_fsm.h"

#include "software/ai/hl/stp/tactic/move_primitive.h"
#include "software/logger/logger.h"

PivotKickFSM::PivotKickFSM(std::shared_ptr<const TbotsProto::AiConfig> ai_config_ptr)
    : TacticFSM<PivotKickFSM>(ai_config_ptr)
{
}

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
        TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT,
        TbotsProto::ObstacleAvoidanceMode::AGGRESSIVE, TbotsProto::DribblerMode::OFF,
        TbotsProto::BallCollisionType::ALLOW, event.control_params.auto_chip_or_kick));
}

bool PivotKickFSM::ballKicked(const Update& event)
{
    if (event.control_params.auto_chip_or_kick.auto_chip_kick_mode ==
        AutoChipOrKickMode::AUTOKICK)
    {
        bool kicked = event.common.world_ptr->ball().hasBallBeenKicked(
            event.control_params.kick_direction);
        LOG(WARNING) << "using hasBallBeenKicked check " << kicked;
        return kicked;
    }
    else
    {
        // check for separation for chipping since kick angle is not reliable
        bool kicked = !event.common.robot.isNearDribbler(
            event.common.world_ptr->ball().position(), ROBOT_MAX_RADIUS_METERS);
        LOG(WARNING) << "using isNearDribbler check " << kicked;
        return kicked;
    }
}
