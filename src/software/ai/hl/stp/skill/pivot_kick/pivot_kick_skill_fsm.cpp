#include "software/ai/hl/stp/skill/pivot_kick/pivot_kick_skill_fsm.h"

#include "software/ai/hl/stp/primitive/move_primitive.h"

void PivotKickSkillFSM::getPossessionAndPivot(
    const Update& event, boost::sml::back::process<DribbleSkillFSM::Update> processEvent)
{
    DribbleSkillFSM::ControlParams control_params{
        .dribble_destination       = event.control_params.kick_origin,
        .final_dribble_orientation = event.control_params.kick_direction,
        .allow_excessive_dribbling = false};

    processEvent(DribbleSkillFSM::Update(control_params, event.common));
}

void PivotKickSkillFSM::kickBall(const Update& event)
{
    event.common.set_primitive(std::make_unique<MovePrimitive>(
        event.common.robot, event.control_params.kick_origin,
        event.control_params.kick_direction,
        TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT, TbotsProto::DribblerMode::OFF,
        TbotsProto::BallCollisionType::ALLOW, event.control_params.auto_chip_or_kick));
}

bool PivotKickSkillFSM::lostPossession(const Update& event)
{
    const TbotsProto::DribbleSkillConfig& dribble_skill_config =
        event.common.strategy->getAiConfig().dribble_skill_config();

    return !event.common.robot.isNearDribbler(
        event.common.world_ptr->ball().position(),
        dribble_skill_config.lose_ball_possession_threshold());
}

bool PivotKickSkillFSM::ballKicked(const Update& event)
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
