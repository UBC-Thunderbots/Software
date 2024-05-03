#include "software/ai/hl/stp/skill/chip/chip_skill_fsm.h"

#include "software/ai/hl/stp/primitive/move_primitive.h"

void ChipSkillFSM::updateGetBehindBall(
    const Update &event,
    boost::sml::back::process<GetBehindBallSkillFSM::Update> processEvent)
{
    GetBehindBallSkillFSM::ControlParams control_params{
        .ball_location   = event.control_params.chip_origin,
        .chick_direction = event.control_params.chip_direction};

    // Update the get behind ball fsm
    processEvent(GetBehindBallSkillFSM::Update(
        control_params, SkillUpdate(event.common.robot, event.common.world_ptr,
                                    event.common.strategy, event.common.set_primitive)));
}

void ChipSkillFSM::updateChip(const Update &event)
{
    event.common.set_primitive(std::make_unique<MovePrimitive>(
        event.common.robot, event.control_params.chip_origin,
        event.control_params.chip_direction,
        TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT,
        TbotsProto::ObstacleAvoidanceMode::SAFE, TbotsProto::DribblerMode::OFF,
        TbotsProto::BallCollisionType::ALLOW,
        AutoChipOrKick{AutoChipOrKickMode::AUTOCHIP,
                       event.control_params.chip_distance_meters}));
}

bool ChipSkillFSM::ballChicked(const Update &event)
{
    return event.common.world_ptr->ball().hasBallBeenKicked(
        event.control_params.chip_direction);
}
