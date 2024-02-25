#include "software/ai/hl/stp/skill/shoot/shoot_skill_fsm.h"

void ShootSkillFSM::pivotKick(
    const Update& event,
    boost::sml::back::process<PivotKickSkillFSM::Update> processEvent)
{
    Point ball_position = event.common.world_ptr->ball().position();
    Point kick_target   = event.common.world_ptr->field().enemyGoalCenter();

    std::optional<Shot> best_shot =
        (*event.common.strategy)->getBestShot(event.common.robot);
    if (best_shot)
    {
        kick_target = best_shot->getPointToShootAt();
    }

    processEvent(PivotKickSkillFSM::Update(
        PivotKickSkillFSM::ControlParams{
            .kick_origin       = ball_position,
            .kick_direction    = (kick_target - ball_position).orientation(),
            .auto_chip_or_kick = AutoChipOrKick{AutoChipOrKickMode::AUTOKICK,
                                                BALL_MAX_SPEED_METERS_PER_SECOND - 0.5}},
        event.common));
}
