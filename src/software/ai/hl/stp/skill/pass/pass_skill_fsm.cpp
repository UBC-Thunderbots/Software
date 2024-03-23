#include "software/ai/hl/stp/skill/pass/pass_skill_fsm.h"

void PassSkillFSM::takePass(
    const Update& event,
    boost::sml::back::process<PivotKickSkillFSM::Update> processEvent)
{
    Point ball_position = event.common.world_ptr->ball().position();

    Pass best_pass = (*event.common.strategy)->getBestCommittedPass();

    processEvent(PivotKickSkillFSM::Update(
        PivotKickSkillFSM::ControlParams{
            .kick_origin    = ball_position,
            .kick_direction = best_pass.passerOrientation(),
            .auto_chip_or_kick =
                AutoChipOrKick{AutoChipOrKickMode::AUTOKICK, best_pass.speed()}},
        event.common));
}
