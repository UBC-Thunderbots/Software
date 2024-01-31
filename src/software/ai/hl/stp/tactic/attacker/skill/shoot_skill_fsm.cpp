#include "software/ai/hl/stp/tactic/attacker/skill/shoot_skill_fsm.h"

bool ShootSkillFSM::done() const
{
    return fsm.is(boost::sml::X);
}

void ShootSkillFSM::updatePrimitive(const TacticUpdate& tactic_update)
{
    control_params.shot = (*strategy_)->getBestShot(tactic_update.robot);
    fsm.process_event(AttackerFSM::Update(control_params, tactic_update));
}