#include "software/ai/hl/stp/skill/stop/stop_skill_fsm.h"

#include "software/ai/hl/stp/tactic/transition_conditions.h"

void StopSkillFSM::updateStop(const Update& event)
{
    event.common.set_primitive(std::make_unique<StopPrimitive>());
}

bool StopSkillFSM::stopDone(const Update& event)
{
    return robotStopped(event.common.robot);
}
