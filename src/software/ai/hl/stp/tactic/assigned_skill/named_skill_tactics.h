#pragma once

#include "software/ai/hl/stp/skill/pivot_kick/pivot_kick_skill_fsm.h"
#include "software/ai/hl/stp/tactic/assigned_skill/assigned_skill_tactic.hpp"

class DribbleSkillTactic : public AssignedSkillTactic<DribbleSkillFSM>
{
    using AssignedSkillTactic::AssignedSkillTactic;
};

class PivotKickSkillTactic : public AssignedSkillTactic<PivotKickSkillFSM, DribbleSkillFSM>
{
    using AssignedSkillTactic::AssignedSkillTactic;
};
