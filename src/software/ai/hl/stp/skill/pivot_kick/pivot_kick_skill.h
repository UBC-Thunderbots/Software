#pragma once

#include "software/ai/hl/stp/skill/base_skill.hpp"
#include "software/ai/hl/stp/skill/pivot_kick/pivot_kick_skill_fsm.h"

class PivotKickSkill : public BaseSkill<PivotKickSkillFSM, DribbleSkillFSM>
{
   public:
    using BaseSkill::BaseSkill;
};

COPY_SKILL_TACTIC(WallKickoffSkillTactic, PivotKickSkill)