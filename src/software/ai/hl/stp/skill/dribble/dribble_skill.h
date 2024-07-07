#pragma once

#include "software/ai/hl/stp/skill/base_skill.hpp"
#include "software/ai/hl/stp/skill/dribble/dribble_skill_fsm.h"

class DribbleSkill : public BaseSkill<DribbleSkillFSM>
{
   public:
    using BaseSkill::BaseSkill;
};
