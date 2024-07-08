#pragma once

#include "software/ai/hl/stp/skill/base_skill.hpp"
#include "software/ai/hl/stp/skill/chip/chip_skill_fsm.h"

class ChipSkill : public BaseSkill<ChipSkillFSM, GetBehindBallSkillFSM>
{
   public:
    using BaseSkill::BaseSkill;
};
