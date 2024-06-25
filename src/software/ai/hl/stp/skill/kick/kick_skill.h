#pragma once

#include "software/ai/hl/stp/skill/base_skill.hpp"
#include "software/ai/hl/stp/skill/kick/kick_skill_fsm.h"

class KickSkill : public BaseSkill<KickSkillFSM, GetBehindBallSkillFSM>
{
   public:
    using BaseSkill::BaseSkill;
};
