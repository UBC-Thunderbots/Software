#pragma once

#include "software/ai/hl/stp/skill/base_skill.hpp"
#include "software/ai/hl/stp/skill/get_behind_ball/get_behind_ball_skill_fsm.h"

class GetBehindBallSkill : public BaseSkill<GetBehindBallSkillFSM>
{
   public:
    using BaseSkill::BaseSkill;
};
