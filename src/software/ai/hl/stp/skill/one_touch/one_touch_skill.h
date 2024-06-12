#pragma once

#include "software/ai/hl/stp/skill/base_skill.hpp"
#include "software/ai/hl/stp/skill/one_touch/one_touch_skill_fsm.h"

class OneTouchSkill : public BaseSkill<OneTouchSkillFSM, DribbleSkillFSM>
{
   public:
    explicit OneTouchSkill(std::shared_ptr<Strategy> strategy);
};
