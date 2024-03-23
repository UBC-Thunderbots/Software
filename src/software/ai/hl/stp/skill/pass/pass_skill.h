#pragma once

#include "software/ai/hl/stp/skill/base_skill.hpp"
#include "software/ai/hl/stp/skill/dribble/dribble_skill_fsm.h"
#include "software/ai/hl/stp/skill/pass/pass_skill_fsm.h"
#include "software/ai/hl/stp/skill/pivot_kick/pivot_kick_skill_fsm.h"

class PassSkill : public BaseSkill<PassSkillFSM, DribbleSkillFSM, PivotKickSkillFSM>
{
   public:
    explicit PassSkill(std::shared_ptr<Strategy> strategy);

    void accept(SkillVisitor& skill_visitor) override;
};
