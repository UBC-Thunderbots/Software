#pragma once

#include "software/ai/hl/stp/skill/base_skill.hpp"

#include "software/ai/hl/stp/skill/dribble/dribble_skill_fsm.h"
#include "software/ai/hl/stp/skill/pivot_kick/pivot_kick_skill_fsm.h"
#include "software/ai/hl/stp/skill/shoot/shoot_skill_fsm.h"

class ShootSkill : public BaseSkill<ShootSkillFSM, DribbleSkillFSM, PivotKickSkillFSM>
{
   public:
    explicit ShootSkill(std::shared_ptr<Strategy> strategy);

    void accept(SkillVisitor& visitor);
};
