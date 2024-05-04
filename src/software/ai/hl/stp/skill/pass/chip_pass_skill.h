#pragma once

#include "software/ai/hl/stp/skill/base_skill.hpp"
#include "software/ai/hl/stp/skill/pass/pass_skill_fsm.h"

class ChipPassSkill : public BaseSkill<PassSkillFSM, DribbleSkillFSM, PivotKickSkillFSM>
{
   public:
    explicit ChipPassSkill(std::shared_ptr<Strategy> strategy);

    void updatePrimitive(const Robot& robot, const WorldPtr& world_ptr,
                         const SetPrimitiveCallback& set_primitive) override;

    void accept(SkillVisitor& skill_visitor) override;
};
