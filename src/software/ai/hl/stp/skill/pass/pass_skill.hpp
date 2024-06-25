#pragma once

#include "software/ai/hl/stp/skill/base_skill.hpp"
#include "software/ai/hl/stp/skill/pass/pass_skill_fsm.h"

template <bool SHOULD_CHIP>
class PassSkill : public BaseSkill<PassSkillFSM, DribbleSkillFSM, PivotKickSkillFSM>
{
   public:
    using BaseSkill::BaseSkill;

    void updatePrimitive(const Robot& robot, const WorldPtr& world_ptr,
                         const SetPrimitiveCallback& set_primitive) override;
};

using KickPassSkill = PassSkill<false>;
using ChipPassSkill = PassSkill<true>;

template <bool SHOULD_CHIP>
void PassSkill<SHOULD_CHIP>::updatePrimitive(const Robot& robot,
                                             const WorldPtr& world_ptr,
                                             const SetPrimitiveCallback& set_primitive)
{
    control_params_ = {.should_chip = SHOULD_CHIP};
    BaseSkill::updatePrimitive(robot, world_ptr, set_primitive);
}
