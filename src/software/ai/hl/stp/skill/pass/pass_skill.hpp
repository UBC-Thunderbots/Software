#pragma once

#include "software/ai/hl/stp/skill/base_skill.hpp"
#include "software/ai/hl/stp/skill/pass/pass_skill_fsm.h"

template <bool SHOULD_CHIP>
class BasePassSkill : public BaseSkill<PassSkillFSM, DribbleSkillFSM, PivotKickSkillFSM>
{
   public:
    using BaseSkill::BaseSkill;

    void updatePrimitive(const Robot& robot, const WorldPtr& world_ptr,
                         const SetPrimitiveCallback& set_primitive) override;
};

class KickPassSkill : public BasePassSkill<false>
{
    using BasePassSkill::BasePassSkill;
};

class ChipPassSkill : public BasePassSkill<true>
{
    using BasePassSkill::BasePassSkill;
};

template <bool SHOULD_CHIP>
void BasePassSkill<SHOULD_CHIP>::updatePrimitive(
    const Robot& robot, const WorldPtr& world_ptr,
    const SetPrimitiveCallback& set_primitive)
{
    control_params_ = {.should_chip = SHOULD_CHIP};
    BaseSkill::updatePrimitive(robot, world_ptr, set_primitive);
}
