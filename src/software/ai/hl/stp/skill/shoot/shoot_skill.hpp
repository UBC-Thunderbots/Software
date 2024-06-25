#pragma once

#include "software/ai/hl/stp/skill/base_skill.hpp"
#include "software/ai/hl/stp/skill/shoot/shoot_skill_fsm.h"

template <bool SAMPLE_FOR_BEST_SHOT>
class BaseShootSkill : public BaseSkill<ShootSkillFSM, ShootSkillFSM::GetBallControlFSM,
                                    DribbleSkillFSM, PivotKickSkillFSM>
{
   public:
    using BaseSkill::BaseSkill;

    void updatePrimitive(const Robot& robot, const WorldPtr& world_ptr,
                         const SetPrimitiveCallback& set_primitive) override;
};

using ShootSkill = BaseShootSkill<false>;
using DribbleShootSkill = BaseShootSkill<true>;

template <bool SAMPLE_FOR_BEST_SHOT>
void BaseShootSkill<SAMPLE_FOR_BEST_SHOT>::updatePrimitive(const Robot& robot,
                                             const WorldPtr& world_ptr,
                                             const SetPrimitiveCallback& set_primitive)
{
    control_params_ = {.sample_for_best_shot = SAMPLE_FOR_BEST_SHOT};
    BaseSkill::updatePrimitive(robot, world_ptr, set_primitive);
}
