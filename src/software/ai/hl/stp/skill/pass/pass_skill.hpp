#pragma once

#include "software/ai/hl/stp/skill/base_skill.hpp"
#include "software/ai/hl/stp/skill/pass/pass_skill_fsm.h"

/**
 * Base class template for a PassSkill.
 *
 * PassSkill finds the best pass on the field and takes the pass by either
 * kicking or chipping the ball.
 *
 * @tparam SHOULD_CHIP whether the PassSkill should chip (true) or kick (false) the ball
 */
template <bool SHOULD_CHIP>
class BasePassSkill : public BaseSkill<PassSkillFSM, DribbleSkillFSM, PivotKickSkillFSM>
{
   public:
    using BaseSkill::BaseSkill;

    void updatePrimitive(const Robot& robot, const WorldPtr& world_ptr,
                         const SetPrimitiveCallback& set_primitive) override;
};

/**
 * PassSkill that will take the pass by kicking the ball.
 */
class KickPassSkill : public BasePassSkill<false>
{
    using BasePassSkill::BasePassSkill;
};

/**
 * PassSkill that will take the pass by chipping the ball.
 */
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
