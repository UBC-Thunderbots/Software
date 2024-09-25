#pragma once

#include "software/ai/hl/stp/skill/base_skill.hpp"
#include "software/ai/hl/stp/skill/shoot/shoot_skill_fsm.h"

/**
 * Base class template for a ShootSkill.
 *
 * ShootSkill finds the best shot on the enemy goal to take, dribbles the ball to the
 * shot's origin point, and takes the shot by kicking the ball.
 *
 * When finding the best shot to take, ShootSkill can either use the ball's current
 * position as the shot origin point, or it can sample multiple potential shot origin
 * points nearby the ball and select the best shot.
 *
 * @tparam SAMPLE_FOR_BEST_SHOT whether to sample multiple potential shot origin
 * points nearby the ball (true), or use the ball's current position as the shot origin
 * point (false)
 */
template <bool SAMPLE_FOR_BEST_SHOT>
class BaseShootSkill : public BaseSkill<ShootSkillFSM, DribbleSkillFSM, PivotKickSkillFSM>
{
   public:
    using BaseSkill::BaseSkill;

    void updatePrimitive(const Robot& robot, const WorldPtr& world_ptr,
                         const SetPrimitiveCallback& set_primitive) override;
};

/**
 * ShootSkill that uses the ball's current position as the shot origin point.
 */
class ShootSkill : public BaseShootSkill<false>
{
    using BaseShootSkill::BaseShootSkill;
};

/**
 * ShootSkill that samples multiple potential shot origin points nearby the ball
 * and selects the best shot to take.
 */
class DribbleShootSkill : public BaseShootSkill<true>
{
    using BaseShootSkill::BaseShootSkill;
};

template <bool SAMPLE_FOR_BEST_SHOT>
void BaseShootSkill<SAMPLE_FOR_BEST_SHOT>::updatePrimitive(
    const Robot& robot, const WorldPtr& world_ptr,
    const SetPrimitiveCallback& set_primitive)
{
    control_params_ = {.sample_for_best_shot = SAMPLE_FOR_BEST_SHOT};
    BaseSkill::updatePrimitive(robot, world_ptr, set_primitive);
}
