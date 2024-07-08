#pragma once

#include "software/ai/hl/stp/skill/base_skill.hpp"
#include "software/ai/hl/stp/skill/pivot_kick/pivot_kick_skill_fsm.h"

/**
 * PivotKickSkill dribbles the ball to a given kick origin, pivots around
 * that origin point to face the desired kick direction, and then kicks/chips the ball.
 */
class PivotKickSkill : public BaseSkill<PivotKickSkillFSM, DribbleSkillFSM>
{
   public:
    using BaseSkill::BaseSkill;
};
