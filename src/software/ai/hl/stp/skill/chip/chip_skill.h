#pragma once

#include "software/ai/hl/stp/skill/base_skill.hpp"
#include "software/ai/hl/stp/skill/chip/chip_skill_fsm.h"

/**
 * In ChipSkill, the robot gets behind the ball, aligns itself in the direction of the
 * desired kick, and drives into the ball to chip it.
 */
class ChipSkill : public BaseSkill<ChipSkillFSM, GetBehindBallSkillFSM>
{
   public:
    using BaseSkill::BaseSkill;
};
