#pragma once

#include "software/ai/hl/stp/skill/base_skill.hpp"
#include "software/ai/hl/stp/skill/kick/kick_skill_fsm.h"

/**
 * In KickSkill, the robot gets behind the ball, aligns itself in the direction of the
 * desired kick, and drives into the ball to kick it.
 */
class KickSkill : public BaseSkill<KickSkillFSM, GetBehindBallSkillFSM>
{
   public:
    using BaseSkill::BaseSkill;
};
