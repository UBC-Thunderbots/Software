#pragma once

#include "software/ai/hl/stp/skill/base_skill.hpp"
#include "software/ai/hl/stp/skill/get_behind_ball/get_behind_ball_skill_fsm.h"

/**
 * GetBehindBallSkill moves the robot so that it is behind the ball and facing
 * the desired direction. 
 */
class GetBehindBallSkill : public BaseSkill<GetBehindBallSkillFSM>
{
   public:
    using BaseSkill::BaseSkill;
};
