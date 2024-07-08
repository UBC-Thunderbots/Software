#pragma once

#include "software/ai/hl/stp/skill/base_skill.hpp"
#include "software/ai/hl/stp/skill/keep_away/keep_away_skill_fsm.h"

/**
 * KeepAwaySkill dribbles the ball to an open area while facing away from 
 * nearby enemy robots.
 */
class KeepAwaySkill : public BaseSkill<KeepAwaySkillFSM, DribbleSkillFSM>
{
   public:
    using BaseSkill::BaseSkill;
};
