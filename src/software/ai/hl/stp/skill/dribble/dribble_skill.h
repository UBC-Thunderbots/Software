#pragma once

#include "software/ai/hl/stp/skill/base_skill.hpp"
#include "software/ai/hl/stp/skill/dribble/dribble_skill_fsm.h"

/**
 * DribbleSkill dribbles the ball to a given location on the field.
 */
class DribbleSkill : public BaseSkill<DribbleSkillFSM>
{
   public:
    using BaseSkill::BaseSkill;
};
