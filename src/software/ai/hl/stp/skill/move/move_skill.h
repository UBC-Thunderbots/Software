#pragma once

#include "software/ai/hl/stp/skill/base_skill.hpp"
#include "software/ai/hl/stp/skill/move/move_skill_fsm.h"

/**
 * MoveSkill will move the robot to the given destination and make it arrive
 * with the specified final orientation and speed
 */
class MoveSkill : public BaseSkill<MoveSkillFSM>
{
   public:
    using BaseSkill::BaseSkill;
};
