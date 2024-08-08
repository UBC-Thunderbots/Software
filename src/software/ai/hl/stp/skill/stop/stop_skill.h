#pragma once

#include "software/ai/hl/stp/skill/base_skill.hpp"
#include "software/ai/hl/stp/skill/stop/stop_skill_fsm.h"

/**
 * StopSkill will stop the robot from moving.
 * The robot will actively try and brake to come to a halt.
 */
class StopSkill : public BaseSkill<StopSkillFSM>
{
   public:
    using BaseSkill::BaseSkill;
};
