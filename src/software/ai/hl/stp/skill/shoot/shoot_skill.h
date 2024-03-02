#pragma once

#include "software/ai/hl/stp/skill/base_skill.hpp"
#include "software/ai/hl/stp/skill/shoot/shoot_skill_fsm.h"
#include "software/util/generic_factory/generic_factory.h"

class ShootSkill : public BaseSkill<ShootSkillFSM, DribbleSkillFSM>
{
   public:
    explicit ShootSkill(std::shared_ptr<Strategy> strategy) : BaseSkill(strategy){};
};

// Register this Skill in the GenericFactory
static TGenericFactory<std::string, Skill, ShootSkill, std::shared_ptr<Strategy>> factory;
