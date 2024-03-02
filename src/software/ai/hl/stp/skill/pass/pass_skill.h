#pragma once

#include "software/ai/hl/stp/skill/base_skill.hpp"

class PassSkill : public BaseSkill<PassSkillFSM>
{
   public:
    explicit PassSkill(std::shared_ptr<Strategy> strategy) : BaseSkill(strategy){};
};

// Register this Skill in the GenericFactory
static TGenericFactory<std::string, Skill, PassSkill, std::shared_ptr<Strategy>> factory;
